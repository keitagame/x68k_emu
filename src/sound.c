/* sound.c - YM2151 (OPM) FM synthesiser, MSM6258 ADPCM and the 44.1 kHz mixer.
 * Original implementation (no table data copied from other emulators; tables are
 * generated at start-up with a small built-in math library). */
#include "x68k.h"

#define OPM_CLOCK   4000000.0
#define OPM_RATE    62500            /* OPM_CLOCK / 64 */
#define OUT_RATE    44100

/* ---------------------------------------------------------- tiny math -- */
static double m_abs(double x) { return x < 0 ? -x : x; }
static double m_sin(double x) {                 /* x in radians */
    const double PI = 3.14159265358979323846;
    while (x > PI) x -= 2 * PI; while (x < -PI) x += 2 * PI;
    double x2 = x * x, t = x, s = x;
    for (int i = 1; i < 12; i++) { t *= -x2 / ((2 * i) * (2 * i + 1)); s += t; }
    return s;
}
static double m_exp(double x) {
    int k = 0; while (m_abs(x) > 0.5) { x *= 0.5; k++; }
    double t = 1, s = 1; for (int i = 1; i < 16; i++) { t *= x / i; s += t; }
    while (k--) s *= s;
    return s;
}
static double m_pow2(double x) { return m_exp(x * 0.69314718055994530942); }

/* ----------------------------------------------------------- tables --- */
static int16_t sin_tab[1024];        /* signed, +-4095 */
static uint16_t att_lin[1024 + 256]; /* 4095 * 2^(-att*0.09375/6.0206) */
static uint32_t dt2_mul[4];          /* cents -> ratio in 16.16 */
static uint32_t pitch_tab[12 * 8 * 64];   /* inc per OPM sample, 2^32 = 1 cycle; index (oct*12+note)*64+kf */
static uint8_t tables_ready;
static const uint8_t eg_pat[4][8] = {{0,1,0,1,0,1,0,1},{0,1,0,1,1,1,0,1},{0,1,1,1,0,1,1,1},{0,1,1,1,1,1,1,1}};
static const uint8_t eg_pat2[4][8] = {{1,1,1,1,1,1,1,1},{1,1,1,2,1,1,1,2},{1,2,1,2,1,2,1,2},{1,2,2,2,1,2,2,2}};
static const uint8_t note_idx[16] = {0,1,2,2,3,4,5,5,6,7,8,8,9,10,11,11};

static void make_tables(void) {
    if (tables_ready) return;
    const double PI = 3.14159265358979323846;
    for (int i = 0; i < 1024; i++) {
        double v = m_sin((i + 0.5) * 2 * PI / 1024.0) * 4095.0;
        sin_tab[i] = (int16_t)(v < 0 ? v - 0.5 : v + 0.5);
    }
    for (int a = 0; a < 1024 + 256; a++) att_lin[a] = (uint16_t)(4095.0 * m_pow2(-(a * 0.09375) / 6.0206) + 0.5);
    static const int cents[4] = {0, 600, 781, 950};
    for (int i = 0; i < 4; i++) dt2_mul[i] = (uint32_t)(m_pow2(cents[i] / 1200.0) * 65536.0);
    /* frequency of semitone s (C# of octave 0 = 0) with A4 (octave 4, note index 8 -> s=56) = 440 Hz */
    for (int s = 0; s < 12 * 8; s++)
        for (int kf = 0; kf < 64; kf++) {
            double f = 440.0 * m_pow2((s + kf / 64.0 - 56.0) / 12.0);
            double inc = f / (3579545.0 / 64.0) * 4294967296.0;
            pitch_tab[s * 64 + kf] = (uint32_t)(inc > 4294967295.0 ? 4294967295.0 : inc);
        }
    tables_ready = 1;
}

/* ------------------------------------------------------------- OPM ---- */
enum { EG_ATT, EG_DEC1, EG_DEC2, EG_REL, EG_OFF };
typedef struct {
    uint8_t dt1, mul, tl, ks, ar, ams_en, d1r, dt2, d2r, d1l, rr;
    uint8_t state, key;
    int32_t env;            /* 0..1023 attenuation */
    uint32_t phase, inc;
    int32_t out, prev;
    uint8_t rate_ks;        /* keycode >> (3 - ks) */
} Op;
typedef struct {
    Op op[4];               /* register slot order: M1, M2, C1, C2 */
    uint8_t rl, fb, con, kc, kf, pms, ams;
    int32_t fb_out[2];
    uint32_t base_inc;
} Chan;
static struct {
    Chan ch[8];
    uint8_t regs[256], addr;
    uint8_t lfrq, pmd, amd, lfo_w, noise_en, nfrq;
    uint32_t lfo_cnt, lfo_phase;
    int lfo_am, lfo_pm;
    uint32_t noise_cnt, noise_lfsr;
    int noise_out;
    uint32_t eg_cnt; uint8_t eg_div;
    uint16_t ta; uint8_t tb, tctl, status;
    int32_t ta_acc, tb_acc;      /* in cyc10 */
    int32_t ta_cnt, tb_cnt;
    uint8_t ct;
} O;
int fm_irq;       /* level used for GPIP3 (1 = asserted) */
int opm_irq_line(void) { return fm_irq; }

static void calc_pitch(Chan *c) {
    int oct = (c->kc >> 4) & 7, note = c->kc & 15;
    int s = oct * 12 + note_idx[note];
    uint32_t inc = pitch_tab[(s * 64 + (c->kf >> 2)) % (12 * 8 * 64)];
    c->base_inc = inc;
}
static void calc_ops(Chan *c) {
    int kcode = (c->kc >> 2) & 31;
    for (int i = 0; i < 4; i++) {
        Op *o = &c->op[i];
        uint64_t inc = c->base_inc;
        inc = inc * dt2_mul[o->dt2] >> 16;
        /* DT1: small detune, proportional to the frequency */
        int d = o->dt1 & 3; if (d) { int32_t delta = (int32_t)(inc >> 11) * d; inc = (o->dt1 & 4) ? inc - delta : inc + delta; }
        inc = o->mul ? inc * o->mul : inc >> 1;
        o->inc = (uint32_t)inc;
        o->rate_ks = (uint8_t)(kcode >> (3 - o->ks));
    }
}
static int eg_rate(Op *o, int r) {
    if (!r) return 0;
    int rate = 2 * r + o->rate_ks;
    return rate > 63 ? 63 : rate;
}
static void key_on(Op *o) {
    if (o->key) return;
    o->key = 1; o->phase = 0;
    int r = eg_rate(o, o->ar);
    if (r >= 62) { o->env = 0; o->state = o->d1l == 0 ? EG_DEC2 : EG_DEC1; }
    else o->state = EG_ATT;
}
static void key_off(Op *o) { if (!o->key) return; o->key = 0; if (o->state != EG_OFF) o->state = EG_REL; }

static void eg_step(Op *o, uint32_t cnt) {
    int rate;
    switch (o->state) {
    case EG_ATT: rate = eg_rate(o, o->ar); break;
    case EG_DEC1: rate = eg_rate(o, o->d1r); break;
    case EG_DEC2: rate = eg_rate(o, o->d2r); break;
    case EG_REL: rate = eg_rate(o, (o->rr << 1) + 1); if (o->rr == 0) rate = eg_rate(o, 1); break;
    default: return;
    }
    if (!rate) return;
    int shift = rate < 48 ? 11 - (rate >> 2) : 0;
    if (shift < 0) shift = 0;
    if (cnt & ((1u << shift) - 1)) return;
    int step = (cnt >> shift) & 7, inc;
    if (rate < 48) inc = eg_pat[rate & 3][step];
    else inc = eg_pat2[rate & 3][step] << ((rate >> 2) - 12);
    if (!inc) return;
    if (o->state == EG_ATT) {
        o->env += (int32_t)((~o->env) * inc) >> 4;
        if (o->env <= 0) { o->env = 0; o->state = o->d1l == 0 ? EG_DEC2 : EG_DEC1; }
    } else {
        o->env += inc;
        if (o->state == EG_DEC1) {
            int lvl = o->d1l == 15 ? 0x3E0 : o->d1l << 5;
            if (o->env >= lvl) { o->env = lvl; o->state = EG_DEC2; }
        } else if (o->env >= 1023) { o->env = 1023; if (o->state == EG_REL) o->state = EG_OFF; }
        if (o->env > 1023) o->env = 1023;
    }
}

static inline int32_t op_calc(Op *o, int32_t pm, int am, int noise) {
    int att = o->env + (o->tl << 3);
    if (o->ams_en) att += am;
    if (att > 1023) att = 1023;
    if (o->state == EG_OFF) return 0;
    if (noise) { int v = O.noise_out ? 4095 : -4095; return (v * att_lin[att]) >> 12; }
    int idx = ((o->phase >> 22) + pm) & 1023;
    return (sin_tab[idx] * att_lin[att]) >> 12;
}

static void lfo_step(void) {
    uint32_t period = 1u << (15 - (O.lfrq >> 4) + 3);
    O.lfo_cnt += 16 + (O.lfrq & 15);
    while (O.lfo_cnt >= period) { O.lfo_cnt -= period; O.lfo_phase = (O.lfo_phase + 1) & 255; }
    int p = (int)O.lfo_phase;
    int am, pm;
    switch (O.lfo_w) {
    case 0: am = 255 - p; pm = p < 128 ? p : p - 256; break;
    case 1: am = p < 128 ? 255 : 0; pm = p < 128 ? 127 : -128; break;
    case 2: am = p < 128 ? p * 2 : (255 - p) * 2; pm = (p < 64) ? p * 2 : (p < 192 ? (128 - p) * 2 : (p - 256) * 2); break;
    default: am = (int)(O.noise_lfsr & 255); pm = (int)((O.noise_lfsr >> 3) & 255) - 128; break;
    }
    O.lfo_am = (am * O.amd) >> 7;
    O.lfo_pm = (pm * O.pmd) >> 7;   /* -127..127 */
}

static void opm_sample(int *l, int *r) {
    /* noise generator */
    if (O.noise_en) {
        int nf = 31 - (O.nfrq & 31);
        O.noise_cnt += 1;
        if (O.noise_cnt >= (uint32_t)(nf * 2 + 1)) {
            O.noise_cnt = 0;
            uint32_t b = ((O.noise_lfsr >> 0) ^ (O.noise_lfsr >> 3)) & 1;
            O.noise_lfsr = (O.noise_lfsr >> 1) | (b << 16);
            O.noise_out = O.noise_lfsr & 1;
        }
    }
    lfo_step();
    if (++O.eg_div >= 3) { O.eg_div = 0; O.eg_cnt++; for (int c = 0; c < 8; c++) for (int i = 0; i < 4; i++) eg_step(&O.ch[c].op[i], O.eg_cnt); }
    int sl = 0, sr = 0;
    static const int pms_cents[8] = {0, 5, 10, 20, 50, 100, 400, 700};
    for (int c = 0; c < 8; c++) {
        Chan *ch = &O.ch[c];
        if (!(ch->rl & 3)) { for (int i = 0; i < 4; i++) { Op *o = &ch->op[i]; o->phase += o->inc; } continue; }
        int am = O.lfo_am >> (3 - ch->ams);
        if (!ch->ams) am = 0;
        int32_t pmv = 0;
        if (ch->pms) { int cents = O.lfo_pm * pms_cents[ch->pms] / 127; pmv = cents; }
        Op *m1 = &ch->op[0], *m2 = &ch->op[1], *c1 = &ch->op[2], *c2 = &ch->op[3];
        Op *s1 = m1, *s2 = c1, *s3 = m2, *s4 = c2;      /* OPN-style chain order */
        Op *ops[4] = {s1, s2, s3, s4};
        for (int i = 0; i < 4; i++) {
            Op *o = ops[i];
            if (pmv) o->phase += (uint32_t)((int64_t)o->inc * (1000000 + pmv * 578) / 1000000);
            else o->phase += o->inc;
        }
        /* feedback on op1 */
        int32_t fbm = 0;
        if (ch->fb) fbm = (ch->fb_out[0] + ch->fb_out[1]) >> (9 - ch->fb);
        int32_t o1 = op_calc(s1, fbm, am, 0);
        ch->fb_out[1] = ch->fb_out[0]; ch->fb_out[0] = o1;
        int32_t o2, o3, o4, out = 0;
        int nz = (c == 7 && O.noise_en);
        switch (ch->con) {
        case 0: o2 = op_calc(s2, o1 >> 1, am, 0); o3 = op_calc(s3, o2 >> 1, am, 0); o4 = op_calc(s4, o3 >> 1, am, nz); out = o4; break;
        case 1: o2 = op_calc(s2, 0, am, 0); o3 = op_calc(s3, (o1 + o2) >> 1, am, 0); o4 = op_calc(s4, o3 >> 1, am, nz); out = o4; break;
        case 2: o2 = op_calc(s2, 0, am, 0); o3 = op_calc(s3, o2 >> 1, am, 0); o4 = op_calc(s4, (o1 + o3) >> 1, am, nz); out = o4; break;
        case 3: o2 = op_calc(s2, o1 >> 1, am, 0); o3 = op_calc(s3, 0, am, 0); o4 = op_calc(s4, (o2 + o3) >> 1, am, nz); out = o4; break;
        case 4: o2 = op_calc(s2, o1 >> 1, am, 0); o3 = op_calc(s3, 0, am, 0); o4 = op_calc(s4, o3 >> 1, am, nz); out = o2 + o4; break;
        case 5: o2 = op_calc(s2, o1 >> 1, am, 0); o3 = op_calc(s3, o1 >> 1, am, 0); o4 = op_calc(s4, o1 >> 1, am, nz); out = o2 + o3 + o4; break;
        case 6: o2 = op_calc(s2, o1 >> 1, am, 0); o3 = op_calc(s3, 0, am, 0); o4 = op_calc(s4, 0, am, nz); out = o2 + o3 + o4; break;
        default: o2 = op_calc(s2, 0, am, 0); o3 = op_calc(s3, 0, am, 0); o4 = op_calc(s4, 0, am, nz); out = o1 + o2 + o3 + o4; break;
        }
        out <<= 1;
        if (ch->rl & 2) sl += out;
        if (ch->rl & 1) sr += out;
    }
    *l = sl; *r = sr;
}

static void opm_set_reg(int a, uint8_t v) {
    O.regs[a] = v;
    if (a < 0x20) {
        switch (a) {
        case 0x01: if (v & 2) { /* LFO reset */ O.lfo_phase = 0; O.lfo_cnt = 0; } break;
        case 0x08: {
            Chan *c = &O.ch[v & 7];
            for (int i = 0; i < 4; i++) { if (v & (8 << i)) key_on(&c->op[i]); else key_off(&c->op[i]); }
            break; }
        case 0x0F: O.noise_en = v >> 7; O.nfrq = v & 31; break;
        case 0x10: O.ta = (uint16_t)((O.ta & 3) | (v << 2)); break;
        case 0x11: O.ta = (uint16_t)((O.ta & 0x3FC) | (v & 3)); break;
        case 0x12: O.tb = v; break;
        case 0x14:
            if (v & 1) { O.ta_cnt = 1024 - O.ta; }
            if (v & 2) { O.tb_cnt = 256 - O.tb; }
            O.tctl = v & 0x0F;
            if (v & 0x10) O.status &= ~1;
            if (v & 0x20) O.status &= ~2;
            fm_irq = (O.status & 3) != 0;
            break;
        case 0x18: O.lfrq = v; break;
        case 0x19: if (v & 0x80) O.pmd = v & 0x7F; else O.amd = v & 0x7F; break;
        case 0x1B: O.lfo_w = v & 3; O.ct = v >> 6; adpcm_pan(-1 - (v >> 7)); break;
        }
        return;
    }
    int ch = a & 7;
    Chan *c = &O.ch[ch];
    if (a < 0x40) {
        switch ((a >> 3) & 3) {
        case 0: c->rl = v >> 6; c->fb = (v >> 3) & 7; c->con = v & 7; break;
        case 1: c->kc = v & 0x7F; calc_pitch(c); calc_ops(c); break;
        case 2: c->kf = v >> 2; calc_pitch(c); calc_ops(c); break;
        case 3: c->pms = (v >> 4) & 7; c->ams = v & 3; break;
        }
        return;
    }
    Op *o = &c->op[(a >> 3) & 3];
    switch ((a - 0x40) >> 5) {
    case 0: o->dt1 = (v >> 4) & 7; o->mul = v & 15; calc_ops(c); break;
    case 1: o->tl = v & 0x7F; break;
    case 2: o->ks = v >> 6; o->ar = v & 31; calc_ops(c); break;
    case 3: o->ams_en = v >> 7; o->d1r = v & 31; break;
    case 4: o->dt2 = v >> 6; o->d2r = v & 31; calc_ops(c); break;
    default: o->d1l = v >> 4; o->rr = v & 15; break;
    }
}
uint8_t opm_read(uint32_t a) {
    if ((a & 3) == 3) return O.status;
    return 0xFF;
}
void opm_write(uint32_t a, uint8_t v) {
    if ((a & 3) == 1) O.addr = v;
    else if ((a & 3) == 3) opm_set_reg(O.addr, v);
}

/* --------------------------------------------------------- ADPCM ------- */
static struct {
    uint8_t fifo[8192]; uint16_t rp, wp;     /* nibbles */
    uint8_t playing, clk4, pan, rate_sel;
    int step, sig, out;
    uint32_t acc;
    uint8_t half, last;
} A;
static const int16_t step_tab[49] = {16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552};
static const int8_t idx_adj[8] = {-1, -1, -1, -1, 2, 4, 6, 8};
static int fifo_count(void) { return (A.wp - A.rp) & 8191; }
void adpcm_pan(int v) {
    if (v >= 0) { A.pan = v & 3; A.rate_sel = (v >> 2) & 3; }
    else A.clk4 = (uint8_t)(-1 - v);     /* OPM CT2 => ADPCM base clock select */
}
int adpcm_dma_ready(void) { return fifo_count() < 4096; }
static void push_nibble(uint8_t n) { uint16_t nw = (A.wp + 1) & 8191; if (nw == A.rp) return; A.fifo[A.wp] = n; A.wp = nw; }
uint8_t adpcm_read(uint32_t a) {
    if ((a & 3) == 1) return A.playing ? 0xC0 : 0x40;
    return 0xFF;
}
void adpcm_write(uint32_t a, uint8_t v) {
    if ((a & 3) == 1) {
        if (v & 1) { A.playing = 0; }
        if (v & 2) { A.playing = 1; A.step = 0; A.sig = 0; A.out = 0; A.acc = 0; }
    } else if ((a & 3) == 3) {
        push_nibble(v & 15); push_nibble((v >> 4) & 15);
    }
}
static int adpcm_rate(void) {
    static const int r8[4] = {7812, 10417, 15625, 10417};
    int r = r8[A.rate_sel & 3];
    return A.clk4 ? r / 2 : r;
}
static void adpcm_tick(void) {
    if (!A.playing) return;
    if (A.rp == A.wp) return;          /* underrun: hold */
    uint8_t n = A.fifo[A.rp]; A.rp = (A.rp + 1) & 8191;
    int st = step_tab[A.step];
    int d = ((2 * (n & 7) + 1) * st) >> 3;
    if (n & 8) d = -d;
    A.sig += d;
    if (A.sig > 2047) A.sig = 2047; if (A.sig < -2048) A.sig = -2048;
    A.step += idx_adj[n & 7];
    if (A.step < 0) A.step = 0; if (A.step > 48) A.step = 48;
    A.out = A.sig;
}

/* -------------------------------------------------------- mixer ------- */
#define RING 16384
static int16_t ring[RING * 2];
static uint32_t ring_w, ring_r;
static int32_t out_acc;                 /* in units of 1e7 / OUT_RATE cyc10 fractions */
static int32_t opm_pos;                 /* fixed 16.16 OPM sample position */
static int opm_l0, opm_r0, opm_l1, opm_r1;
static int adpcm_acc;
static int hp_l, hp_r;

void sound_init(void) { make_tables(); }
void sound_reset(void) {
    make_tables();
    memset(&O, 0, sizeof O); memset(&A, 0, sizeof A);
    O.noise_lfsr = 0x1FFFF;
    for (int c = 0; c < 8; c++) { O.ch[c].rl = 3; for (int i = 0; i < 4; i++) { O.ch[c].op[i].state = EG_OFF; O.ch[c].op[i].env = 1023; } }
    fm_irq = 0; ring_w = ring_r = 0; out_acc = 0; opm_pos = 0; opm_l0 = opm_r0 = opm_l1 = opm_r1 = 0; adpcm_acc = 0;
    mfp_gpip_fm = 0;
}
static void emit_frame(void) {
    opm_pos += (int32_t)(((int64_t)OPM_RATE << 16) / OUT_RATE);
    while (opm_pos >= 65536) { opm_pos -= 65536; opm_l0 = opm_l1; opm_r0 = opm_r1; opm_sample(&opm_l1, &opm_r1); }
    int frac = opm_pos;
    int l = opm_l0 + (int)(((int64_t)(opm_l1 - opm_l0) * frac) >> 16);
    int r = opm_r0 + (int)(((int64_t)(opm_r1 - opm_r0) * frac) >> 16);
    adpcm_acc += adpcm_rate();
    while (adpcm_acc >= OUT_RATE) { adpcm_acc -= OUT_RATE; adpcm_tick(); }
    int ad = A.playing ? A.out * 8 : 0;
    if (!(A.pan & 1)) l += ad;
    if (!(A.pan & 2)) r += ad;
    if (l > 32767) l = 32767; if (l < -32768) l = -32768;
    if (r > 32767) r = 32767; if (r < -32768) r = -32768;
    uint32_t nw = (ring_w + 1) % RING;
    if (nw == ring_r) return;
    ring[ring_w * 2] = (int16_t)l; ring[ring_w * 2 + 1] = (int16_t)r;
    ring_w = nw;
}
void sound_run(int cyc10) {
    /* OPM timers (cycle accurate enough at line granularity) */
    O.ta_acc += cyc10; O.tb_acc += cyc10;
    while (O.ta_acc >= 160) {
        O.ta_acc -= 160;
        if (O.tctl & 1) {
            if (--O.ta_cnt <= 0) {
                O.ta_cnt = 1024 - O.ta;
                if (O.tctl & 4) { O.status |= 1; if (!fm_irq) mfp_int(12); fm_irq = 1; }
            }
        }
    }
    while (O.tb_acc >= 2560) {
        O.tb_acc -= 2560;
        if (O.tctl & 2) {
            if (--O.tb_cnt <= 0) {
                O.tb_cnt = 256 - O.tb;
                if (O.tctl & 8) { O.status |= 2; if (!fm_irq) mfp_int(12); fm_irq = 1; }
            }
        }
    }
    mfp_gpip_fm = fm_irq;
    out_acc += cyc10 * OUT_RATE;
    while (out_acc >= 10000000) { out_acc -= 10000000; emit_frame(); }
    if (A.playing || fifo_count() < 4096) dmac_exec(3);
}
int sound_render(int16_t *out, int frames) {
    int n = 0;
    while (n < frames && ring_r != ring_w) {
        out[n * 2] = ring[ring_r * 2]; out[n * 2 + 1] = ring[ring_r * 2 + 1];
        ring_r = (ring_r + 1) % RING; n++;
    }
    return n;
}
void sound_state(void *s) {
    SER(s, O); SER(s, A);
}
