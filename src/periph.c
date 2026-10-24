/* periph.c - MFP 68901, RTC, system port, IOC, PPI/joystick, keyboard, SCC (mouse + serial) */
#include "x68k.h"

extern int mach_vline, mach_line10, mach_cpu_mhz;
int mfp_gpip_hsync, mfp_gpip_vdisp, mfp_gpip_cirq, mfp_gpip_fm;
uint8_t sys_contrast;

/* ================================================================ MFP == */
enum { GPIP, AER, DDR, IERA, IERB, IPRA, IPRB, ISRA, ISRB, IMRA, IMRB, VR, TACR, TBCR, TCDCR,
       TADR, TBDR, TCDR, TDDR, SCR, UCR, RSR, TSR, UDR };
static struct {
    uint8_t r[24];
    uint8_t reload[4];
    int32_t tick[4];
    uint8_t key_last, key_flag;
    uint8_t keyfifo[256]; uint8_t krp, kwp;
} M;
static const int prescale[8] = {1, 10, 25, 40, 125, 160, 250, 500};

void mfp_init(void) {
    /* hardware reset state: every register cleared (the IPL ROM programs what it needs) */
    memset(&M, 0, sizeof M);
    M.r[TSR] = 0x80;
}
int mfp_pending(void) {
    for (int ofs = 0; ofs < 2; ofs++)
        if (M.r[IPRA + ofs] & M.r[IMRA + ofs] & ~M.r[ISRA + ofs]) return 1;
    return 0;
}
void mfp_int(int irq) {
    int ofs = irq >= 8; uint8_t flag = (uint8_t)(0x80 >> (irq & 7));
    if (M.r[IERA + ofs] & flag) M.r[IPRA + ofs] |= flag;
    irq_update();
}
int mfp_ack(void) {
    uint8_t flag; int vect = 0, ofs = 0;
    for (flag = 0x80, vect = 15; flag; flag >>= 1, vect--)
        if ((M.r[IPRA] & flag) && (M.r[IMRA] & flag) && !(M.r[ISRA] & flag)) break;
    if (!flag) {
        ofs = 1;
        for (flag = 0x80, vect = 7; flag; flag >>= 1, vect--)
            if ((M.r[IPRB] & flag) && (M.r[IMRB] & flag) && !(M.r[ISRB] & flag)) break;
    }
    if (!flag) return 24 + 6; /* spurious: autovector */
    M.r[IPRA + ofs] &= (uint8_t)~flag;
    if (M.r[VR] & 8) M.r[ISRA + ofs] |= flag;
    irq_update();
    return (vect & 15) | (M.r[VR] & 0xF0);
}
void mfp_key_data(uint8_t k) { M.key_last = k; M.key_flag = 1; mfp_int(3); }

uint8_t mfp_read(uint32_t a) {
    if (a > 0xE8802F || !(a & 1)) return 0xFF;
    int reg = (a & 0x3F) >> 1;
    switch (reg) {
    case GPIP: {
        uint8_t ret = 0x20;
        if (mach_vline >= crtc_vstart && mach_vline < crtc_vend) ret |= 0x10;
        ret |= 0x03 | (mfp_gpip_fm ? 0 : 0x08) | 0x04;
        int len = mach_line10 ? mach_line10 : 1;
        int pos = (int)((int64_t)cpu.cycles * 10 / (mach_cpu_mhz ? mach_cpu_mhz : 10));
        int ht = crtc_regs[1] ? crtc_regs[1] : 1;
        if (pos >= (int)crtc_regs[5] * len / ht && pos < (int)crtc_regs[7] * len / ht) ret &= 0x7F; else ret |= 0x80;
        if (mach_vline != crtc_intline) ret |= 0x40;
        return ret; }
    case UDR: M.key_flag = 0; return M.key_last;
    case RSR: return (M.krp != M.kwp) ? (M.r[reg] & 0x7F) : (M.r[reg] | 0x80);
    default: return M.r[reg];
    }
}
void mfp_write(uint32_t a, uint8_t v) {
    if (a > 0xE8802F || !(a & 1)) return;
    int reg = (a & 0x3F) >> 1;
    switch (reg) {
    case IERA: case IERB: M.r[reg] = v; M.r[reg + 2] &= v; irq_update(); break;
    case IPRA: case IPRB: case ISRA: case ISRB: M.r[reg] &= v; irq_update(); break;
    case IMRA: case IMRB: M.r[reg] = v; irq_update(); break;
    case TADR: case TBDR: case TCDR: case TDDR: M.reload[reg - TADR] = M.r[reg] = v; break;
    case TSR: M.r[reg] = v | 0x80; break;
    case UDR: break;
    default: M.r[reg] = v; break;
    }
}
void mfp_run(int clk) {
    static const int tint[4] = {2, 7, 10, 11};
    for (int ch = 0; ch < 4; ch++) {
        int mode = ch == 2 ? (M.r[TCDCR] >> 4) : ch == 3 ? M.r[TCDCR] : M.r[TACR + ch];
        if (ch < 2 && (mode & 8)) continue;
        if (!(mode & 7)) continue;
        int t = prescale[mode & 7];
        M.tick[ch] += clk;
        while (M.tick[ch] >= t) {
            M.tick[ch] -= t;
            M.r[TADR + ch]--;
            if (M.r[TADR + ch] == 0) { M.r[TADR + ch] = M.reload[ch]; mfp_int(tint[ch]); }
        }
    }
}
void mfp_timera_event(void) {
    if ((M.r[TACR] & 15) != 8) return;
    int hit;
    if (M.r[AER] & 0x10) hit = mach_vline == crtc_vstart;
    else hit = (crtc_vend >= crtc_vtotal) ? mach_vline == crtc_vtotal - 1 : mach_vline == crtc_vend;
    if (hit) M.r[TADR]--;
    if (!M.r[TADR]) { M.r[TADR] = M.reload[0]; mfp_int(2); }
}
void mfp_line_events(int vline) {
    mfp_int(0);
    if (!(M.r[AER] & 0x40) && vline == crtc_intline) mfp_int(1);
    if (M.r[AER] & 0x10) { if (vline == crtc_vstart) mfp_int(9); }
    else if (crtc_vend >= crtc_vtotal) { if (vline == crtc_vend - crtc_vtotal) mfp_int(9); }
    else if (vline == crtc_vtotal - 1) mfp_int(9);
}
void mfp_line_end_events(int vline) {
    if ((M.r[AER] & 0x40) && vline == crtc_intline) mfp_int(1);
}

/* ============================================================ keyboard == */
void key_event(int sc, int down) {
    uint8_t code = (uint8_t)(sc | (down ? 0 : 0x80));
    uint8_t nw = (uint8_t)(M.kwp + 1);
    if (nw == M.krp) return;
    M.keyfifo[M.kwp] = code; M.kwp = nw;
}
void keyboard_tick(void) {
    if (M.krp != M.kwp && !M.key_flag) { uint8_t k = M.keyfifo[M.krp++]; mfp_key_data(k); }
}

/* ===================================================================== RTC */
static struct { uint8_t bank; uint8_t r0[16], r1[16]; int32_t t1, t16; } RT;
uint8_t rtc_read(uint32_t a) {
    int tm[9]; host_localtime(host_now_ms(), tm);
    a &= 0x1F;
    if (!(a & 1)) return 0;
    int idx = a >> 1;
    if (RT.bank == 0) {
        int yr = tm[5] - 80;
        switch (idx) {
        case 0: return tm[0] % 10; case 1: return tm[0] / 10;
        case 2: return tm[1] % 10; case 3: return tm[1] / 10;
        case 4: return tm[2] % 10; case 5: return tm[2] / 10;
        case 6: return (uint8_t)tm[6];
        case 7: return tm[3] % 10; case 8: return tm[3] / 10;
        case 9: return (tm[4] + 1) % 10; case 10: return (tm[4] + 1) / 10;
        case 11: return yr % 10; case 12: return (yr / 10) & 15;
        case 13: return RT.r0[13]; case 14: return RT.r0[14]; default: return RT.r0[15];
        }
    }
    if (idx == 13) return RT.r1[13] | 1;
    if (idx == 11) return (uint8_t)((tm[5] - 80) % 4);
    return RT.r1[idx];
}
void rtc_write(uint32_t a, uint8_t v) {
    a &= 0x1F;
    if (!(a & 1)) return;
    int idx = a >> 1;
    if (idx == 13) { RT.bank = v & 1; RT.r0[13] = RT.r1[13] = v & 0x0C; }
    else if (idx == 15) RT.r0[15] = RT.r1[15] = v & 0x0C;
    else if (idx == 10 && RT.bank) RT.r1[10] = v;
}
void rtc_run(int clk) {
    RT.t1 += clk; RT.t16 += clk;
    if (RT.t1 >= 10000000) { if (!(RT.r0[15] & 8)) mfp_int(15); RT.t1 -= 10000000; }
    if (RT.t16 >= 625000) { if (!(RT.r0[15] & 4)) mfp_int(15); RT.t16 -= 625000; }
}

/* ============================================================ sys port == */
static struct { uint8_t contrast, monitor, keyctrl, cputype; } SP;
uint8_t sysport_read(uint32_t a) {
    if (!(a & 1)) return 0xFF;
    switch ((a & 0x0F) >> 1) {
    case 0: return 0xF0 | SP.contrast;
    case 1: return 0xF0 | SP.monitor;
    case 3: return 0xF0 | SP.keyctrl;
    case 5: return 0xF0 | SP.cputype;
    default: return 0xFF;
    }
}
void sysport_write(uint32_t a, uint8_t v) {
    if (!(a & 1)) return;
    switch ((a & 0x0F) >> 1) {
    case 0: SP.contrast = v & 15; sys_contrast = SP.contrast; break;
    case 1: SP.monitor = v & 0x0B; break;
    case 3: SP.keyctrl = v & 0x0E; break;
    case 6: sram_wen = (v == 0x31); break;
    default: break;
    }
}

/* ========================================================== IOC / PPI ==== */
uint8_t ioc_stat, ioc_vect;
void ioc_irq(int bit) {
    /* enable bits: FDC 0x04, FDD 0x02, HDD 0x08; status bits as per IOC_* */
    int en = bit == IOC_FDC ? 4 : bit == IOC_FDD ? 2 : bit == IOC_HDD ? 8 : 1;
    ioc_stat |= (uint8_t)bit;
    if (ioc_stat & en) { extern uint8_t ioc_pend; ioc_pend |= (uint8_t)bit; irq_update(); }
}
uint8_t ioc_pend;
static uint8_t pia_a = 0xFF, pia_b = 0xFF, pia_c = 0x0B, joy[2] = {0xFF, 0xFF};
void set_joy(int port, uint8_t bits) { joy[port & 1] = (uint8_t)~bits; }
uint8_t pia_read(uint32_t a) {
    if (a == 0xE9A001) return joy[0];
    if (a == 0xE9A003) return joy[1];
    if (a == 0xE9A005) return pia_c;
    return 0xFF;
}
void pia_write(uint32_t a, uint8_t d) {
    uint8_t old = pia_c;
    if (a == 0xE9A005) { pia_c = d; if ((old & 15) != (d & 15)) adpcm_pan(d & 15); }
    else if (a == 0xE9A007) {
        if (!(d & 0x80)) {
            uint8_t m = (uint8_t)(1 << ((d >> 1) & 7));
            if (d & 1) pia_c |= m; else pia_c &= (uint8_t)~m;
            if ((old & 15) != (pia_c & 15)) adpcm_pan(pia_c & 15);
        }
    }
}

/* ==================================================== SCC (mouse, serial) */
static struct {
    int mx, my, mst;       /* last packet */
    int acc_dx, acc_dy, btn;
    uint8_t regnum_a, regset_a, regnum_b, regset_b;
    uint8_t regs_a[16], regs_b[16];
    uint8_t vector; uint8_t dat[3]; uint8_t datnum;
    /* serial (channel A) */
    uint8_t rx[4096]; uint16_t rxr, rxw;
    uint8_t tx[4096]; uint16_t txr, txw;
    uint8_t connected, tx_writable;
} S;
int mouse_last_dx, mouse_last_dy, mouse_last_stat;
void scc_reset(void) { uint8_t c = S.connected, w = S.tx_writable; memset(&S, 0, sizeof S); S.connected = c; S.tx_writable = w; }
void mouse_add(int dx, int dy, int l, int r) {
    S.acc_dx += dx; S.acc_dy += dy; S.btn = (l ? 1 : 0) | (r ? 2 : 0);
}
static void mouse_set_data(void) {
    int dx = S.acc_dx, dy = S.acc_dy, st = S.btn;
    if (dx > 127) { dx = 127; st |= 0x10; } if (dx < -128) { dx = -128; st |= 0x20; }
    if (dy > 127) { dy = 127; st |= 0x40; } if (dy < -128) { dy = -128; st |= 0x80; }
    S.acc_dx -= dx; S.acc_dy -= dy;
    S.mx = dx; S.my = dy; S.mst = st;
    mouse_last_dx = dx; mouse_last_dy = dy; mouse_last_stat = st;
}
int scc_pending(void) {
    if (S.datnum && (S.regs_b[1] & 0x18) == 0x10 && (S.regs_b[9] & 8)) return 1;
    if (S.datnum == 3 && (S.regs_b[1] & 0x18) == 0x08 && (S.regs_b[9] & 8)) return 1;
    return 0;
}
void scc_int_check(void) { if (scc_pending()) irq_update(); }
int scc_ack(void) {
    int v;
    if (!(S.regs_b[9] & 2)) {
        if (S.regs_b[9] & 1) v = (S.regs_b[9] & 0x10) ? ((S.vector & 0x8F) + 0x20) : ((S.vector & 0xF1) + 4);
        else v = S.vector;
    } else v = 24 + 5;
    S.datnum = S.datnum; /* one-shot like the reference: data is consumed by the guest */
    irq_update();
    return v;
}
uint8_t scc_read(uint32_t a) {
    if (a >= 0xE98008) return 0;
    switch (a & 7) {
    case 1: { uint8_t r = 0; if (!S.regnum_b) r = S.datnum ? 1 : 0; S.regnum_b = 0; S.regset_b = 0; return r; }
    case 3: if (S.datnum) { S.datnum--; return S.dat[S.datnum]; } return 0;
    case 5: {
        uint8_t r = 0;
        if (S.regnum_a == 0) r = (S.rxr != S.rxw ? 1 : 0) | (S.tx_writable || !S.connected ? 4 : 0) | (S.connected ? 0x28 : 0);
        else if (S.regnum_a == 3) r = (S.datnum ? 4 : 0);
        S.regnum_a = 0; S.regset_a = 0; return r; }
    case 7: if (S.rxr != S.rxw) { uint8_t c = S.rx[S.rxr]; S.rxr = (S.rxr + 1) & 4095; return c; } return 0;
    default: return 0;
    }
}
void scc_write(uint32_t a, uint8_t v) {
    if (a >= 0xE98008) return;
    switch (a & 7) {
    case 1:
        if (S.regset_b) {
            if (S.regnum_b == 5) {
                if (!(S.regs_b[5] & 2) && (v & 2) && (S.regs_b[3] & 1) && !S.datnum) {
                    mouse_set_data(); S.datnum = 3;
                    S.dat[2] = (uint8_t)S.mst; S.dat[1] = (uint8_t)S.mx; S.dat[0] = (uint8_t)S.my;
                }
            } else if (S.regnum_b == 2) S.vector = v;
            S.regset_b = 0; S.regs_b[S.regnum_b] = v; S.regnum_b = 0;
        } else {
            if (!(v & 0xF0)) { S.regset_b = 1; S.regnum_b = v & 15; } else { S.regset_b = 0; S.regnum_b = 0; }
        }
        break;
    case 5:
        if (S.regset_a) {
            S.regset_a = 0;
            S.regs_a[S.regnum_a] = v;
            if (S.regnum_a == 2) { S.regs_b[2] = v; S.vector = v; }
            else if (S.regnum_a == 9) S.regs_b[9] = v;
        } else {
            v &= 15;
            if (v) { S.regset_a = 1; S.regnum_a = v; } else { S.regset_a = 0; S.regnum_a = 0; }
        }
        break;
    case 7: { uint16_t n = (S.txw + 1) & 4095; if (n != S.txr) { S.tx[S.txw] = v; S.txw = n; } break; }
    default: break;
    }
}
/* serial bridge exports */
int serial_rx(const uint8_t *p, int n) {
    int k = 0;
    while (k < n) { uint16_t nw = (S.rxw + 1) & 4095; if (nw == S.rxr) break; S.rx[S.rxw] = p[k++]; S.rxw = nw; }
    return k;
}
int serial_tx_available(void) { return (S.txw - S.txr) & 4095; }
int serial_tx_drain(uint8_t *p, int n) {
    int k = 0; while (k < n && S.txr != S.txw) { p[k++] = S.tx[S.txr]; S.txr = (S.txr + 1) & 4095; } return k;
}
void serial_reset(void) { S.rxr = S.rxw = S.txr = S.txw = 0; }
void serial_set_connected(int c) { S.connected = (uint8_t)c; }
void serial_set_tx_writable(int c) { S.tx_writable = (uint8_t)c; }
int serial_baud(void) {
    int tc = S.regs_a[12] | (S.regs_a[13] << 8);
    int mult = (S.regs_a[4] >> 6) & 3; mult = mult == 0 ? 1 : mult == 1 ? 16 : mult == 2 ? 32 : 64;
    if (!(S.regs_a[14] & 1) && !(S.regs_a[11] & 0x80)) return 0;
    return (int)(5000000L / (2L * (tc + 2) * mult));
}
int get_mouse_scc_x(void) { return S.mx; }
int get_mouse_scc_y(void) { return S.my; }
int get_mouse_scc_stat(void) { return S.mst; }
int get_mouse_enabled(void) { return (S.regs_b[3] & 1) ? 1 : 0; }

/* ===================================================== reset + state ==== */
void periph_reset(void) {
    mfp_init();
    memset(&RT, 0, sizeof RT); RT.r0[15] = 0x0C;
    SP.contrast = 15; sys_contrast = 15; SP.monitor = 0; SP.keyctrl = 0;
    SP.cputype = cfg.cpu_mhz >= 25 ? 0xDC : cfg.cpu_mhz >= 16 ? 0xFE : 0xFF;
    ioc_stat = 0; ioc_vect = 0; ioc_pend = 0;
    pia_a = pia_b = 0xFF; pia_c = 0x0B;
    scc_reset();
    mfp_gpip_fm = 0;
}
void periph_state(void *ser) {
    SER(ser, M); SER(ser, RT); SER(ser, SP); SER(ser, ioc_stat); SER(ser, ioc_vect); SER(ser, ioc_pend);
    SER(ser, pia_c);
    uint8_t keep_c = S.connected, keep_w = S.tx_writable;
    SER(ser, S);
    S.connected = keep_c; S.tx_writable = keep_w;
}
