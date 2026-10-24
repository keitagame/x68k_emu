/* video.c - CRTC, video controller, text VRAM, graphics VRAM, sprites/BG and scanline compositor */
#include "x68k.h"

extern uint8_t sys_contrast;
uint8_t  crtc_regs[0x30];
uint16_t crtc_vstart, crtc_vend, crtc_hstart, crtc_hend, crtc_vtotal = 568, crtc_intline;
uint32_t text_scroll_x, text_scroll_y;
int video_hfreq_hi = 1;
uint16_t video_fb[1024 * 1024];
int video_w = 768, video_h = 512, video_changed;

static uint16_t gscroll_x[4], gscroll_y[4];
static uint8_t  crtc_mode_port;
static uint8_t  pal_raw[2][512];        /* [0] graphic, [1] text/sprite; big-endian byte order */
static uint16_t pal565[2][256];
static uint8_t  vcreg[3][2];
static uint8_t  bg_regs[0x12];
static uint8_t  sprite_regs[0x400];
static uint8_t  bgram[0x8000];
static uint8_t  cur_contrast = 15;
static uint16_t conv_tab[65536];
static int      conv_valid;

/* ---------------------------------------------------------- colours ---- */
static uint16_t conv_slow(uint16_t w, int contrast) {
    int g5 = (w >> 11) & 31, r5 = (w >> 6) & 31, b5 = (w >> 1) & 31, i = w & 1;
    int g6 = (g5 << 1) | i;
    if (contrast < 15) {
        int f = contrast + 1;
        r5 = r5 * f / 16; g6 = g6 * f / 16; b5 = b5 * f / 16;
    }
    return (uint16_t)((r5 << 11) | (g6 << 5) | b5);
}
static void rebuild_pals(void) {
    for (int t = 0; t < 2; t++)
        for (int i = 0; i < 256; i++) {
            uint16_t w = (uint16_t)((pal_raw[t][i * 2] << 8) | pal_raw[t][i * 2 + 1]);
            pal565[t][i] = conv_slow(w, cur_contrast);
        }
    conv_valid = 0;
}
static void ensure_conv(void) {
    if (conv_valid) return;
    for (uint32_t w = 0; w < 65536; w++) conv_tab[w] = conv_slow((uint16_t)w, cur_contrast);
    conv_valid = 1;
}

/* ------------------------------------------------------------ CRTC ----- */
static void crtc_derive(void) {
    uint16_t *r = (uint16_t *)0; (void)r;
#define W16(i) ((uint16_t)((crtc_regs[(i) * 2] << 8) | crtc_regs[(i) * 2 + 1]))
    crtc_hstart = W16(2) & 0x3FF; crtc_hend = W16(3) & 0x3FF;
    crtc_vtotal = (uint16_t)((W16(4) & 0x3FF) + 1);
    crtc_vstart = W16(6) & 0x3FF; crtc_vend = W16(7) & 0x3FF;
    crtc_intline = W16(9) & 0x3FF;
    text_scroll_x = W16(10) & 0x3FF; text_scroll_y = W16(11) & 0x3FF;
    int mask = (crtc_regs[0x28] & 4) ? 1023 : 511;
    for (int p = 0; p < 4; p++) { gscroll_x[p] = W16(12 + p * 2) & mask; gscroll_y[p] = W16(13 + p * 2) & mask; }
    video_hfreq_hi = (crtc_regs[0x29] & 0x10) != 0;
}
void crtc_fast_clear(void) {
    static const uint16_t masks[16] = {0xFFFF, 0xFFF0, 0xFF0F, 0xFF00, 0xF0FF, 0xF0F0, 0xF00F, 0xF000,
                                       0x0FFF, 0x0FF0, 0x0F0F, 0x0F00, 0x00FF, 0x00F0, 0x000F, 0x0000};
    uint16_t m = masks[crtc_regs[0x2B] & 15];
    uint16_t *g = (uint16_t *)gvram;
    for (int i = 0; i < 0x40000; i++) g[i] &= m;
}
static void raster_copy(void) {
    static const uint32_t off[4] = {0, 0x20000, 0x40000, 0x60000};
    uint32_t src = (uint32_t)crtc_regs[0x2C] << 9, dst = (uint32_t)crtc_regs[0x2D] << 9;
    for (int b = 0; b < 4; b++)
        if (crtc_regs[0x2B] & (1 << b)) memmove(&tvram[(dst + off[b]) & 0x7FFFF], &tvram[(src + off[b]) & 0x7FFFF], 512);
}
uint8_t crtc_read(uint32_t a) {
    uint32_t o = a & 0xFFF;
    if (o < 0x30) {
        if (o >= 0x28 || (o >= 0x24 && o < 0x28)) return crtc_regs[o];
        return crtc_regs[o];
    }
    if (a == 0xE80481) return crtc_mode_port & 0xFD; /* fast clear completes instantly */
    return 0xFF;
}
void crtc_write(uint32_t a, uint8_t v) {
    uint32_t o = a & 0xFFF;
    if (o < 0x30) {
        crtc_regs[o] = v;
        crtc_derive();
        if ((o == 0x2C || o == 0x2D) && (crtc_mode_port & 8) && o == 0x2D) raster_copy();
    } else if (a == 0xE80481) {
        crtc_mode_port = v;
        if (v & 8) raster_copy();
        if (v & 2) crtc_fast_clear();
    }
}

/* ---------------------------------------------- video controller ------- */
uint8_t vctrl_read(uint32_t a) {
    uint32_t o = a & 0x1FFF;
    if (o < 0x400) return pal_raw[o >= 0x200][o & 0x1FF];
    if (o >= 0x400 && o < 0x700) { int w = (o - 0x400) >> 8; if (w < 3) return vcreg[w][o & 1]; }
    return 0xFF;
}
void vctrl_write(uint32_t a, uint8_t v) {
    uint32_t o = a & 0x1FFF;
    if (o < 0x400) {
        int t = o >= 0x200; uint32_t k = o & 0x1FF;
        pal_raw[t][k] = v;
        int idx = k >> 1;
        pal565[t][idx] = conv_slow((uint16_t)((pal_raw[t][idx * 2] << 8) | pal_raw[t][idx * 2 + 1]), cur_contrast);
    } else if (o >= 0x400 && o < 0x700) {
        int w = (o - 0x400) >> 8;
        if (w < 3 && (o & 0xFF) < 2) vcreg[w][o & 1] = v;
    }
}

/* ---------------------------------------------------- text VRAM -------- */
void tvram_write8(uint32_t a, uint8_t v) {
    uint32_t off = a & 0x7FFFF;
    int conc = crtc_regs[0x2A] & 1, mask = crtc_regs[0x2A] & 2;
    uint32_t mp = off & 1;
    uint32_t in = off & 0x1FFFF;
    if (conc) {
        for (int p = 0; p < 4; p++) {
            if (!(crtc_regs[0x2B] & (0x10 << p))) continue;
            uint32_t i = ((in + p * 0x20000) ^ 1) & 0x7FFFF;
            if (mask) { uint8_t m = crtc_regs[0x2E + mp]; tvram[i] = (uint8_t)((tvram[i] & m) | (v & ~m)); }
            else tvram[i] = v;
        }
    } else {
        uint32_t i = off ^ 1;
        if (mask) { uint8_t m = crtc_regs[0x2E + mp]; tvram[i] = (uint8_t)((tvram[i] & m) | (v & ~m)); }
        else tvram[i] = v;
    }
}

/* ---------------------------------------------------- sprites / BG ----- */
uint8_t bg_read(uint32_t a) {
    if (a < 0xEB0400) return sprite_regs[(a - 0xEB0000) ^ 1];
    if (a >= 0xEB0800 && a < 0xEB0812) return bg_regs[a - 0xEB0800];
    if (a >= 0xEB8000) return bgram[a - 0xEB8000];
    return 0xFF;
}
void bg_write(uint32_t a, uint8_t v) {
    if (a < 0xEB0400) sprite_regs[(a - 0xEB0000) ^ 1] = v;
    else if (a >= 0xEB0800 && a < 0xEB0812) bg_regs[a - 0xEB0800] = v;
    else if (a >= 0xEB8000) bgram[a - 0xEB8000] = v;
}
/* 16x16 pattern pixel (row y, col x) of pattern n; flips applied by caller */
static inline int pcg16(int n, int x, int y) {
    int blk = (x >> 3) * 2 + (y >> 3);              /* UL=0 LL=1 UR=2 LR=3 -> 32 byte blocks */
    int byte = (n * 128 + blk * 32 + (y & 7) * 4 + ((x & 7) >> 1)) & 0x7FFF;
    uint8_t b = bgram[byte];
    return (x & 1) ? (b & 15) : (b >> 4);
}
static inline int pcg8(int n, int x, int y) {
    int byte = ((n & 255) * 32 + y * 4 + (x >> 1)) & 0x1FFF;
    uint8_t b = bgram[byte];
    return (x & 1) ? (b & 15) : (b >> 4);
}

/* ----------------------------------------------------- line buffers ---- */
static uint16_t gl_col[1024]; static uint8_t gl_op[1024];
static uint16_t tl_col[1024]; static uint8_t tl_op[1024];
static uint16_t sl_col[1024 + 32]; static uint8_t sl_op[1024 + 32];   /* sprite/BG result */
static uint8_t  lay_op[5][1056]; static uint16_t lay_col[5][1056];   /* spr1,bg1,spr2,bg0,spr3 */

static void render_text(int y, int w) {
    ensure_conv();
    uint32_t ty = (y + text_scroll_y) & 1023;
    uint32_t rowbase = ty * 128;
    for (int x = 0; x < w; x++) {
        uint32_t tx = (x + text_scroll_x) & 1023;
        uint32_t o = rowbase + (tx >> 3);
        int sh = 7 - (tx & 7), idx = 0;
        idx |= (tvram[(o) ^ 1] >> sh) & 1;
        idx |= ((tvram[(o + 0x20000) ^ 1] >> sh) & 1) << 1;
        idx |= ((tvram[(o + 0x40000) ^ 1] >> sh) & 1) << 2;
        idx |= ((tvram[(o + 0x60000) ^ 1] >> sh) & 1) << 3;
        tl_op[x] = idx != 0;
        tl_col[x] = pal565[1][idx];
    }
}

static void render_graphics(int y, int w) {
    int cm = vcreg[0][1] & 3, big = (crtc_regs[0x28] & 4) != 0;
    int on = vcreg[2][1] & 0x1F;
    for (int x = 0; x < w; x++) gl_op[x] = 0;
    if (!on) return;
    const uint8_t *g8 = gvram;
    if (big) { /* 1024x1024, 16 colours, 4 pages tiled */
        for (int x = 0; x < w; x++) {
            int xx = (x + gscroll_x[0]) & 1023, yy = (y + gscroll_y[0]) & 1023;
            int q = ((yy >> 9) << 1) | (xx >> 9);
            uint16_t wd = *(const uint16_t *)&g8[(((yy & 511) * 512) + (xx & 511)) * 2];
            int idx = (wd >> (q * 4)) & 15;
            if (idx) { gl_op[x] = 1; gl_col[x] = pal565[0][idx]; }
        }
        return;
    }
    int pri[4];
    for (int p = 0; p < 4; p++) pri[p] = (vcreg[1][1] >> (2 * p)) & 3;
    if (cm == 0) {
        int order[4] = {0, 1, 2, 3};
        for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) {
            int a = order[i], b = order[j];
            if (pri[b] > pri[a] || (pri[b] == pri[a] && b < a)) { order[i] = b; order[j] = a; }
        } /* now bottom (lowest priority) first */
        for (int k = 0; k < 4; k++) {
            int p = order[k];
            if (!(vcreg[2][1] & (1 << p))) continue;
            int yy = (y + gscroll_y[p]) & 511;
            const uint8_t *row = &g8[yy * 1024];
            for (int x = 0; x < w; x++) {
                int xx = (x + gscroll_x[p]) & 511;
                int idx = (*(const uint16_t *)&row[xx * 2] >> (p * 4)) & 15;
                if (idx) { gl_op[x] = 1; gl_col[x] = pal565[0][idx]; }
            }
        }
    } else if (cm == 1) {
        int first = pri[1] > pri[0] ? 1 : 0;
        for (int k = 0; k < 2; k++) {
            int p = k == 0 ? first : 1 - first;
            if (!(vcreg[2][1] & (p ? 0x0C : 0x03))) continue;
            int yy = (y + gscroll_y[p]) & 511;
            const uint8_t *row = &g8[yy * 1024];
            for (int x = 0; x < w; x++) {
                int xx = (x + gscroll_x[p]) & 511;
                int idx = (*(const uint16_t *)&row[xx * 2] >> (p * 8)) & 255;
                if (idx) { gl_op[x] = 1; gl_col[x] = pal565[0][idx]; }
            }
        }
    } else {
        ensure_conv();
        int yy = (y + gscroll_y[0]) & 511;
        const uint8_t *row = &g8[yy * 1024];
        for (int x = 0; x < w; x++) {
            int xx = (x + gscroll_x[0]) & 511;
            uint16_t wd = *(const uint16_t *)&row[xx * 2];
            if (wd) { gl_op[x] = 1; gl_col[x] = conv_tab[wd]; }
        }
    }
}

static void render_sprites(int y, int w) {
    for (int x = 0; x < w; x++) sl_op[x] = 0;
    if (!(vcreg[2][1] & 0x40) || !(bg_regs[8] & 2) || (bg_regs[0x11] & 2 && 0)) return;
    int hadj = ((int)bg_regs[0x0D] - ((int)crtc_hstart + 4)) * 8;
    int div = (bg_regs[0x11] & 4) ? 1 : 2;
    int bgv = ((int)bg_regs[0x0F] - (int)crtc_vstart) / div;
    int size16 = (bg_regs[0x11] & 3) != 0;
    for (int l = 0; l < 5; l++) for (int x = 0; x < w; x++) lay_op[l][x] = 0;
    /* background tiles */
    for (int b = 0; b < 2; b++) {
        int en = b == 0 ? (bg_regs[9] & 1) : (bg_regs[9] & 8);
        if (!en) continue;
        int top;
        if (b == 0) top = (bg_regs[9] & 6) ? 0x6000 : 0x4000; else top = (bg_regs[9] & 0x30) ? 0x6000 : 0x4000;
        int sx = ((bg_regs[b * 4] << 8) | bg_regs[b * 4 + 1]) & (size16 ? 1023 : 511);
        int sy = ((bg_regs[b * 4 + 2] << 8) | bg_regs[b * 4 + 3]) & (size16 ? 1023 : 511);
        int layer = b == 0 ? 3 : 1;
        int yy = (sy + y - bgv) & (size16 ? 1023 : 511);
        int ts = size16 ? 16 : 8, tmask = size16 ? 63 : 63;
        for (int x = 0; x < w; x++) {
            int xx = (sx - hadj + x) & (size16 ? 1023 : 511);
            int tx = (xx / ts) & tmask, ty = (yy / ts) & tmask;
            int ent = top + (ty * 64 + tx) * 2;
            int hi = bgram[ent & 0x7FFF], code = bgram[(ent + 1) & 0x7FFF];
            int px = xx % ts, py = yy % ts;
            if (hi & 0x40) px = ts - 1 - px;
            if (hi & 0x80) py = ts - 1 - py;
            int c = size16 ? pcg16(code, px, py) : pcg8(code, px, py);
            if (c) { lay_op[layer][x] = 1; lay_col[layer][x] = pal565[1][((hi & 15) << 4) | c]; }
        }
    }
    /* sprites: priority 1,2,3 -> layers 0,2,4; lower index wins */
    for (int n = 127; n >= 0; n--) {
        const uint8_t *s = &sprite_regs[n * 8];
        int pr = s[6 ^ 1] & 3;  /* low byte of word at +6 (logical +7) */
        pr = sprite_regs[(n * 8 + 7) ^ 1] & 3;
        if (!pr) continue;
        int px = ((sprite_regs[(n * 8) ^ 1] << 8) | sprite_regs[(n * 8 + 1) ^ 1]) & 0x3FF;
        int py = ((sprite_regs[(n * 8 + 2) ^ 1] << 8) | sprite_regs[(n * 8 + 3) ^ 1]) & 0x3FF;
        int ctl = (sprite_regs[(n * 8 + 4) ^ 1] << 8) | sprite_regs[(n * 8 + 5) ^ 1];
        int row = y - bgv - (py - 16);
        if (row < 0 || row > 15) continue;
        int sx = px + hadj - 16;
        int hflip = (ctl & 0x4000) != 0, vflip = (ctl & 0x8000) != 0, code = ctl & 255, pal = (ctl >> 4) & 0xF0;
        int ry = vflip ? 15 - row : row;
        int layer = pr == 1 ? 0 : pr == 2 ? 2 : 4;
        for (int i = 0; i < 16; i++) {
            int x = sx + i;
            if (x < 0 || x >= w) continue;
            int c = pcg16(code, hflip ? 15 - i : i, ry);
            if (c) { lay_op[layer][x] = 1; lay_col[layer][x] = pal565[1][pal | c]; }
        }
    }
    for (int x = 0; x < w; x++) {
        for (int l = 4; l >= 0; l--) if (lay_op[l][x]) { sl_op[x] = 1; sl_col[x] = lay_col[l][x]; break; }
    }
}

/* ------------------------------------------------------- frame ---------- */
void video_reset(void) {
    memset(crtc_regs, 0, sizeof crtc_regs);
    memset(pal_raw, 0, sizeof pal_raw);
    memset(vcreg, 0, sizeof vcreg);
    memset(bg_regs, 0, sizeof bg_regs);
    memset(sprite_regs, 0, sizeof sprite_regs);
    memset(bgram, 0, sizeof bgram);
    crtc_mode_port = 0; cur_contrast = 15;
    rebuild_pals();
    crtc_derive();
    video_w = 768; video_h = 512; video_changed = 1;
}
int video_get_size(int *w, int *h) { *w = video_w; *h = video_h; return 0; }
static void compute_size(int *w, int *h) {
    int ww = ((int)crtc_hend - (int)crtc_hstart) * 8;
    int hh = (int)crtc_vend - (int)crtc_vstart;
    if (ww < 64 || ww > 1024) ww = 768;
    if (hh < 32 || hh > 1024) hh = 512;
    int m = crtc_regs[0x29] & 0x14;
    if (m == 0x10) hh /= 2; else if (m == 0x04) hh *= 2;
    if (hh > 1024) hh = 1024;
    *w = ww; *h = hh;
}
void video_begin_frame(void) {
    if (cur_contrast != sys_contrast) { cur_contrast = sys_contrast; rebuild_pals(); }
    int w, h; compute_size(&w, &h);
    if (w != video_w || h != video_h) { video_w = w; video_h = h; video_changed = 1; }
}

void video_render_line(int vline) {
    if (vline < (int)crtc_vstart || vline >= (int)crtc_vend) return;
    int rel = vline - crtc_vstart;
    int m = crtc_regs[0x29] & 0x14;
    int y, dup = 0;
    if (m == 0x10) { if (!(rel & 1)) return; y = rel >> 1; }
    else if (m == 0x04) { y = rel * 2; dup = 1; }
    else y = rel;
    if (y >= video_h) return;
    int w = video_w;
    for (int pass = 0; pass <= dup; pass++, y++) {
        if (y >= video_h) break;
        int tx_on = (vcreg[2][1] & 0x20) != 0;
        render_graphics(y, w);
        if (tx_on) render_text(y, w); else for (int x = 0; x < w; x++) tl_op[x] = 0;
        render_sprites(y, w);
        int gp = vcreg[1][0] & 3, sp = (vcreg[1][0] >> 4) & 3, tp = (vcreg[1][0] >> 2) & 3;
        int key[3] = { gp * 3 + 2, sp * 3 + 1, tp * 3 + 0 };  /* larger key = further back */
        int ord[3] = {0, 1, 2};
        for (int i = 0; i < 3; i++) for (int j = i + 1; j < 3; j++) if (key[ord[j]] > key[ord[i]]) { int t = ord[i]; ord[i] = ord[j]; ord[j] = t; }
        uint16_t bgc = pal565[1][0];
        uint16_t *out = &video_fb[y * video_w];
        for (int x = 0; x < w; x++) {
            uint16_t c = bgc;
            for (int k = 0; k < 3; k++) {
                int l = ord[k];
                if (l == 0) { if (gl_op[x]) c = gl_col[x]; }
                else if (l == 1) { if (sl_op[x]) c = sl_col[x]; }
                else { if (tl_op[x]) c = tl_col[x]; }
            }
            out[x] = c;
        }
    }
}

void video_state(void *s) {
    SER(s, crtc_regs); SER(s, crtc_mode_port); SER(s, pal_raw); SER(s, vcreg);
    SER(s, bg_regs); SER(s, sprite_regs); SER(s, bgram);
    if (((Ser *)s)->writing == 0) { crtc_derive(); rebuild_pals(); }
}
