/* dmac.c - HD63450 DMA controller (4 channels), behaviourally modelled */
#include "x68k.h"

typedef struct {
    uint8_t CSR, CER, DCR, OCR, SCR, CCR, NIV, EIV, MFC, CPR, DFC, BFC, GCR;
    uint16_t MTC, BTC;
    uint32_t MAR, DAR, BAR;
} Ch;
static Ch D[4];
static uint8_t int_ch, last_int;

static int ready(int ch) {
    switch (ch) {
    case 0: return fdc_data_ready();
    case 1: return sasi_data_ready();
    case 3: return adpcm_dma_ready();
    default: return 0;
    }
}
static void dma_int(int ch) { if (D[ch].CCR & 0x08) { int_ch |= (uint8_t)(1 << ch); irq_update(); } }
static void dma_err(int ch, int e) {
    D[ch].CER = (uint8_t)e; D[ch].CSR |= 0x10; D[ch].CSR &= 0xF7; D[ch].CCR &= 0x7F; dma_int(ch);
}
int dmac_pending(void) { return int_ch != 0; }
int dmac_ack(void) {
    int i = last_int, vec = 24 + 3;
    for (int n = 0; n < 4; n++, i = (i + 1) & 3) {
        if (int_ch & (1 << i)) {
            vec = (D[i].CSR & 0x10) ? D[i].EIV : D[i].NIV;
            int_ch &= (uint8_t)~(1 << i);
            i = (i + 1) & 3;
            break;
        }
    }
    last_int = (uint8_t)i;
    irq_update();
    return vec;
}
void dmac_reset(void) { memset(D, 0, sizeof D); int_ch = 0; last_int = 0; }

static uint32_t rd32(uint32_t a) {
    int b0 = dma_read8(a), b1 = dma_read8(a + 1), b2 = dma_read8(a + 2), b3 = dma_read8(a + 3);
    if ((b0 | b1 | b2 | b3) < 0) return 0xFFFFFFFFu;
    return ((uint32_t)b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}
static int rd16(uint32_t a) {
    int b0 = dma_read8(a), b1 = dma_read8(a + 1);
    if ((b0 | b1) < 0) return -1;
    return (b0 << 8) | b1;
}
static int mem_step(Ch *c, int n) { int d = 0; if (c->SCR & 4) d = n; else if (c->SCR & 8) d = -n; c->MAR += d; return d; }
static void dev_step(Ch *c, int n) { if (c->SCR & 1) c->DAR += n; else if (c->SCR & 2) c->DAR -= n; }

/* one transfer unit; returns bus error code (0 ok) */
static int unit(Ch *c) {
    int dev2mem = c->OCR & 0x80;
    int kind = ((c->OCR >> 4) & 3) + ((c->DCR >> 1) & 4);
    int nbytes, port16 = 0;
    switch (kind) {
    case 0: case 3: nbytes = 1; break;
    case 1: nbytes = 2; break;
    case 2: nbytes = 4; break;
    case 4: nbytes = 1; port16 = 1; break;
    case 5: nbytes = 2; port16 = 1; break;
    case 6: nbytes = 4; port16 = 1; break;
    default: nbytes = 1; break;
    }
    if (!port16 || nbytes == 1) {
        for (int i = 0; i < nbytes; i++) {
            uint32_t s = dev2mem ? c->DAR : c->MAR, d = dev2mem ? c->MAR : c->DAR;
            int v = dma_read8(s);
            if (v < 0) return dev2mem ? 1 : 2;
            if (dma_write8(d, (uint8_t)v) < 0) return dev2mem ? 2 : 1;
            mem_step(c, 1);
            dev_step(c, port16 ? 1 : 2);
        }
    } else {
        for (int i = 0; i < nbytes; i += 2) {
            uint32_t s = dev2mem ? c->DAR : c->MAR, d = dev2mem ? c->MAR : c->DAR;
            int v = rd16(s);
            if (v < 0) return dev2mem ? 1 : 2;
            if (dma_write8(d, (uint8_t)(v >> 8)) < 0 || dma_write8(d + 1, (uint8_t)v) < 0) return dev2mem ? 2 : 1;
            mem_step(c, 2);
            dev_step(c, 2);
        }
    }
    return 0;
}

static uint8_t in_exec[4];
void dmac_exec(int ch) {
    Ch *c = &D[ch];
    int guard = 0;
    if (in_exec[ch]) return;
    in_exec[ch] = 1;
    while ((c->CSR & 0x08) && !(c->CCR & 0x20) && !(c->CSR & 0x80) && c->MTC &&
           (((c->OCR & 3) != 2) || ready(ch)) && guard++ < 0x20000) {
        int e = unit(c);
        if (e) {
            int d2m = c->OCR & 0x80;
            dma_err(ch, e == 1 ? (d2m ? 0x0A : 0x09) : (d2m ? 0x09 : 0x0A));
            break;
        }
        c->MTC--;
        if (!c->MTC) {
            if (c->OCR & 8) {
                if (c->OCR & 4) {
                    if (c->BAR) {
                        c->MAR = rd32(c->BAR); c->MTC = (uint16_t)rd16(c->BAR + 4); c->BAR = rd32(c->BAR + 6);
                        if (!c->MTC) { dma_err(ch, 0x0D); break; }
                    }
                } else {
                    c->BTC--;
                    if (c->BTC) {
                        c->MAR = rd32(c->BAR); c->MTC = (uint16_t)rd16(c->BAR + 4); c->BAR += 6;
                        if (!c->MTC) { dma_err(ch, 0x0D); break; }
                    }
                }
            } else if (c->CCR & 0x40) {
                c->CSR |= 0x40; dma_int(ch);
                if (c->BAR) {
                    c->MAR = c->BAR; c->MTC = c->BTC; c->CSR |= 0x08; c->BAR = 0; c->BTC = 0;
                    if (!c->MTC) { dma_err(ch, 0x0D); break; }
                    c->CCR &= 0xBF;
                }
            }
            if (!c->MTC) { c->CSR |= 0x80; c->CSR &= 0xF7; dma_int(ch); }
        }
    }
    in_exec[ch] = 0;
}
void dmac_run(void) { dmac_exec(0); dmac_exec(1); dmac_exec(2); }

uint8_t dmac_read(uint32_t a) {
    int off = a & 0x3F, ch = (a >> 6) & 3;
    if (a >= 0xE84100) return 0;
    Ch *c = &D[ch];
    switch (off) {
    case 0x00: return c->CSR; case 0x01: return c->CER;
    case 0x04: return c->DCR; case 0x05: return c->OCR; case 0x06: return c->SCR; case 0x07: return c->CCR;
    case 0x0A: return c->MTC >> 8; case 0x0B: return c->MTC & 0xFF;
    case 0x0C: return c->MAR >> 24; case 0x0D: return c->MAR >> 16; case 0x0E: return c->MAR >> 8; case 0x0F: return c->MAR;
    case 0x14: return c->DAR >> 24; case 0x15: return c->DAR >> 16; case 0x16: return c->DAR >> 8; case 0x17: return c->DAR;
    case 0x1A: return c->BTC >> 8; case 0x1B: return c->BTC & 0xFF;
    case 0x1C: return c->BAR >> 24; case 0x1D: return c->BAR >> 16; case 0x1E: return c->BAR >> 8; case 0x1F: return c->BAR;
    case 0x25: return c->NIV; case 0x27: return c->EIV; case 0x29: return c->MFC; case 0x2D: return c->CPR;
    case 0x31: return c->DFC; case 0x39: return c->BFC; case 0x3F: return c->GCR;
    }
    return 0;
}
void dmac_write(uint32_t a, uint8_t v) {
    int off = a & 0x3F, ch = (a >> 6) & 3;
    if (a >= 0xE84100) return;
    Ch *c = &D[ch];
    switch (off) {
    case 0x00: c->CSR &= (uint8_t)((~v) | 0x09); break;
    case 0x01: c->CER &= (uint8_t)~v; break;
    case 0x04: c->DCR = v; break;
    case 0x05: c->OCR = v; break;
    case 0x06: c->SCR = v; break;
    case 0x07: {
        uint8_t old = c->CCR;
        c->CCR = (uint8_t)((v & 0xEF) | (c->CCR & 0x80));
        if ((v & 0x10) && (c->CCR & 0x80)) { dma_err(ch, 0x11); break; }
        if (v & 0x20) break;
        if (v & 0x80) {
            if (old & 0x20) { c->CSR |= 0x08; dmac_exec(ch); }
            else {
                if (c->CSR & 0xF8) { dma_err(ch, 0x02); break; }
                c->CSR |= 0x08;
                if (c->OCR & 8) {
                    c->MAR = rd32(c->BAR) & 0xFFFFFF; c->MTC = (uint16_t)rd16(c->BAR + 4);
                    if (c->OCR & 4) c->BAR = rd32(c->BAR + 6);
                    else { c->BAR += 6; if (!c->BTC) { dma_err(ch, 0x0F); break; } }
                }
                if (!c->MTC) { dma_err(ch, 0x0D); break; }
                c->CER = 0;
                dmac_exec(ch);
            }
        }
        if ((v & 0x40) && !c->MTC) {
            if (c->CCR & 0x80) {
                if (c->CCR & 0x40) dma_err(ch, 0x02);
                else if (c->OCR & 8) dma_err(ch, 0x01);
                else {
                    c->MAR = c->BAR; c->MTC = c->BTC; c->CSR |= 0x08; c->BAR = 0; c->BTC = 0;
                    if (!c->MAR) { c->CSR |= 0x40; dma_int(ch); break; }
                    else if (!c->MTC) { dma_err(ch, 0x0D); break; }
                    c->CCR &= 0xBF;
                    dmac_exec(ch);
                }
            } else dma_err(ch, 0x02);
        }
        break; }
    case 0x0A: c->MTC = (uint16_t)((c->MTC & 0xFF) | (v << 8)); break;
    case 0x0B: c->MTC = (uint16_t)((c->MTC & 0xFF00) | v); break;
    case 0x0C: break;
    case 0x0D: c->MAR = (c->MAR & 0xFF00FFFFu) | ((uint32_t)v << 16); break;
    case 0x0E: c->MAR = (c->MAR & 0xFFFF00FFu) | ((uint32_t)v << 8); break;
    case 0x0F: c->MAR = (c->MAR & 0xFFFFFF00u) | v; break;
    case 0x14: break;
    case 0x15: c->DAR = (c->DAR & 0xFF00FFFFu) | ((uint32_t)v << 16); break;
    case 0x16: c->DAR = (c->DAR & 0xFFFF00FFu) | ((uint32_t)v << 8); break;
    case 0x17: c->DAR = (c->DAR & 0xFFFFFF00u) | v; break;
    case 0x1A: c->BTC = (uint16_t)((c->BTC & 0xFF) | (v << 8)); break;
    case 0x1B: c->BTC = (uint16_t)((c->BTC & 0xFF00) | v); break;
    case 0x1C: break;
    case 0x1D: c->BAR = (c->BAR & 0xFF00FFFFu) | ((uint32_t)v << 16); break;
    case 0x1E: c->BAR = (c->BAR & 0xFFFF00FFu) | ((uint32_t)v << 8); break;
    case 0x1F: c->BAR = (c->BAR & 0xFFFFFF00u) | v; break;
    case 0x25: c->NIV = v; break;
    case 0x27: c->EIV = v; break;
    case 0x29: c->MFC = v; break;
    case 0x2D: c->CPR = v; break;
    case 0x31: c->DFC = v; break;
    case 0x39: c->BFC = v; break;
    case 0x3F: if (ch == 3) c->GCR = v; break;
    }
}
void dmac_state(void *s) { SER(s, D); SER(s, int_ch); SER(s, last_int); }
