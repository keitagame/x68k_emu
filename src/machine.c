/* machine.c - X68000 memory map, interrupt routing and frame scheduler */
#include "x68k.h"

uint8_t ram[RAM_MAX];
uint8_t tvram[0x80000];
uint8_t gvram[0x80000];
uint8_t sram[0x4000];
uint8_t ipl_rom[0x20000];
uint8_t *cg_rom;
uint32_t ram_size = 2u << 20;
int sram_wen;
Config cfg = { 10, 2 };

int mach_vline, mach_line10 = 317, mach_cpu_mhz = 10;
extern uint8_t ioc_stat, ioc_vect, ioc_pend;
void cpu_bus_error(uint32_t a, int rw);
void mfp_line_end_events(int vline);
static int g_dma, g_dma_err;

static void berr(uint32_t a, int rw) { if (g_dma) g_dma_err = 1; else cpu_bus_error(a, rw); }

/* ------------------------------------------------------ read / write -- */
uint8_t mem_read8(uint32_t a) {
    a &= 0xFFFFFF;
    if (a < 0xC00000) { if (a < ram_size) return ram[a ^ 1]; berr(a, 1); return 0xFF; }
    if (a < 0xE00000) return gvram[(a & 0x7FFFF) ^ 1];
    if (a < 0xE80000) return tvram[(a & 0x7FFFF) ^ 1];
    if (a < 0xEA0000) {
        switch ((a - 0xE80000) >> 13) {
        case 0: return crtc_read(a);
        case 1: return vctrl_read(a);
        case 2: return dmac_read(a);
        case 4: return mfp_read(a);
        case 5: return rtc_read(a);
        case 7: return sysport_read(a);
        case 8: return opm_read(a);
        case 9: return adpcm_read(a);
        case 10: return fdc_read(a);
        case 11: return sasi_read(a);
        case 12: return scc_read(a);
        case 13: return pia_read(a);
        case 14: return a == 0xE9C001 ? ioc_stat : 0xFF;
        default: return 0;
        }
    }
    if (a < 0xEB0000) { berr(a, 1); return 0xFF; }
    if (a < 0xEC0000) return bg_read(a);
    if (a < 0xED0000) { berr(a, 1); return 0xFF; }
    if (a < 0xEE0000) return (a & 0xFFFF) < 0x4000 ? sram[a & 0x3FFF] : 0xFF;
    if (a < 0xF00000) { berr(a, 1); return 0xFF; }
    if (a < 0xFC0000) return cg_rom ? cg_rom[a - 0xF00000] : 0;
    return ipl_rom[a & 0x1FFFF];
}
void mem_write8(uint32_t a, uint8_t v) {
    a &= 0xFFFFFF;
    if (a < 0xC00000) { if (a < ram_size) ram[a ^ 1] = v; else berr(a, 0); return; }
    if (a < 0xE00000) { gvram[(a & 0x7FFFF) ^ 1] = v; return; }
    if (a < 0xE80000) { tvram_write8(a, v); return; }
    if (a < 0xEA0000) {
        switch ((a - 0xE80000) >> 13) {
        case 0: crtc_write(a, v); break;
        case 1: vctrl_write(a, v); break;
        case 2: dmac_write(a, v); break;
        case 4: mfp_write(a, v); break;
        case 5: rtc_write(a, v); break;
        case 7: sysport_write(a, v); break;
        case 8: opm_write(a, v); break;
        case 9: adpcm_write(a, v); break;
        case 10: fdc_write(a, v); break;
        case 11: sasi_write(a, v); break;
        case 12: scc_write(a, v); break;
        case 13: pia_write(a, v); break;
        case 14:
            if (a == 0xE9C001) { ioc_stat = (uint8_t)((ioc_stat & 0xF0) | (v & 0x0F)); }
            else if (a == 0xE9C003) ioc_vect = v & 0xFC;
            break;
        default: break;
        }
        return;
    }
    if (a < 0xEB0000) { berr(a, 0); return; }
    if (a < 0xEC0000) { bg_write(a, v); return; }
    if (a < 0xED0000) { berr(a, 0); return; }
    if (a < 0xEE0000) { if (sram_wen && (a & 0xFFFF) < 0x4000) sram[a & 0x3FFF] = v; return; }
    berr(a, 0); /* ROM / unmapped */
}
uint16_t mem_read16(uint32_t a) {
    a &= 0xFFFFFF;
    if (a < 0xC00000) { if (a + 1 < ram_size) return *(uint16_t *)&ram[a]; }
    else if (a < 0xE00000) return *(uint16_t *)&gvram[a & 0x7FFFE];
    else if (a < 0xE80000) return *(uint16_t *)&tvram[a & 0x7FFFE];
    return (uint16_t)((mem_read8(a) << 8) | mem_read8(a + 1));
}
void mem_write16(uint32_t a, uint16_t v) {
    a &= 0xFFFFFF;
    if (a < 0xC00000 && a + 1 < ram_size) { *(uint16_t *)&ram[a] = v; return; }
    if (a >= 0xC00000 && a < 0xE00000) { *(uint16_t *)&gvram[a & 0x7FFFE] = v; return; }
    mem_write8(a, (uint8_t)(v >> 8)); mem_write8(a + 1, (uint8_t)v);
}
uint32_t mem_read32(uint32_t a) { return ((uint32_t)mem_read16(a) << 16) | mem_read16(a + 2); }
void mem_write32(uint32_t a, uint32_t v) { mem_write16(a, (uint16_t)(v >> 16)); mem_write16(a + 2, (uint16_t)v); }

int dma_read8(uint32_t a) { g_dma = 1; g_dma_err = 0; uint8_t v = mem_read8(a); g_dma = 0; return g_dma_err ? -1 : v; }
int dma_write8(uint32_t a, uint8_t v) { g_dma = 1; g_dma_err = 0; mem_write8(a, v); g_dma = 0; return g_dma_err ? -1 : 0; }

/* CPU-side interface */
uint8_t  bus_rd8(uint32_t a) { return mem_read8(a); }
uint16_t bus_rd16(uint32_t a) { return mem_read16(a); }
void     bus_wr8(uint32_t a, uint8_t v) { mem_write8(a, v); }
void     bus_wr16(uint32_t a, uint16_t v) { mem_write16(a, v); }
void     bus_reset_instr(void) { }

/* ------------------------------------------------------- interrupts --- */
void irq_update(void) {
    int lvl = 0;
    if (mfp_pending()) lvl = 6;
    else if (scc_pending()) lvl = 5;
    else if (dmac_pending()) lvl = 3;
    else if (ioc_pend) lvl = 1;
    cpu_set_irq(lvl);
}
int bus_irq_ack(int level) {
    int v;
    switch (level) {
    case 6: v = mfp_ack(); break;
    case 5: v = scc_ack(); break;
    case 3: v = dmac_ack(); break;
    case 1:
        if (ioc_pend & IOC_FDC) { ioc_pend &= (uint8_t)~IOC_FDC; v = ioc_vect; }
        else if (ioc_pend & IOC_FDD) { ioc_pend &= (uint8_t)~IOC_FDD; v = ioc_vect | 1; }
        else { ioc_pend &= (uint8_t)~IOC_HDD; v = ioc_vect | 2; }
        break;
    default: v = 24 + level; break;
    }
    irq_update();
    return v;
}

/* -------------------------------------------------------- lifecycle --- */
static void sram_defaults(void) {
    /* if no SRAM image was loaded the IPL ROM will initialise it itself */
}
void machine_init(void) {
    memset(sram, 0xFF, sizeof sram);
    sram_defaults();
    sound_init();
}
void machine_reset(void) {
    ram_size = (uint32_t)cfg.ram_mb << 20;
    if (ram_size > RAM_MAX) ram_size = RAM_MAX;
    memset(ram, 0, sizeof ram);
    memset(tvram, 0, sizeof tvram);
    memset(gvram, 0, sizeof gvram);
    sram_wen = 0;
    mach_cpu_mhz = cfg.cpu_mhz;
    periph_reset(); video_reset(); dmac_reset(); fdc_reset(); sasi_reset(); sound_reset();
    uint32_t ssp = ((uint32_t)ipl_rom[0x10000] << 24) | (ipl_rom[0x10001] << 16) | (ipl_rom[0x10002] << 8) | ipl_rom[0x10003];
    uint32_t pc  = ((uint32_t)ipl_rom[0x10004] << 24) | (ipl_rom[0x10005] << 16) | (ipl_rom[0x10006] << 8) | ipl_rom[0x10007];
    cpu_reset(ssp, pc);
    irq_update();
}

/* ------------------------------------------------------ frame loop ---- */
static int cpu_budget;
void machine_run_frame(void) {
    uint32_t sz = ((uint32_t)sram[8] << 24) | (sram[9] << 16) | (sram[10] << 8) | sram[11];
    if (sz != ram_size) { sram[8] = (uint8_t)(ram_size >> 24); sram[9] = (uint8_t)(ram_size >> 16); sram[10] = (uint8_t)(ram_size >> 8); sram[11] = (uint8_t)ram_size; }
    mach_cpu_mhz = cfg.cpu_mhz;
    video_begin_frame();
    int vtotal = crtc_vtotal ? crtc_vtotal : 568;
    int frame10 = video_hfreq_hi ? 180310 : 162200;
    int key_cnt = 0, mouse_cnt = 0;
    int acc10 = 0;
    for (int vline = 0; vline < vtotal; vline++) {
        mach_vline = vline;
        int next10 = (int)((int64_t)frame10 * (vline + 1) / vtotal);
        int line10 = next10 - acc10; acc10 = next10;
        mach_line10 = line10;
        mfp_line_events(vline);
        cpu_budget += (int)((int64_t)line10 * cfg.cpu_mhz / 10);
        int used = cpu_budget > 0 ? cpu_execute(cpu_budget) : 0;
        cpu_budget -= used;
        mfp_run(line10); rtc_run(line10); sound_run(line10);
        dmac_run();
        mfp_timera_event();
        mfp_line_end_events(vline);
        video_render_line(vline);
        if (++key_cnt > vtotal / 4) { key_cnt = 0; keyboard_tick(); }
        if (++mouse_cnt > vtotal / 8) { mouse_cnt = 0; scc_int_check(); }
    }
}

void machine_state(void *s) {
    SER(s, cpu); SER(s, cpu_budget); SER(s, ram_size);
    ser_blob((Ser *)s, ram, ram_size);
    ser_blob((Ser *)s, gvram, sizeof gvram);
    ser_blob((Ser *)s, tvram, sizeof tvram);
    ser_blob((Ser *)s, sram, sizeof sram);
    SER(s, sram_wen);
    periph_state(s); video_state(s); dmac_state(s); fdc_state(s); sasi_state(s); sound_state(s);
}
