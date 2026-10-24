/* x68k.h - shared declarations for the X68000 core */
#ifndef X68K_H
#define X68K_H
#include <stdint.h>
#include <stddef.h>
#include "cpu.h"

/* ---- runtime (host.c) ---- */
void *memcpy(void *d, const void *s, size_t n);
void *memmove(void *d, const void *s, size_t n);
void *memset(void *d, int c, size_t n);
int   memcmp(const void *a, const void *b, size_t n);
size_t strlen(const char *s);
int   strcmp(const char *a, const char *b);
void *malloc(size_t n);
void  free(void *p);
void  log_msg(int level, const char *msg);
double host_now_ms(void);
int   host_localtime(double ms, int *tm9);   /* sec,min,hour,mday,mon,year-1900,wday */
/* file access through the host virtual filesystem */
int   hfile_open(const char *path, int rw);   /* returns fd >=0 or <0 */
int   hfile_read(int fd, void *buf, int n);
int   hfile_write(int fd, const void *buf, int n);
long  hfile_seek(int fd, long off, int whence);
void  hfile_close(int fd);
long  hfile_size(int fd);
void *hfile_load(const char *path, long *size);  /* malloc'd */

/* ---- machine (machine.c) ---- */
#define RAM_MAX (12u << 20)
extern uint8_t  ram[RAM_MAX];
extern uint8_t  tvram[0x80000];      /* stored with address^1 (LE word layout) */
extern uint8_t  gvram[0x80000];      /* same layout */
extern uint8_t  sram[0x4000];        /* logical byte order */
extern uint8_t  ipl_rom[0x20000];    /* FE0000-FFFFFF, logical order */
extern uint8_t *cg_rom;              /* 0xC0000 bytes at F00000, logical order */
extern uint32_t ram_size;
extern int      sram_wen;

typedef struct { int cpu_mhz; int ram_mb; } Config;
extern Config cfg;

void machine_init(void);
void machine_reset(void);
void machine_run_frame(void);
void machine_state(void *ser);

uint8_t  mem_read8(uint32_t a);
uint16_t mem_read16(uint32_t a);
uint32_t mem_read32(uint32_t a);
void     mem_write8(uint32_t a, uint8_t v);
void     mem_write16(uint32_t a, uint16_t v);
void     mem_write32(uint32_t a, uint32_t v);
/* DMA / debug access: never raises bus errors; returns -1 on bus error */
int      dma_read8(uint32_t a);
int      dma_write8(uint32_t a, uint8_t v);

void irq_update(void);
/* IOC */
#define IOC_FDC 0x80
#define IOC_FDD 0x40
#define IOC_HDD 0x10
void ioc_irq(int statbit);
/* MFP */
void mfp_init(void);
void mfp_int(int irq);            /* irq numbering: 0..7 -> IPRA bit 7..0, 8..15 -> IPRB */
void mfp_run(int cyc10);
void mfp_line_events(int vline);
void mfp_timera_event(void);
uint8_t mfp_read(uint32_t a);
void    mfp_write(uint32_t a, uint8_t v);
int  mfp_pending(void);
int  mfp_ack(void);
void mfp_key_data(uint8_t k);
extern int mfp_gpip_hsync, mfp_gpip_vdisp, mfp_gpip_cirq, mfp_gpip_fm;
/* CRTC / video (video.c) */
uint8_t crtc_read(uint32_t a);
void    crtc_write(uint32_t a, uint8_t v);
uint8_t vctrl_read(uint32_t a);
void    vctrl_write(uint32_t a, uint8_t v);
uint8_t bg_read(uint32_t a);
void    bg_write(uint32_t a, uint8_t v);
void    tvram_write8(uint32_t a, uint8_t v);
void    video_reset(void);
void    video_state(void *ser);
void    video_render_line(int vline);
void    video_begin_frame(void);
int     video_get_size(int *w, int *h);
extern uint16_t video_fb[1024 * 1024];
extern int video_w, video_h, video_changed;
extern uint8_t  crtc_regs[0x30];
extern uint16_t crtc_vstart, crtc_vend, crtc_hstart, crtc_hend, crtc_vtotal, crtc_intline;
extern uint32_t text_scroll_x, text_scroll_y;
extern int video_hfreq_hi;
void    crtc_fast_clear(void);
/* sound (sound.c) */
void    sound_init(void);
void    sound_reset(void);
uint8_t opm_read(uint32_t a);
void    opm_write(uint32_t a, uint8_t v);
uint8_t adpcm_read(uint32_t a);
void    adpcm_write(uint32_t a, uint8_t v);
void    adpcm_pan(int v);
int     adpcm_dma_ready(void);
void    sound_run(int cyc10);
int     sound_render(int16_t *out, int frames);  /* stereo interleaved */
void    sound_state(void *ser);
int     opm_irq_line(void);
/* DMAC (dmac.c) */
uint8_t dmac_read(uint32_t a);
void    dmac_write(uint32_t a, uint8_t v);
void    dmac_exec(int ch);
void    dmac_run(void);
int     dmac_pending(void);
int     dmac_ack(void);
void    dmac_state(void *ser);
void    dmac_reset(void);
/* FDC / FDD (fdc.c) */
uint8_t fdc_read(uint32_t a);
void    fdc_write(uint32_t a, uint8_t v);
void    fdc_reset(void);
int     fdc_data_ready(void);
int     fdd_insert(int drive, const char *path);
void    fdd_eject(int drive);
int     fdd_ready(int drive);
extern int fdc_is_reading, fdc_access_drive, fdd_dirty_mask;
void    fdc_state(void *ser);
/* SASI (sasi.c) */
uint8_t sasi_read(uint32_t a);
void    sasi_write(uint32_t a, uint8_t v);
int     sasi_load(const char *path);
int     sasi_data_ready(void);
void    sasi_reset(void);
void    sasi_state(void *ser);
extern int sasi_accessing, sasi_dirty_flag;
extern uint32_t sasi_stat[8];
/* SCC / mouse / serial (periph.c) */
uint8_t scc_read(uint32_t a);
void    scc_write(uint32_t a, uint8_t v);
int     scc_pending(void);
int     scc_ack(void);
void    scc_int_check(void);
void    scc_reset(void);
void    mouse_add(int dx, int dy, int l, int r);
/* keyboard / joystick */
void    key_event(int scancode, int down);
void    keyboard_tick(void);
uint8_t pia_read(uint32_t a);
void    pia_write(uint32_t a, uint8_t v);
void    set_joy(int port, uint8_t bits);
uint8_t rtc_read(uint32_t a);
void    rtc_write(uint32_t a, uint8_t v);
void    rtc_run(int cyc10);
uint8_t sysport_read(uint32_t a);
void    sysport_write(uint32_t a, uint8_t v);
void    periph_reset(void);
void    periph_state(void *ser);
extern uint8_t sys_contrast;
/* serialization helper */
typedef struct { uint8_t *p; size_t pos, cap; int writing; int err; } Ser;
void ser_blob(Ser *s, void *data, size_t n);
#define SER(s, v) ser_blob((Ser *)(s), &(v), sizeof(v))
#endif
