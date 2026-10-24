/* core.c - libretro front end + the webx68k_* / get_* exports used by the WebX68k host (core.js) */
#include "x68k.h"
#include "../third_party/libretro.h"
#include "keymap.inc"

#ifdef __wasm__
#define EXPORT __attribute__((used, visibility("default")))
#else
#define EXPORT __attribute__((used, visibility("default")))
#endif

static retro_environment_t env_cb;
static retro_video_refresh_t video_cb;
static retro_audio_sample_t audio_cb;
static retro_audio_sample_batch_t audio_batch_cb;
static retro_input_poll_t poll_cb;
static retro_input_state_t input_cb;

static int loaded, ipl_ok;
static char sysdir[256] = "/system";
static char path_buf[320];
static int cur_fps_x100;
static uint8_t keydown[128];
static int16_t abuf[2048 * 2];
extern int mouse_last_dx, mouse_last_dy, mouse_last_stat;
int serial_rx(const uint8_t *p, int n);
int serial_tx_available(void);
int serial_tx_drain(uint8_t *p, int n);
void serial_reset(void);
void serial_set_connected(int c);
void serial_set_tx_writable(int c);
int serial_baud(void);
int get_mouse_scc_x(void); int get_mouse_scc_y(void); int get_mouse_scc_stat(void); int get_mouse_enabled(void);

/* ------------------------------------------------------------- helpers -- */
static void lg(const char *m) { log_msg(1, m); }
static int get_var_int(const char *key, int def) {
    struct retro_variable v; v.key = key; v.value = 0;
    if (env_cb && env_cb(RETRO_ENVIRONMENT_GET_VARIABLE, &v) && v.value) {
        const char *p = v.value; int n = 0, any = 0;
        while (*p >= '0' && *p <= '9') { n = n * 10 + (*p - '0'); p++; any = 1; }
        if (any) return n;
    }
    return def;
}
static void read_options(int at_load) {
    int mhz = get_var_int("px68k_cpuspeed", 10);
    if (mhz < 1) mhz = 10; if (mhz > 200) mhz = 200;
    cfg.cpu_mhz = mhz;
    if (at_load) {
        int mb = get_var_int("px68k_ramsize", 2);
        if (mb < 1) mb = 1; if (mb > 12) mb = 12;
        cfg.ram_mb = mb;
    }
}
static int load_rom(const char *name, uint8_t *dst, long maxsz, long *got) {
    int n = 0; const char *a = sysdir, *b = "/keropi/";
    while (*a && n < 250) path_buf[n++] = *a++;
    while (*b) path_buf[n++] = *b++;
    a = name; while (*a && n < 310) path_buf[n++] = *a++;
    path_buf[n] = 0;
    long sz = 0; uint8_t *d = hfile_load(path_buf, &sz);
    if (!d) return 0;
    long take = sz < maxsz ? sz : maxsz;
    if (sz > maxsz) memcpy(dst, d + (sz - maxsz), (size_t)maxsz);   /* keep the tail (IPL at the end) */
    else memcpy(dst, d, (size_t)take);
    free(d);
    if (got) *got = sz;
    return 1;
}
static int ends_with_ci(const char *s, const char *ext) {
    int a = (int)strlen(s), b = (int)strlen(ext);
    if (a < b) return 0;
    for (int i = 0; i < b; i++) { char x = s[a - b + i]; if (x >= 'A' && x <= 'Z') x += 32; if (x != ext[i]) return 0; }
    return 1;
}
static void notify_av(void) {
    int w, h; video_get_size(&w, &h);
    int fps100 = video_hfreq_hi ? 5546 : 6165;
    if (!env_cb) return;
    if (fps100 != cur_fps_x100) {
        struct retro_system_av_info av;
        av.geometry.base_width = (unsigned)w; av.geometry.base_height = (unsigned)h;
        av.geometry.max_width = 1024; av.geometry.max_height = 1024; av.geometry.aspect_ratio = 4.0f / 3.0f;
        av.timing.fps = fps100 / 100.0; av.timing.sample_rate = 44100.0;
        cur_fps_x100 = fps100;
        env_cb(RETRO_ENVIRONMENT_SET_SYSTEM_AV_INFO, &av);
    } else {
        struct retro_game_geometry g;
        g.base_width = (unsigned)w; g.base_height = (unsigned)h; g.max_width = 1024; g.max_height = 1024; g.aspect_ratio = 4.0f / 3.0f;
        env_cb(RETRO_ENVIRONMENT_SET_GEOMETRY, &g);
    }
}

/* --------------------------------------------------------- libretro API -- */
EXPORT unsigned retro_api_version(void) { return RETRO_API_VERSION; }
EXPORT void retro_set_environment(retro_environment_t cb) {
    env_cb = cb;
    bool t = true;
    cb(RETRO_ENVIRONMENT_SET_SUPPORT_NO_GAME, &t);
}
EXPORT void retro_set_video_refresh(retro_video_refresh_t cb) { video_cb = cb; }
EXPORT void retro_set_audio_sample(retro_audio_sample_t cb) { audio_cb = cb; }
EXPORT void retro_set_audio_sample_batch(retro_audio_sample_batch_t cb) { audio_batch_cb = cb; }
EXPORT void retro_set_input_poll(retro_input_poll_t cb) { poll_cb = cb; }
EXPORT void retro_set_input_state(retro_input_state_t cb) { input_cb = cb; }
EXPORT void retro_set_controller_port_device(unsigned port, unsigned device) { (void)port; (void)device; }
EXPORT void retro_get_system_info(struct retro_system_info *info) {
    info->library_name = "x68core";
    info->library_version = "0.1";
    info->valid_extensions = "dim|xdf|d88|88d|hdm|dup|2hd|img|hdf";
    info->need_fullpath = true;
    info->block_extract = false;
}
EXPORT void retro_get_system_av_info(struct retro_system_av_info *info) {
    int w, h; video_get_size(&w, &h);
    info->geometry.base_width = (unsigned)w; info->geometry.base_height = (unsigned)h;
    info->geometry.max_width = 1024; info->geometry.max_height = 1024;
    info->geometry.aspect_ratio = 4.0f / 3.0f;
    info->timing.fps = video_hfreq_hi ? 55.46 : 61.65;
    info->timing.sample_rate = 44100.0;
}
EXPORT void retro_init(void) {
    enum retro_pixel_format fmt = RETRO_PIXEL_FORMAT_RGB565;
    if (env_cb) env_cb(RETRO_ENVIRONMENT_SET_PIXEL_FORMAT, &fmt);
    const char *d = 0;
    if (env_cb && env_cb(RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY, &d) && d && *d) {
        int n = 0; while (d[n] && n < 250) { sysdir[n] = d[n]; n++; } sysdir[n] = 0;
    }
    if (!cg_rom) cg_rom = malloc(0xC0000);
    if (cg_rom) memset(cg_rom, 0, 0xC0000);
    machine_init();
}
EXPORT void retro_deinit(void) { loaded = 0; }
EXPORT bool retro_load_game(const struct retro_game_info *info) {
    read_options(1);
    ipl_ok = 0;
    static const char *ipls[] = { "iplrom.dat", "iplromxv.dat", "iplromco.dat", "iplrom30.dat" };
    memset(ipl_rom, 0, sizeof ipl_rom);
    for (unsigned i = 0; i < sizeof ipls / sizeof *ipls && !ipl_ok; i++) ipl_ok = load_rom(ipls[i], ipl_rom, 0x20000, 0);
    if (!ipl_ok) { lg("IPL ROM (keropi/iplrom.dat) not found"); return false; }
    long gotcg = 0;
    if (cg_rom) { memset(cg_rom, 0, 0xC0000); if (!load_rom("cgrom.dat", cg_rom, 0xC0000, &gotcg)) load_rom("cgrom.tmp", cg_rom, 0xC0000, &gotcg); }
    /* SRAM image (logical byte order) */
    memset(sram, 0xFF, sizeof sram);
    { uint8_t tmp[0x4000]; long got = 0; memset(tmp, 0xFF, sizeof tmp);
      if (load_rom("sram.dat", tmp, 0x4000, &got) && got >= 0x4000) memcpy(sram, tmp, sizeof sram); }
    machine_reset();
    if (info && info->path && *info->path) {
        const char *p = info->path;
        if (ends_with_ci(p, ".hdf")) sasi_load(p);
        else fdd_insert(0, p);
    }
    cur_fps_x100 = video_hfreq_hi ? 5546 : 6165;
    loaded = 1;
    return true;
}
EXPORT bool retro_load_game_special(unsigned t, const struct retro_game_info *i, size_t n) { (void)t; (void)i; (void)n; return false; }
EXPORT void retro_unload_game(void) { loaded = 0; }
EXPORT void retro_reset(void) {
    if (!loaded) return;
    read_options(1);
    machine_reset();
}
EXPORT unsigned retro_get_region(void) { return RETRO_REGION_NTSC; }

static void poll_input(void) {
    if (!input_cb) return;
    poll_cb();
    /* keyboard: OR together all libretro keys that map onto the same scancode */
    uint8_t now[128]; memset(now, 0, sizeof now);
    for (unsigned i = 0; i < sizeof keymap / sizeof *keymap; i++)
        if (input_cb(0, RETRO_DEVICE_KEYBOARD, 0, keymap[i].retrok)) now[keymap[i].sc & 127] = 1;
    for (int sc = 1; sc < 128; sc++)
        if (now[sc] != keydown[sc]) { keydown[sc] = now[sc]; key_event(sc, now[sc]); }
    /* mouse */
    int dx = input_cb(0, RETRO_DEVICE_MOUSE, 0, RETRO_DEVICE_ID_MOUSE_X);
    int dy = input_cb(0, RETRO_DEVICE_MOUSE, 0, RETRO_DEVICE_ID_MOUSE_Y);
    int l = input_cb(0, RETRO_DEVICE_MOUSE, 0, RETRO_DEVICE_ID_MOUSE_LEFT);
    int r = input_cb(0, RETRO_DEVICE_MOUSE, 0, RETRO_DEVICE_ID_MOUSE_RIGHT);
    mouse_add(dx, dy, l, r);
    /* joypads: UP DOWN LEFT RIGHT = bits 0..3, button A/B = bits 5/6 */
    for (int port = 0; port < 2; port++) {
        uint8_t b = 0;
        if (input_cb(port, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP)) b |= 1;
        if (input_cb(port, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN)) b |= 2;
        if (input_cb(port, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT)) b |= 4;
        if (input_cb(port, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT)) b |= 8;
        if (input_cb(port, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B)) b |= 0x20;
        if (input_cb(port, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A)) b |= 0x40;
        set_joy(port, b);
    }
}
EXPORT void retro_run(void) {
    if (!loaded) return;
    bool upd = false;
    if (env_cb && env_cb(RETRO_ENVIRONMENT_GET_VARIABLE_UPDATE, &upd) && upd) read_options(0);
    poll_input();
    machine_run_frame();
    if (video_changed) { video_changed = 0; notify_av(); }
    if (video_cb) video_cb(video_fb, (unsigned)video_w, (unsigned)video_h, (size_t)video_w * 2);
    if (audio_batch_cb) {
        int n;
        while ((n = sound_render(abuf, 2048)) > 0) audio_batch_cb(abuf, (size_t)n);
    }
}

/* ------------------------------------------------------- save states ---- */
#define STATE_MAGIC 0x38364B58u /* "XK68" */
static size_t state_size_cache;
EXPORT size_t retro_serialize_size(void) {
    Ser s = {0, 0, (size_t)-1, 1, 0};
    uint32_t hdr[2] = {STATE_MAGIC, 1};
    SER(&s, hdr); machine_state(&s);
    state_size_cache = s.pos + 64;
    return state_size_cache;
}
EXPORT bool retro_serialize(void *data, size_t size) {
    if (size < 16) return false;
    Ser s = {(uint8_t *)data, 0, size, 1, 0};
    uint32_t hdr[2] = {STATE_MAGIC, 1};
    SER(&s, hdr); machine_state(&s);
    return !s.err;
}
EXPORT bool retro_unserialize(const void *data, size_t size) {
    Ser s = {(uint8_t *)data, 0, size, 0, 0};
    uint32_t hdr[2] = {0, 0};
    SER(&s, hdr);
    if (s.err || hdr[0] != STATE_MAGIC || hdr[1] != 1) return false;
    machine_state(&s);
    if (s.err) return false;
    irq_update();
    video_changed = 1;
    return true;
}
EXPORT void retro_cheat_reset(void) {}
EXPORT void retro_cheat_set(unsigned i, bool e, const char *c) { (void)i; (void)e; (void)c; }
EXPORT void *retro_get_memory_data(unsigned id) { return id == RETRO_MEMORY_SAVE_RAM ? (void *)sram : id == RETRO_MEMORY_SYSTEM_RAM ? (void *)ram : 0; }
EXPORT size_t retro_get_memory_size(unsigned id) { return id == RETRO_MEMORY_SAVE_RAM ? sizeof sram : id == RETRO_MEMORY_SYSTEM_RAM ? ram_size : 0; }

/* ----------------------------------------------------------- log shim ---- */
EXPORT void retro_log_shim(enum retro_log_level level, const char *fmt, ...) { log_msg((int)level, fmt); }
EXPORT void *get_retro_log_shim(void) { return (void *)retro_log_shim; }

/* -------------------------------------------------- webx68k_* exports ---- */
EXPORT uint8_t *webx68k_tvram_data(void) { return tvram; }
EXPORT int webx68k_text_dot_x(void) { return video_w; }
EXPORT int webx68k_text_dot_y(void) { return video_h; }
EXPORT int webx68k_text_scroll_x(void) { return (int)text_scroll_x; }
EXPORT int webx68k_text_scroll_y(void) { return (int)text_scroll_y; }
EXPORT void webx68k_send_key_make(int sc) { key_event(sc & 127, 1); }
EXPORT int webx68k_peek8(uint32_t a) { a &= 0xFFFFFF; if (a < ram_size) return ram[a ^ 1]; int v = dma_read8(a); return v < 0 ? 0xFF : v; }
EXPORT int webx68k_peek16(uint32_t a) { return (webx68k_peek8(a) << 8) | webx68k_peek8(a + 1); }
EXPORT int webx68k_sram_read(int off) { return sram[off & 0x3FFF]; }
EXPORT void webx68k_sram_write(int off, int v) { sram[off & 0x3FFF] = (uint8_t)v; }
EXPORT int webx68k_mem_read(uint32_t a, uint8_t *dst, int n) { for (int i = 0; i < n; i++) dst[i] = (uint8_t)webx68k_peek8(a + i); return n; }
EXPORT int webx68k_mem_write(uint32_t a, const uint8_t *src, int n) {
    for (int i = 0; i < n; i++) { uint32_t p = (a + i) & 0xFFFFFF; if (p < ram_size) ram[p ^ 1] = src[i]; else dma_write8(p, src[i]); }
    return n;
}
EXPORT int webx68k_configured_ram_size(void) { return (int)ram_size; }
EXPORT int webx68k_keybuf_peek(uint32_t a) { return webx68k_peek8(a); }
EXPORT int webx68k_keybuf_write_pointer(void) { return webx68k_peek16(0x0812); }  /* IOCS key buffer work area (best effort) */
EXPORT int webx68k_joystick_read(int port) { return pia_read(port ? 0xE9A003 : 0xE9A001); }
EXPORT int webx68k_fdd_insert(int drive, const char *path) { return fdd_insert(drive, path); }
EXPORT void webx68k_fdd_eject(int drive) { fdd_eject(drive); }
EXPORT int webx68k_serial_rx(const uint8_t *p, int n) { return serial_rx(p, n); }
EXPORT int webx68k_serial_tx_available(void) { return serial_tx_available(); }
EXPORT int webx68k_serial_tx_drain(uint8_t *p, int n) { return serial_tx_drain(p, n); }
EXPORT void webx68k_serial_reset(void) { serial_reset(); }
EXPORT void webx68k_serial_set_connected(int c) { serial_set_connected(c); }
EXPORT void webx68k_serial_set_tx_writable(int c) { serial_set_tx_writable(c); }
EXPORT int webx68k_serial_guest_baud_rate(void) { return serial_baud(); }
/* host file system bridge (human68k driver hook of the original core): not implemented */
EXPORT int webx68k_hostfs_poll(void) { return 0; }
EXPORT int webx68k_hostfs_request(void) { return 0; }
EXPORT void webx68k_hostfs_complete(void) {}

/* -------------------------------------------------- get_* diagnostics ---- */
EXPORT int get_fdd_is_reading(void) { return fdc_is_reading; }
EXPORT int get_fdd_access_drive(void) { return fdc_access_drive; }
EXPORT int get_sasi_is_accessing(void) { return sasi_accessing; }
EXPORT int get_fdd_dirty_mask(void) { return fdd_dirty_mask; }
EXPORT int get_sasi_dirty(void) { return sasi_dirty_flag; }
EXPORT void clear_fdd_dirty(int d) { fdd_dirty_mask &= ~(1 << (d & 3)); }
EXPORT void clear_sasi_dirty(void) { sasi_dirty_flag = 0; }
EXPORT int get_sasi_read_count(void) { return (int)sasi_stat[0]; }
EXPORT int get_sasi_write_count(void) { return (int)sasi_stat[1]; }
EXPORT int get_sasi_last_read_lba(void) { return (int)sasi_stat[2]; }
EXPORT int get_sasi_last_write_lba(void) { return (int)sasi_stat[3]; }
EXPORT int get_sasi_req_total(void) { return (int)sasi_stat[4]; }
EXPORT int get_mouse_dx(void) { return mouse_last_dx; }
EXPORT int get_mouse_dy(void) { return mouse_last_dy; }
EXPORT int get_mouse_stat(void) { return mouse_last_stat; }
/* SCSI diagnostics of the original core (this core has no SCSI board) */
EXPORT int get_scsi_req_total(void) { return 0; }
EXPORT int get_scsi_unsupported_count(void) { return 0; }
EXPORT int get_scsi_read_count(void) { return 0; }
EXPORT int get_scsi_write_count(void) { return 0; }
EXPORT int get_scsi_last_read_unit(void) { return 0; }
EXPORT int get_scsi_last_read_logsec(void) { return 0; }
EXPORT int get_scsi_last_write_unit(void) { return 0; }
EXPORT int get_scsi_last_write_logsec(void) { return 0; }
EXPORT int get_scsi_strategy_call_count(void) { return 0; }
EXPORT int get_scsi_interrupt_call_count(void) { return 0; }
EXPORT int get_scsi_boot_control_transferred(void) { return 0; }
