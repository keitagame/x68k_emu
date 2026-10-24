/* sasi.c - SASI hard disk interface (one image, 256-byte sectors, write-through) */
#include "x68k.h"

extern uint8_t ioc_stat;
int sasi_accessing, sasi_dirty_flag;
uint32_t sasi_stat[8];   /* [0]=read count [1]=write count [2]=last read lba [3]=last write lba [4]=req total */
static int hd_fd = -1;
static long hd_size;
static struct {
    int phase, rw, bufptr, senseptr;
    uint32_t sector, blocks;
    int device, unit, cmdptr;
    uint8_t stat, error, cmd[6], buf[256], sense[4];
} S;

int sasi_load(const char *path) {
    if (hd_fd >= 0) { hfile_close(hd_fd); hd_fd = -1; }
    int fd = hfile_open(path, 1);
    if (fd < 0) fd = hfile_open(path, 0);
    if (fd < 0) return -1;
    hd_fd = fd; hd_size = hfile_size(fd);
    return 0;
}
void sasi_reset(void) { memset(&S, 0, sizeof S); }
int sasi_data_ready(void) { return S.phase == 2 || S.phase == 3 || S.phase == 9; }
static int present(void) { return hd_fd >= 0 && S.device == 0; }
static int load_sector(void) {
    memset(S.buf, 0, 256);
    if (hd_fd < 0) return -1;
    if (hfile_seek(hd_fd, (long)S.sector << 8, 0) != ((long)S.sector << 8)) return 0;
    if (hfile_read(hd_fd, S.buf, 256) != 256) return 0;
    sasi_stat[0]++; sasi_stat[2] = S.sector;
    return 1;
}
static int flush_sector(void) {
    if (hd_fd < 0) return -1;
    if (hfile_seek(hd_fd, (long)S.sector << 8, 0) != ((long)S.sector << 8)) return 0;
    if (hfile_write(hd_fd, S.buf, 256) != 256) return 0;
    sasi_stat[1]++; sasi_stat[3] = S.sector; sasi_dirty_flag = 1;
    return 1;
}
static void raise_hdd_int(void) { ioc_irq(IOC_HDD); }

uint8_t sasi_read(uint32_t a) {
    uint8_t ret = 0;
    if (a == 0xE96003) {
        if (S.phase) ret |= 2;
        if (S.phase > 1) ret |= 1;
        if (S.phase == 2) ret |= 8;
        if (S.phase == 3 && S.rw) ret |= 4;
        if (S.phase == 9) ret |= 4;
        if (S.phase == 4 || S.phase == 5) ret |= 0x0C;
        if (S.phase == 5) ret |= 0x10;
    } else if (a == 0xE96001) {
        if (S.phase == 3 && S.rw) {
            ret = S.buf[S.bufptr++];
            if (S.bufptr == 256) {
                S.blocks--;
                if (S.blocks) {
                    S.sector++; S.bufptr = 0;
                    if (!load_sector()) { S.error = 0x0F; S.phase++; }
                } else S.phase++;
            }
        } else if (S.phase == 4) {
            ret = S.error ? 0x02 : S.stat; S.phase++;
        } else if (S.phase == 5) S.phase = 0;
        else if (S.phase == 9) {
            ret = S.sense[S.senseptr++];
            if (S.senseptr == 4) { S.error = 0; S.phase = 4; }
        }
        if (S.phase == 4) raise_hdd_int();
    }
    sasi_accessing = S.phase ? 2 : 0;
    return ret;
}
static void check_cmd(void) {
    S.unit = (S.cmd[1] >> 5) & 1;
    int ok = present() && S.unit == 0;
    switch (S.cmd[0]) {
    case 0x00: case 0x0B:
        if (ok) S.stat = 0; else { S.stat = 2; S.error = 0x7F; }
        S.phase += 2; break;
    case 0x01:
        if (ok) { S.sector = 0; S.stat = 0; } else { S.stat = 2; S.error = 0x7F; }
        S.phase += 2; break;
    case 0x03:
        S.sense[0] = S.error; S.sense[1] = (uint8_t)((S.unit << 5) | ((S.sector >> 16) & 0x1F));
        S.sense[2] = (uint8_t)(S.sector >> 8); S.sense[3] = (uint8_t)S.sector;
        S.error = 0; S.phase = 9; S.stat = 0; S.senseptr = 0; break;
    case 0x04: S.phase += 2; S.stat = 0; break;
    case 0x08: case 0x0A: {
        S.sector = ((uint32_t)(S.cmd[1] & 0x1F) << 16) | ((uint32_t)S.cmd[2] << 8) | S.cmd[3];
        S.blocks = S.cmd[4]; S.phase++; S.rw = S.cmd[0] == 0x08; S.bufptr = 0; S.stat = 0;
        sasi_stat[4] += S.blocks;
        int r = ok ? load_sector() : -1;
        if (!S.rw) memset(S.buf, 0, 256);
        if (r == 0 || r == -1) S.error = 0x0F;
        dmac_exec(1);
        break; }
    case 0xC2:
        S.phase = 10; S.senseptr = 0;
        if (ok) S.stat = 0; else { S.stat = 2; S.error = 0x7F; }
        break;
    default: S.phase += 2; break;
    }
}
void sasi_write(uint32_t a, uint8_t v) {
    if (a == 0xE96007 && S.phase == 0) {
        S.device = 0x7F;
        if (v) for (int i = 0; i < 8; i++) if (v & (1 << i)) { S.device = i; break; }
        if (S.device == 0 && hd_fd >= 0) { S.phase++; S.cmdptr = 0; } else S.phase = 0;
    } else if (a == 0xE96003 && S.phase == 1) S.phase++;
    else if (a == 0xE96005) { int keep = S.device; (void)keep; memset(&S, 0, sizeof S); }
    else if (a == 0xE96001) {
        if (S.phase == 2) { S.cmd[S.cmdptr++] = v; if (S.cmdptr == 6) check_cmd(); }
        else if (S.phase == 3 && !S.rw) {
            S.buf[S.bufptr++] = v;
            if (S.bufptr == 256) {
                flush_sector();
                S.blocks--;
                if (S.blocks) { S.sector++; S.bufptr = 0; if (!load_sector()) { S.error = 0x0F; S.phase++; } }
                else S.phase++;
            }
        } else if (S.phase == 10) {
            S.senseptr++;
            if (S.senseptr == 10) S.phase = 4;
        }
        if (S.phase == 4) raise_hdd_int();
    }
    sasi_accessing = S.phase ? 2 : 0;
}
void sasi_state(void *s) { SER(s, S); }
