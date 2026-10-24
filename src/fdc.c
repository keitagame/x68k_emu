/* fdc.c - uPD72065 floppy controller + FDD front end (XDF / DIM / D88 images, write-through) */
#include "x68k.h"

extern uint8_t ioc_stat;
typedef struct { uint8_t c, h, r, n; } Id;

typedef struct {
    int fd, type;           /* type: 0 none, 1 fixed 77x2x8x1024 (XDF/DIM), 2 D88 */
    long base;              /* header size for fixed types */
    int ro, ready;
    uint32_t trk_off[164];
    int rid_pos;
    char path[160];
} Drive;
static Drive drv[4];
int fdd_dirty_mask, fdc_is_reading, fdc_access_drive = -1;

/* ------------------------------------------------------------ disks ---- */
static uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static int ends_with(const char *s, const char *ext) {
    int a = (int)strlen(s), b = (int)strlen(ext);
    if (a < b) return 0;
    for (int i = 0; i < b; i++) { char x = s[a - b + i]; if (x >= 'A' && x <= 'Z') x += 32; if (x != ext[i]) return 0; }
    return 1;
}
int fdd_insert(int d, const char *path) {
    if (d < 0 || d > 3) return -1;
    fdd_eject(d);
    int rw = 1;
    int fd = hfile_open(path, 1);
    if (fd < 0) { fd = hfile_open(path, 0); rw = 0; }
    if (fd < 0) return -1;
    Drive *x = &drv[d];
    long size = hfile_size(fd);
    memset(x, 0, sizeof *x);
    x->fd = fd; x->ro = !rw;
    int n = (int)strlen(path); if (n > 159) n = 159;
    memcpy(x->path, path, n); x->path[n] = 0;
    uint8_t hdr[0x2B0];
    hfile_seek(fd, 0, 0);
    int got = hfile_read(fd, hdr, sizeof hdr);
    if (ends_with(path, ".d88") || ends_with(path, ".88d") || ends_with(path, ".d77") || ends_with(path, ".dat")) {
        if (got < 0x2B0) { hfile_close(fd); x->fd = -1; return -1; }
        x->type = 2;
        for (int i = 0; i < 164; i++) x->trk_off[i] = le32(hdr + 0x20 + i * 4);
    } else if (ends_with(path, ".dim")) {
        x->type = 1; x->base = 256;
    } else {
        x->type = 1; x->base = 0;
        if (size < 1261568 && size > 0 && got >= 256 && ends_with(path, ".hdm") == 0 && ends_with(path, ".xdf") == 0 && ends_with(path, ".2hd") == 0 && ends_with(path, ".img") == 0) {
            /* unknown extension: treat as raw 2HD anyway */
        }
    }
    x->ready = 1;
    if (ioc_stat & 2) ioc_irq(IOC_FDD);
    return 0;
}
void fdd_eject(int d) {
    if (d < 0 || d > 3) return;
    if (drv[d].fd > 0 || drv[d].type) { if (drv[d].fd >= 0) hfile_close(drv[d].fd); }
    memset(&drv[d], 0, sizeof drv[d]);
    drv[d].fd = -1;
    if (ioc_stat & 2) ioc_irq(IOC_FDD);
}
int fdd_ready(int d) { return d >= 0 && d < 4 && drv[d].type && drv[d].ready; }

/* locate sector; returns file offset of the data and size via *len, or -1 */
static long find_sector(int d, int trk, Id *id, int *len, uint8_t *flags_out, int match) {
    Drive *x = &drv[d];
    if (trk < 0 || trk > 163) return -1;
    if (x->type == 1) {
        if (trk >= 154) return -1;
        int r = id->r;
        if (match && (id->c != (trk >> 1) || id->h != (trk & 1) || r < 1 || r > 8 || id->n != 3)) return -1;
        *len = 1024; if (flags_out) *flags_out = 0;
        return x->base + ((long)trk * 8 + (r - 1)) * 1024;
    }
    uint32_t off = x->trk_off[trk];
    if (!off) return -1;
    uint8_t h[16];
    int total = -1;
    for (int i = 0; i < 64; i++) {
        hfile_seek(x->fd, off, 0);
        if (hfile_read(x->fd, h, 16) != 16) return -1;
        if (total < 0) total = h[4] | (h[5] << 8);
        int sz = h[14] | (h[15] << 8);
        if (!match || (h[0] == id->c && h[1] == id->h && h[2] == id->r && h[3] == id->n)) {
            id->c = h[0]; id->h = h[1]; id->r = h[2]; id->n = h[3];
            *len = sz; if (flags_out) *flags_out = h[7];
            return (long)off + 16;
        }
        off += 16 + sz;
        if (i + 1 >= total) break;
    }
    return -1;
}
static int fdd_read_sector(int d, int trk, Id *id, uint8_t *buf, int want, int *deleted) {
    int len = 0; uint8_t fl = 0;
    long off = find_sector(d, trk, id, &len, &fl, 1);
    if (off < 0) return 0;
    hfile_seek(drv[d].fd, off, 0);
    int n = want < len ? want : len;
    int got = hfile_read(drv[d].fd, buf, n);
    if (got < n) memset(buf + (got > 0 ? got : 0), 0, n - (got > 0 ? got : 0));
    if (want > len) memset(buf + len, 0, want - len);
    *deleted = fl == 0x10;
    return 1;
}
static int fdd_write_sector(int d, int trk, Id *id, const uint8_t *buf, int want) {
    int len = 0;
    long off = find_sector(d, trk, id, &len, 0, 1);
    if (off < 0) return 0;
    hfile_seek(drv[d].fd, off, 0);
    int n = want < len ? want : len;
    if (hfile_write(drv[d].fd, buf, n) < 0) return 0;
    fdd_dirty_mask |= 1 << d;
    return 1;
}
static int fdd_read_id(int d, int trk, Id *id) {
    Drive *x = &drv[d];
    if (x->type == 1) {
        if (trk >= 154) return 0;
        x->rid_pos = (x->rid_pos % 8) + 1;
        id->c = (uint8_t)(trk >> 1); id->h = (uint8_t)(trk & 1); id->r = (uint8_t)x->rid_pos; id->n = 3;
        return 1;
    }
    uint32_t off = x->trk_off[trk];
    if (!off) return 0;
    uint8_t h[16];
    hfile_seek(x->fd, off, 0);
    if (hfile_read(x->fd, h, 16) != 16) return 0;
    int total = h[4] | (h[5] << 8); if (!total) return 0;
    int want = x->rid_pos % total; x->rid_pos = want + 1;
    for (int i = 0; i < want; i++) { off += 16 + (h[14] | (h[15] << 8)); hfile_seek(x->fd, off, 0); hfile_read(x->fd, h, 16); }
    id->c = h[0]; id->h = h[1]; id->r = h[2]; id->n = h[3];
    return 1;
}
static int fdd_format_track(int d, int trk, const uint8_t *ids, int sc, uint8_t fill) {
    Drive *x = &drv[d];
    if (x->type != 1 || trk >= 154) return x->type == 2 ? 1 : 0;
    uint8_t buf[1024]; memset(buf, fill, sizeof buf);
    for (int i = 0; i < sc && i < 8; i++) {
        hfile_seek(x->fd, x->base + ((long)trk * 8 + (ids[i * 4 + 2] - 1 & 7)) * 1024, 0);
        hfile_write(x->fd, buf, 1024);
    }
    fdd_dirty_mask |= 1 << d;
    return 1;
}

/* -------------------------------------------------------------- FDC ---- */
static const uint8_t CMD_TABLE[32] = {0, 0, 8, 2, 1, 8, 8, 1, 0, 8, 1, 0, 8, 5, 0, 2, 0, 8, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 8, 0, 0};
static const uint8_t DAT_TABLE[32] = {1, 1, 7, 0, 1, 7, 7, 0, 2, 7, 7, 0, 7, 7, 1, 0, 1, 7, 1, 1, 1, 1, 1, 1, 1, 7, 1, 1, 1, 7, 1, 1};
static struct {
    int cmd, cyl, drv, ready, ctrl, wexec, rdptr, wrptr, rdnum, wrnum, bufnum, st0, st1, st2;
    uint8_t rsp[10], prm[10];
    uint8_t data[0x8000], scan[0x8000];
    Id cur;
} F;

static void set_int(void) { ioc_irq(IOC_FDC); }
static int sel(void) { return F.drv < 0 ? 0 : F.drv; }
static void ephase_end(void) {
    F.rsp[0] = (uint8_t)F.st0; F.rsp[1] = (uint8_t)F.st1; F.rsp[2] = (uint8_t)F.st2;
    F.rsp[3] = F.cur.c; F.rsp[4] = F.cur.h; F.rsp[5] = F.cur.r; F.rsp[6] = F.cur.n;
    F.bufnum = 0; F.rdptr = 0; F.wexec = 0; fdc_is_reading = 0;
    set_int();
}
int fdc_data_ready(void) { return F.bufnum ? 1 : 0; }
void fdc_reset(void) {
    memset(&F, 0, sizeof F); F.drv = -1;
    for (int i = 0; i < 4; i++) drv[i].rid_pos = 0;
}
static int track_of(void) { return (F.cyl << 1) + ((F.prm[1] >> 2) & 1); }

static void exec_cmd(void) {
    int us = F.prm[1] & 7, hd = (F.prm[1] >> 2) & 1, d = sel();
    F.cur.c = F.prm[2]; F.cur.h = F.prm[3]; F.cur.r = F.prm[4]; F.cur.n = F.prm[5];
    int dtl = F.prm[8], n = F.prm[5];
    int ready = fdd_ready(d) || F.ready;
    switch (F.cmd) {
    case 3: break;
    case 4:
        F.st0 = us; F.rsp[0] = (uint8_t)F.st0;
        if (!F.cyl) F.rsp[0] |= 0x10;
        if (ready) F.rsp[0] |= 0x20;
        if (drv[d].ro) F.rsp[0] |= 0x40;
        break;
    case 5: case 9:
        F.st0 = us;
        if (drv[d].ro) { F.st0 |= 0x40; F.st1 |= 2; }
        else if (ready) {
            if (!fdd_ready(d) || track_of() > 163) { F.st0 |= 0x40; F.st1 |= 4; }
            else { F.bufnum = n ? (128 << n) : dtl; F.wexec = 1; fdc_access_drive = d; fdc_is_reading = 0; }
        } else F.st0 |= 0x48;
        if (F.st0 & 0x40) ephase_end();
        break;
    case 6: case 12: case 17: case 25: case 29: {
        F.st0 = us;
        if (!ready || !fdd_ready(d)) { F.st0 |= 0x48; ephase_end(); break; }
        Id id = F.cur; int del = 0; int want = n ? (128 << n) : dtl;
        if (want > 0x8000) want = 0x8000;
        uint8_t *dst = (F.cmd == 6 || F.cmd == 12) ? F.data : F.scan;
        if (!fdd_read_sector(d, track_of(), &id, dst, want, &del)) { F.st0 |= 0x40; F.st1 |= 4; ephase_end(); break; }
        F.bufnum = want;
        if (F.cmd != 6 && F.cmd != 12) F.wexec = 1;
        if ((del && F.cmd != 12) || (!del && F.cmd == 12)) F.st2 |= 0x40;
        fdc_access_drive = d; fdc_is_reading = 1;
        break; }
    case 2: F.st0 = us | 0x48; ephase_end(); break;
    case 7:
        F.st0 = us; F.cyl = 0;
        if (!ready && !fdd_ready(d)) F.st0 |= 0x48; else if (F.drv >= 0 && F.drv < 2) F.st0 |= 0x20; else F.st0 |= 0x50;
        set_int();
        break;
    case 8:
        F.rsp[0] = (uint8_t)F.st0; F.rsp[1] = (uint8_t)F.cyl; F.st0 = 0x80;
        break;
    case 10: {
        Id id = {0, 0, 0, 0}; F.st0 = us;
        if (ready && fdd_ready(d)) {
            if (!fdd_read_id(d, track_of(), &id)) { F.st0 |= 0x40; F.st1 |= 4; }
        } else F.st0 |= 0x48;
        F.cur = id;
        F.rsp[0] = (uint8_t)F.st0; F.rsp[1] = (uint8_t)F.st1; F.rsp[2] = (uint8_t)F.st2;
        F.rsp[3] = id.c; F.rsp[4] = id.h; F.rsp[5] = id.r; F.rsp[6] = id.n;
        set_int();
        break; }
    case 13:
        F.st0 = us;
        if (drv[d].ro) { F.st0 |= 0x40; F.st1 |= 2; }
        else if (ready) { F.bufnum = F.prm[3] << 2; F.wexec = 1; }
        else F.st0 |= 0x48;
        if (F.st0 & 0x40) ephase_end();
        break;
    case 15:
        F.st0 = us & 7;
        if (fdd_ready(d) || F.ready) { F.cyl = F.prm[2]; F.st0 |= 0x20; }
        else F.st0 |= 0x48;
        (void)hd;
        set_int();
        break;
    default:
        F.rsp[0] = 0x80; F.st0 = 0x80;
        break;
    }
    if (F.bufnum) dmac_exec(0);   /* an armed DMA channel starts moving data right away */
}
static void next_track(void) {
    /* after a sector has been transferred */
    if (F.prm[4] == F.prm[6]) {
        if (((F.prm[0] >> 7) & 1) && !((F.prm[1] >> 2) & 1) && !F.prm[3]) { F.prm[1] |= 4; F.prm[3] = 1; F.prm[4] = 1; }
        else { ephase_end(); return; }
    } else F.prm[4]++;
    exec_cmd();
}
static void write_buffer(void) {
    Id id = { F.prm[2], F.prm[3], F.prm[4], F.prm[5] };
    int d = sel();
    switch (F.cmd) {
    case 5: case 9:
        if (!fdd_write_sector(d, track_of(), &id, F.data, F.bufnum)) { F.st0 |= 0x40; F.st1 |= 4; set_int(); }
        else next_track();
        break;
    case 13:
        if (!fdd_format_track(d, track_of(), F.data, F.prm[3], F.prm[5])) { F.st0 |= 0x40; F.st1 |= 4; }
        ephase_end();
        break;
    case 17: case 25: case 29: {
        int bad = 0;
        for (int i = 0; i < F.bufnum && !bad; i++) {
            if (F.data[i] == 0xFF) continue;
            if (F.cmd == 17 && F.scan[i] != F.data[i]) bad = 1;
            else if (F.cmd == 25 && F.scan[i] > F.data[i]) bad = 1;
            else if (F.cmd == 29 && F.scan[i] < F.data[i]) bad = 1;
            if (F.cmd != 17 && F.scan[i] != F.data[i]) break;
        }
        if (bad) { F.st0 |= 0x40; F.st2 = (F.st2 & ~8) | 4; ephase_end(); }
        else next_track();
        break; }
    }
}
uint8_t fdc_read(uint32_t a) {
    uint8_t ret = 0;
    if (a == 0xE94001) {
        ret = 0x80;
        ret |= (F.rdnum && !F.wexec) ? 0x40 : 0;
        ret |= (F.wrnum || F.rdnum) ? 0x10 : 0;
        if (F.rdnum == 1 && F.cmd == 8) ret &= 0xAF;
    } else if (a == 0xE94003) {
        if (F.bufnum) {
            ret = F.data[F.rdptr++];
            if (F.rdptr >= F.bufnum) { F.rdptr = 0; next_track(); }
        } else if (F.rdnum) { ret = F.rsp[F.rdptr++]; F.rdnum--; }
    } else if (a == 0xE94005) {
        if ((F.ctrl & 1) && fdd_ready(0)) ret = 0x80;
        if ((F.ctrl & 2) && fdd_ready(1)) ret = 0x80;
    }
    return ret;
}
void fdc_write(uint32_t a, uint8_t v) {
    if (a == 0xE94003) {
        if (F.bufnum) {
            F.data[F.wrptr++] = v;
            if (F.wrptr >= F.bufnum) { write_buffer(); F.wrptr = 0; }
        } else {
            if (F.wrnum) { F.prm[F.wrptr++] = v; F.wrnum--; }
            else {
                F.cmd = v & 0x1F; F.rdptr = 0; F.wrptr = 0; F.rdnum = 0;
                F.wrnum = CMD_TABLE[F.cmd]; F.prm[F.wrptr++] = v;
            }
            if (!F.wrnum) {
                F.wrptr = 0; F.rdnum = DAT_TABLE[F.cmd];
                if (F.cmd == 3 || F.cmd == 7 || F.cmd == 15) F.rdnum = 0;
                F.st1 = 0; F.st2 = 0;
                if (F.cmd == 17 || F.cmd == 25 || F.cmd == 29) F.st2 |= 8;
                exec_cmd();
            }
        }
    } else if (a == 0xE94005) {
        for (int i = 0; i < 2; i++)
            if ((F.ctrl & (1 << i)) && !(v & (1 << i))) {
                if (F.ctrl & 0x20) fdd_eject(i);
            }
        F.ctrl = v;
    } else if (a == 0xE94007) {
        F.drv = (v & 0x80) ? (v & 3) : -1;
        fdc_access_drive = F.drv;
    }
}
void fdc_state(void *s) { SER(s, F); }
