/* host.c - minimal runtime.  On wasm32 it talks to the Emscripten-style JS glue
 * (the same imports the original px68k_libretro.wasm uses); on native builds it
 * uses libc so the machine can be unit tested. */
#include "x68k.h"

#ifdef __wasm__
/* ---- imports provided by px68k_libretro.js (Emscripten runtime) ---- */
#define IMP(m, n) __attribute__((import_module(m), import_name(n)))
IMP("env", "__syscall_openat") int  sys_openat(int dirfd, const char *path, int flags, int varargs);
IMP("env", "fd_read")  int  sys_fd_read(int fd, const void *iov, int iovcnt, int *pnum);
IMP("env", "fd_write") int  sys_fd_write(int fd, const void *iov, int iovcnt, int *pnum);
IMP("env", "fd_seek")  int  sys_fd_seek(int fd, int64_t off, int whence, int64_t *newoff);
IMP("env", "fd_close") int  sys_fd_close(int fd);
IMP("env", "emscripten_date_now") double sys_date_now(void);
IMP("env", "_localtime_js") int sys_localtime(int64_t t, int *tm);
IMP("env", "emscripten_resize_heap") int sys_resize_heap(size_t sz);
IMP("env", "js_retro_log") void sys_log(int level, const char *msg);

void *memcpy(void *d, const void *s, size_t n) { uint8_t *a = d; const uint8_t *b = s; while (n--) *a++ = *b++; return d; }
void *memmove(void *d, const void *s, size_t n) {
    uint8_t *a = d; const uint8_t *b = s;
    if (a < b) while (n--) *a++ = *b++; else { a += n; b += n; while (n--) *--a = *--b; }
    return d;
}
void *memset(void *d, int c, size_t n) { uint8_t *a = d; while (n--) *a++ = (uint8_t)c; return d; }
int memcmp(const void *x, const void *y, size_t n) {
    const uint8_t *a = x, *b = y;
    for (; n; n--, a++, b++) if (*a != *b) return *a - *b;
    return 0;
}
size_t strlen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
int strcmp(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return (uint8_t)*a - (uint8_t)*b; }

/* ---- tiny first-fit allocator over the wasm heap ---- */
extern uint8_t __heap_base;
typedef struct Blk { size_t size; int free; struct Blk *next; } Blk;
static Blk *heap_head;
static uint8_t *heap_end;
static void heap_init(void) {
    uint8_t *start = (uint8_t *)(((size_t)&__heap_base + 15) & ~15u);
    size_t total = (size_t)__builtin_wasm_memory_size(0) * 65536;
    heap_head = (Blk *)start;
    heap_head->size = total - (size_t)start - sizeof(Blk);
    heap_head->free = 1; heap_head->next = 0;
    heap_end = (uint8_t *)(size_t)total;
}
static int heap_grow(size_t need) {
    size_t cur = (size_t)__builtin_wasm_memory_size(0) * 65536;
    size_t want = cur + ((need + 65535) & ~65535u) + (1u << 20);
    if (!sys_resize_heap(want)) return 0;
    size_t now = (size_t)__builtin_wasm_memory_size(0) * 65536;
    Blk *b = heap_head; while (b->next) b = b->next;
    if (b->free) b->size += now - cur;
    else { Blk *n = (Blk *)heap_end; n->size = now - cur - sizeof(Blk); n->free = 1; n->next = 0; b->next = n; }
    heap_end = (uint8_t *)now;
    return 1;
}
void *malloc(size_t n) {
    if (!heap_head) heap_init();
    n = (n + 15) & ~15u;
    for (int pass = 0; pass < 2; pass++) {
        for (Blk *b = heap_head; b; b = b->next) {
            if (b->free && b->size >= n) {
                if (b->size >= n + sizeof(Blk) + 32) {
                    Blk *r = (Blk *)((uint8_t *)(b + 1) + n);
                    r->size = b->size - n - sizeof(Blk); r->free = 1; r->next = b->next;
                    b->size = n; b->next = r;
                }
                b->free = 0;
                return b + 1;
            }
        }
        if (pass == 0 && !heap_grow(n + sizeof(Blk))) break;
    }
    return 0;
}
void free(void *p) {
    if (!p) return;
    Blk *b = (Blk *)p - 1;
    b->free = 1;
    for (Blk *c = heap_head; c; c = c->next)
        while (c->free && c->next && c->next->free) { c->size += sizeof(Blk) + c->next->size; c->next = c->next->next; }
}
void log_msg(int level, const char *msg) { sys_log(level, msg); }
double host_now_ms(void) { return sys_date_now(); }
int host_localtime(double ms, int *tm9) {
    int tm[16];                      /* struct tm incl. gmtoff / zone pointer written by the glue */
    for (int i = 0; i < 16; i++) tm[i] = 0;
    sys_localtime((int64_t)(ms / 1000.0), tm);
    for (int i = 0; i < 7; i++) tm9[i] = tm[i];
    return 0;
}
int hfile_open(const char *path, int rw) { return sys_openat(-100, path, rw ? 2 : 0, 0); }
int hfile_read(int fd, void *buf, int n) {
    struct { const void *p; int l; } iov = { buf, n };
    int got = 0;
    if (sys_fd_read(fd, &iov, 1, &got)) return -1;
    return got;
}
int hfile_write(int fd, const void *buf, int n) {
    struct { const void *p; int l; } iov = { buf, n };
    int got = 0;
    if (sys_fd_write(fd, &iov, 1, &got)) return -1;
    return got;
}
long hfile_seek(int fd, long off, int whence) {
    int64_t np = 0;
    if (sys_fd_seek(fd, off, whence, &np)) return -1;
    return (long)np;
}
void hfile_close(int fd) { sys_fd_close(fd); }
#else
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
void log_msg(int level, const char *msg) { (void)level; fprintf(stderr, "[x68core] %s\n", msg); }
double host_now_ms(void) { struct timeval tv; gettimeofday(&tv, 0); return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0; }
int host_localtime(double ms, int *tm9) {
    time_t t = (time_t)(ms / 1000.0); struct tm *tm = localtime(&t);
    tm9[0] = tm->tm_sec; tm9[1] = tm->tm_min; tm9[2] = tm->tm_hour; tm9[3] = tm->tm_mday;
    tm9[4] = tm->tm_mon; tm9[5] = tm->tm_year; tm9[6] = tm->tm_wday; return 0;
}
int hfile_open(const char *path, int rw) { return open(path, rw ? O_RDWR : O_RDONLY); }
int hfile_read(int fd, void *buf, int n) { return (int)read(fd, buf, n); }
int hfile_write(int fd, const void *buf, int n) { return (int)write(fd, buf, n); }
long hfile_seek(int fd, long off, int whence) { return (long)lseek(fd, off, whence); }
void hfile_close(int fd) { close(fd); }
#endif

long hfile_size(int fd) {
    long cur = hfile_seek(fd, 0, 1);
    long end = hfile_seek(fd, 0, 2);
    hfile_seek(fd, cur < 0 ? 0 : cur, 0);
    return end;
}
void *hfile_load(const char *path, long *size) {
    int fd = hfile_open(path, 0);
    if (fd < 0) return 0;
    long n = hfile_size(fd);
    if (n <= 0) { hfile_close(fd); return 0; }
    uint8_t *buf = malloc((size_t)n);
    if (!buf) { hfile_close(fd); return 0; }
    long got = 0;
    while (got < n) { int r = hfile_read(fd, buf + got, (int)(n - got > 65536 ? 65536 : n - got)); if (r <= 0) break; got += r; }
    hfile_close(fd);
    if (size) *size = got;
    return buf;
}

/* serialization */
void ser_blob(Ser *s, void *data, size_t n) {
    if (s->err) return;
    if (!s->p) { s->pos += n; return; }            /* size-counting pass */
    if (s->pos + n > s->cap) { s->err = 1; return; }
    if (s->writing) memcpy(s->p + s->pos, data, n); else memcpy(data, s->p + s->pos, n);
    s->pos += n;
}
