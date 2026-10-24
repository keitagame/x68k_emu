/* cpu.c - MC68000 interpreter.
 * Original implementation for this project.  Bus errors / address errors are
 * recorded in cpu.fault by the bus and taken after the faulting instruction
 * completes (no setjmp/longjmp, so it builds for bare wasm32). */
#include "cpu.h"

CPU cpu;

void cpu_bus_error(uint32_t a, int rw);

#define D(n) cpu.d[n]
#define A(n) cpu.a[n]

static inline uint32_t MASKS(int sz) { return sz == 0 ? 0xFFu : sz == 1 ? 0xFFFFu : 0xFFFFFFFFu; }
static inline uint32_t MSBS(int sz)  { return sz == 0 ? 0x80u : sz == 1 ? 0x8000u : 0x80000000u; }
static inline int32_t sext(uint32_t v, int sz) {
    return sz == 0 ? (int32_t)(int8_t)v : sz == 1 ? (int32_t)(int16_t)v : (int32_t)v;
}

/* ------------------------------------------------------------ SR ------ */
uint16_t cpu_get_sr(void) {
    return (uint16_t)((cpu.t << 15) | (cpu.s << 13) | (cpu.imask << 8) |
                      (cpu.x << 4) | (cpu.n << 3) | (cpu.z << 2) | (cpu.v << 1) | cpu.c);
}
static void set_ccr(uint16_t v) {
    cpu.x = (v >> 4) & 1; cpu.n = (v >> 3) & 1; cpu.z = (v >> 2) & 1;
    cpu.v = (v >> 1) & 1; cpu.c = v & 1;
}
void cpu_set_sr(uint16_t sr) {
    int ns = (sr >> 13) & 1;
    if (ns != cpu.s) {
        if (cpu.s) cpu.ssp = A(7); else cpu.usp = A(7);
        A(7) = ns ? cpu.ssp : cpu.usp;
    }
    cpu.s = ns; cpu.t = (sr >> 15) & 1; cpu.imask = (sr >> 8) & 7;
    set_ccr(sr);
}
static void enter_super(void) {
    if (!cpu.s) { cpu.usp = A(7); A(7) = cpu.ssp; cpu.s = 1; }
}

/* ------------------------------------------------------- memory ------- */
void cpu_bus_error(uint32_t a, int rw) {
    if (!cpu.fault) { cpu.fault = 1; cpu.fault_addr = a; cpu.fault_rw = (uint8_t)rw; }
}
static void addr_error(uint32_t a, int rw) {
    if (!cpu.fault) { cpu.fault = 2; cpu.fault_addr = a; cpu.fault_rw = (uint8_t)rw; }
}
static inline void setfc(int prog) {
    cpu.fault_fc = cpu.s ? (prog ? CPU_FC_SP : CPU_FC_SD) : (prog ? CPU_FC_UP : CPU_FC_UD);
}
static uint32_t mem_r(uint32_t a, int sz) {
    setfc(0);
    if (sz == 0) return bus_rd8(a);
    if (a & 1) { addr_error(a, 1); return 0; }
    if (sz == 1) return bus_rd16(a);
    return ((uint32_t)bus_rd16(a) << 16) | bus_rd16(a + 2);
}
static void mem_w(uint32_t a, int sz, uint32_t v) {
    if (cpu.fault) return; /* instruction already aborted by a prior fault */
    setfc(0);
    if (sz == 0) { bus_wr8(a, (uint8_t)v); return; }
    if (a & 1) { addr_error(a, 0); return; }
    if (sz == 1) { bus_wr16(a, (uint16_t)v); return; }
    bus_wr16(a, (uint16_t)(v >> 16));
    bus_wr16(a + 2, (uint16_t)v);
}
static inline uint32_t r16(uint32_t a) { return mem_r(a, 1); }
static inline uint32_t r32(uint32_t a) { return mem_r(a, 2); }
static inline void w16(uint32_t a, uint32_t v) { mem_w(a, 1, v); }
static inline void w32(uint32_t a, uint32_t v) { mem_w(a, 2, v); }

static uint16_t fetch16(void) {
    uint32_t a = cpu.pc;
    setfc(1);
    cpu.pc += 2;
    if (a & 1) { addr_error(a, 1); return 0; }
    return bus_rd16(a);
}
static inline uint32_t fetch32(void) { uint32_t h = fetch16(); return (h << 16) | fetch16(); }
static inline void push16(uint16_t v) { A(7) -= 2; w16(A(7), v); }
static inline void push32(uint32_t v) { A(7) -= 4; w32(A(7), v); }
static inline uint32_t pop16(void) { uint32_t v = r16(A(7)); A(7) += 2; return v; }
static inline uint32_t pop32(void) { uint32_t v = r32(A(7)); A(7) += 4; return v; }

/* ---------------------------------------------------- exceptions ------ */
static void exception(int vec, int cyc) {
    if (cpu.fault) return; /* a pending bus/address error supersedes */
    uint16_t old = cpu_get_sr();
    enter_super();
    cpu.t = 0;
    A(7) -= 6;
    w32(A(7) + 2, cpu.pc);
    w16(A(7), old);
    cpu.pc = r32((uint32_t)vec * 4);
    cpu.cycles += cyc;
    if (cpu.fault) { cpu.fault = 0; cpu.halted = 1; }
}
static void take_fault(void) {
    int kind = cpu.fault;
    uint16_t old = cpu_get_sr();
    uint16_t ssw = (uint16_t)((cpu.ir & 0xFFE0) | (cpu.fault_rw ? 0x10 : 0) | (cpu.fault_fc & 7));
    uint32_t fa = cpu.fault_addr;
    cpu.fault = 0;
    enter_super();
    cpu.t = 0;
    A(7) -= 14;
    w16(A(7), ssw);
    w32(A(7) + 2, fa);
    w16(A(7) + 6, cpu.ir);
    w16(A(7) + 8, old);
    w32(A(7) + 10, cpu.pc);
    if (cpu.fault) { cpu.fault = 0; cpu.halted = 1; return; }
    cpu.pc = r32(kind == 1 ? 8u : 12u);
#ifdef DBG
    { extern int printf(const char*,...); printf("fault kind=%d fa=%06X pc=%06X sp=%06X v=%04X %04X\n", kind, fa, cpu.pc, A(7), bus_rd16(12), bus_rd16(14)); }
#endif
    cpu.cycles += 50;
    if (cpu.fault) { cpu.fault = 0; cpu.halted = 1; }
}

/* ------------------------------------------------------ EA ------------ */
typedef struct { int kind; uint32_t addr; int reg; uint32_t imm; } Loc; /* 0 Dn 1 An 2 mem 3 imm */

static uint32_t brief_ext(uint32_t base) {
    uint16_t e = fetch16();
    int r = (e >> 12) & 7;
    uint32_t idx = (e & 0x8000) ? A(r) : D(r);
    if (!(e & 0x0800)) idx = (uint32_t)(int32_t)(int16_t)idx;
    return base + (uint32_t)(int32_t)(int8_t)e + idx;
}
static void ea_loc(Loc *l, int mode, int reg, int sz) {
    switch (mode) {
    case 0: l->kind = 0; l->reg = reg; return;
    case 1: l->kind = 1; l->reg = reg; return;
    case 2: l->kind = 2; l->addr = A(reg); return;
    case 3: {
        int inc = sz == 0 ? (reg == 7 ? 2 : 1) : sz == 1 ? 2 : 4;
        l->kind = 2; l->addr = A(reg); A(reg) += inc; return; }
    case 4: {
        int dec = sz == 0 ? (reg == 7 ? 2 : 1) : sz == 1 ? 2 : 4;
        A(reg) -= dec; l->kind = 2; l->addr = A(reg); return; }
    case 5: l->kind = 2; l->addr = A(reg) + (uint32_t)(int32_t)(int16_t)fetch16(); return;
    case 6: l->kind = 2; l->addr = brief_ext(A(reg)); return;
    default:
        switch (reg) {
        case 0: l->kind = 2; l->addr = (uint32_t)(int32_t)(int16_t)fetch16(); return;
        case 1: l->kind = 2; l->addr = fetch32(); return;
        case 2: { uint32_t b = cpu.pc; l->kind = 2; l->addr = b + (uint32_t)(int32_t)(int16_t)fetch16(); return; }
        case 3: { uint32_t b = cpu.pc; l->kind = 2; l->addr = brief_ext(b); return; }
        case 4:
            l->kind = 3;
            if (sz == 2) l->imm = fetch32();
            else { l->imm = fetch16(); if (sz == 0) l->imm &= 0xFF; }
            return;
        default: l->kind = 3; l->imm = 0; cpu.fault = cpu.fault ? cpu.fault : 0; return;
        }
    }
}
static uint32_t loc_read(Loc *l, int sz) {
    switch (l->kind) {
    case 0: return D(l->reg) & MASKS(sz);
    case 1: return A(l->reg) & MASKS(sz);
    case 2: return mem_r(l->addr, sz);
    default: return l->imm;
    }
}
static void loc_write(Loc *l, int sz, uint32_t v) {
    switch (l->kind) {
    case 0: {
        uint32_t m = MASKS(sz);
        D(l->reg) = (D(l->reg) & ~m) | (v & m);
        return; }
    case 1: A(l->reg) = v; return;
    case 2: mem_w(l->addr, sz, v); return;
    default: return;
    }
}
/* EA timing (word/byte), long adds 4 for memory modes */
static int ea_cyc(int mode, int reg, int sz) {
    static const uint8_t t[12] = {0, 0, 4, 4, 6, 8, 10, 8, 12, 8, 10, 4};
    int i = mode < 7 ? mode : 7 + reg;
    if (i > 11) i = 11;
    return t[i] + ((sz == 2 && i >= 2) ? 4 : 0);
}
static int is_ctrl_mode(int mode, int reg) {
    return mode == 2 || mode == 5 || mode == 6 || (mode == 7 && reg <= 3);
}
static int is_data_alterable(int mode, int reg) { return mode != 1 && (mode < 7 || reg <= 1); }
static int is_mem_alterable(int mode, int reg) { return mode >= 2 && (mode < 7 || reg <= 1); }

/* ------------------------------------------------------- ALU ---------- */
static uint32_t alu_add(uint32_t s, uint32_t d, int sz, uint32_t cin, int ext) {
    uint32_t m = MASKS(sz), msb = MSBS(sz);
    uint64_t r = (uint64_t)(s & m) + (d & m) + cin;
    uint32_t res = (uint32_t)r & m;
    cpu.c = cpu.x = (uint8_t)((r >> (sz == 0 ? 8 : sz == 1 ? 16 : 32)) & 1);
    cpu.v = (((s ^ res) & (d ^ res)) & msb) != 0;
    cpu.n = (res & msb) != 0;
    if (ext) { if (res) cpu.z = 0; } else cpu.z = (res == 0);
    return res;
}
static uint32_t alu_sub(uint32_t s, uint32_t d, int sz, uint32_t cin, int ext, int setx) {
    uint32_t m = MASKS(sz), msb = MSBS(sz);
    s &= m; d &= m;
    uint32_t res = (d - s - cin) & m;
    uint8_t c = ((uint64_t)s + cin) > d;
    cpu.c = c; if (setx) cpu.x = c;
    cpu.v = (((s ^ d) & (res ^ d)) & msb) != 0;
    cpu.n = (res & msb) != 0;
    if (ext) { if (res) cpu.z = 0; } else cpu.z = (res == 0);
    return res;
}
static inline void logic_flags(uint32_t res, int sz) {
    res &= MASKS(sz);
    cpu.n = (res & MSBS(sz)) != 0; cpu.z = res == 0; cpu.v = 0; cpu.c = 0;
}
static int cond_true(int cc) {
    switch (cc) {
    case 0: return 1;
    case 1: return 0;
    case 2: return !cpu.c && !cpu.z;
    case 3: return cpu.c || cpu.z;
    case 4: return !cpu.c;
    case 5: return cpu.c;
    case 6: return !cpu.z;
    case 7: return cpu.z;
    case 8: return !cpu.v;
    case 9: return cpu.v;
    case 10: return !cpu.n;
    case 11: return cpu.n;
    case 12: return cpu.n == cpu.v;
    case 13: return cpu.n != cpu.v;
    case 14: return !cpu.z && cpu.n == cpu.v;
    default: return cpu.z || cpu.n != cpu.v;
    }
}

static uint32_t do_shift(int type, int left, uint32_t val, int cnt, int sz) {
    uint32_t m = MASKS(sz), msb = MSBS(sz);
    int bits = 8 << sz;
    uint32_t res = val & m;
    int c = 0, v = 0;
    val &= m;
    if (cnt == 0) {
        cpu.v = 0; cpu.c = (type == 2) ? cpu.x : 0;
        cpu.n = (res & msb) != 0; cpu.z = res == 0;
        return res;
    }
    switch (type) {
    case 0:
        if (!left) {
            int s = (val & msb) != 0;
            if (cnt >= bits) { res = s ? m : 0; c = s; }
            else { res = (uint32_t)(sext(val, sz) >> cnt) & m; c = (val >> (cnt - 1)) & 1; }
        } else {
            if (cnt >= bits) { res = 0; c = (cnt == bits) ? (val & 1) : 0; v = val != 0; }
            else {
                uint32_t mask = (m << (bits - cnt - 1)) & m, t = val & mask;
                res = (val << cnt) & m; c = (val >> (bits - cnt)) & 1;
                v = (t != 0 && t != mask);
            }
        }
        cpu.x = (uint8_t)c;
        break;
    case 1:
        if (!left) {
            if (cnt >= bits) { res = 0; c = (cnt == bits) ? ((val >> (bits - 1)) & 1) : 0; }
            else { res = val >> cnt; c = (val >> (cnt - 1)) & 1; }
        } else {
            if (cnt >= bits) { res = 0; c = (cnt == bits) ? (val & 1) : 0; }
            else { res = (val << cnt) & m; c = (val >> (bits - cnt)) & 1; }
        }
        cpu.x = (uint8_t)c;
        break;
    case 2: {
        int x = cpu.x;
        int n = cnt % (bits + 1);
        for (int i = 0; i < n; i++) {
            if (left) { int nx = (res & msb) != 0; res = ((res << 1) | x) & m; x = nx; }
            else { int nx = res & 1; res = (res >> 1) | (x ? msb : 0); x = nx; }
        }
        c = x; cpu.x = (uint8_t)x;
        break; }
    default: {
        int n = cnt % bits;
        if (left) { if (n) res = ((val << n) | (val >> (bits - n))) & m; c = res & 1; }
        else { if (n) res = ((val >> n) | (val << (bits - n))) & m; c = (res & msb) != 0; }
        break; }
    }
    cpu.c = (uint8_t)c; cpu.v = (uint8_t)v;
    cpu.n = (res & msb) != 0; cpu.z = res == 0;
    return res;
}

/* BCD helpers (match common reference behaviour for undefined flags) */
static uint32_t bcd_add(uint32_t src, uint32_t dst) {
    uint32_t res = (src & 0xF) + (dst & 0xF) + cpu.x;
    uint32_t t = ~res;
    if (res > 9) res += 6;
    res += (src & 0xF0) + (dst & 0xF0);
    cpu.x = cpu.c = res > 0x99;
    if (cpu.c) res -= 0xA0;
    cpu.v = ((t & res) >> 7) & 1;
    cpu.n = (res >> 7) & 1;
    res &= 0xFF;
    if (res) cpu.z = 0;
    return res;
}
static uint32_t bcd_sub(uint32_t src, uint32_t dst) {
    uint32_t res = (dst & 0xF) - (src & 0xF) - cpu.x;
    uint32_t t = ~res;
    if (res > 9) res -= 6;
    res += (dst & 0xF0) - (src & 0xF0);
    cpu.x = cpu.c = res > 0x99;
    if (cpu.c) res += 0xA0;
    res &= 0xFF;
    cpu.v = ((t & res) >> 7) & 1;
    cpu.n = (res >> 7) & 1;
    if (res) cpu.z = 0;
    return res;
}

/* ---------------------------------------------------- instructions ----- */
static void illegal(void) { cpu.pc = cpu.ppc; exception(4, 34); }
static void privilege(void) { cpu.pc = cpu.ppc; exception(8, 34); }

static void op_movem(int tomem_dir_is_load, int sz, int mode, int reg) {
    uint16_t mask = fetch16();
    int step = sz == 2 ? 4 : 2, n = 0;
    uint32_t addr;
    if (!tomem_dir_is_load) {
        if (mode == 4) {
            addr = A(reg);
            for (int i = 0; i < 16; i++) if (mask & (1u << i)) {
                int r = 15 - i;
                uint32_t val = r < 8 ? D(r) : A(r - 8);
                addr -= step; mem_w(addr, sz, val); n++;
            }
            A(reg) = addr;
        } else {
            Loc l; ea_loc(&l, mode, reg, sz); addr = l.addr;
            for (int i = 0; i < 16; i++) if (mask & (1u << i)) {
                uint32_t val = i < 8 ? D(i) : A(i - 8);
                mem_w(addr, sz, val); addr += step; n++;
            }
        }
        cpu.cycles += 8 + n * (sz == 2 ? 8 : 4);
    } else {
        if (mode == 3) addr = A(reg);
        else { Loc l; ea_loc(&l, mode, reg, sz); addr = l.addr; }
        for (int i = 0; i < 16; i++) if (mask & (1u << i)) {
            uint32_t val = mem_r(addr, sz);
            if (sz == 1) val = (uint32_t)(int32_t)(int16_t)val;
            if (i < 8) D(i) = val; else A(i - 8) = val;
            addr += step; n++;
        }
        mem_r(addr, 1); /* dummy trailing read like the real chip */
        if (mode == 3) A(reg) = addr;
        cpu.cycles += 12 + n * (sz == 2 ? 8 : 4);
    }
}

static void op_bit(int type, int mode, int reg, uint32_t cnt) {
    Loc l; int sz;
    if (mode == 0) { sz = 2; cnt &= 31; } else { sz = 0; cnt &= 7; }
    ea_loc(&l, mode, reg, sz);
    uint32_t v = loc_read(&l, sz);
    cpu.z = ((v >> cnt) & 1) == 0;
    switch (type) {
    case 0: cpu.cycles += mode == 0 ? 6 : 4 + ea_cyc(mode, reg, 0); return;
    case 1: v ^= 1u << cnt; break;
    case 2: v &= ~(1u << cnt); break;
    default: v |= 1u << cnt; break;
    }
    loc_write(&l, sz, v);
    cpu.cycles += mode == 0 ? (type == 2 ? 10 : 8) : 8 + ea_cyc(mode, reg, 0);
}

static void step(void) {
    uint16_t ir;
    cpu.ppc = cpu.pc;
    ir = fetch16();
    cpu.ir = ir;
    int reg = ir & 7, mode = (ir >> 3) & 7, g = ir >> 12;
    Loc s, d;
    uint32_t v, r;
    int sz;

    if ((g == 0x8 || g == 0x9 || g == 0xB || g == 0xC || g == 0xD || g == 0x4 || g == 0x0) && mode == 7 && reg > 4) { illegal(); return; }
    switch (g) {
    case 0x0: {
        int dn = (ir >> 9) & 7;
        if (ir & 0x100) {
            if (mode == 1) { /* MOVEP */
                uint32_t a = A(reg) + (uint32_t)(int32_t)(int16_t)fetch16();
                int opm = (ir >> 6) & 3;
                if (opm < 2) {
                    if (opm == 0) v = ((uint32_t)bus_rd8(a) << 8) | bus_rd8(a + 2);
                    else v = ((uint32_t)bus_rd8(a) << 24) | ((uint32_t)bus_rd8(a + 2) << 16) |
                             ((uint32_t)bus_rd8(a + 4) << 8) | bus_rd8(a + 6);
                    if (opm == 0) D(dn) = (D(dn) & 0xFFFF0000u) | v; else D(dn) = v;
                } else {
                    v = D(dn);
                    if (opm == 3) { bus_wr8(a, v >> 24); bus_wr8(a + 2, v >> 16); bus_wr8(a + 4, v >> 8); bus_wr8(a + 6, v); }
                    else { bus_wr8(a, v >> 8); bus_wr8(a + 2, v); }
                }
                cpu.cycles += opm & 1 ? 24 : 16;
            } else {
                if (((ir >> 6) & 3) != 0 ? !is_data_alterable(mode, reg) : (mode == 1 || (mode == 7 && reg > 4))) { illegal(); break; }
                op_bit((ir >> 6) & 3, mode, reg, D(dn));
            }
            break;
        }
        int op = (ir >> 9) & 7;
        if (op == 4) {
            uint32_t cnt = fetch16() & 0xFF;
            int type = (ir >> 6) & 3;
            if (type != 0 && !is_data_alterable(mode, reg)) { illegal(); break; }
            if (type == 0 && (mode == 1 || (mode == 7 && reg == 4))) { illegal(); break; }
            op_bit(type, mode, reg, cnt);
            break;
        }
        if (op == 7) { illegal(); break; }
        sz = (ir >> 6) & 3;
        if (sz == 3) { illegal(); break; }
        if (mode == 7 && reg == 4) { /* to CCR / SR */
            if (op != 0 && op != 1 && op != 5) { illegal(); break; }
            if (sz == 0) {
                uint32_t imm = fetch16() & 0xFF, ccr = cpu_get_sr() & 0xFF;
                ccr = op == 0 ? ccr | imm : op == 1 ? ccr & imm : ccr ^ imm;
                set_ccr((uint16_t)ccr); cpu.cycles += 20; break;
            } else if (sz == 1) {
                if (!cpu.s) { privilege(); break; }
                uint32_t imm = fetch16(), sr = cpu_get_sr();
                sr = op == 0 ? sr | imm : op == 1 ? sr & imm : sr ^ imm;
                cpu_set_sr((uint16_t)sr); cpu.cycles += 20; break;
            }
            illegal(); break;
        }
        uint32_t imm = sz == 2 ? fetch32() : (sz == 1 ? fetch16() : (fetch16() & 0xFF));
        if (!is_data_alterable(mode, reg)) { illegal(); break; }
        ea_loc(&d, mode, reg, sz);
        v = loc_read(&d, sz);
        switch (op) {
        case 0: r = v | imm; logic_flags(r, sz); loc_write(&d, sz, r); break;
        case 1: r = v & imm; logic_flags(r, sz); loc_write(&d, sz, r); break;
        case 2: r = alu_sub(imm, v, sz, 0, 0, 1); loc_write(&d, sz, r); break;
        case 3: r = alu_add(imm, v, sz, 0, 0); loc_write(&d, sz, r); break;
        case 5: r = v ^ imm; logic_flags(r, sz); loc_write(&d, sz, r); break;
        default: alu_sub(imm, v, sz, 0, 0, 0); break;
        }
        if (mode == 0) cpu.cycles += sz == 2 ? (op == 6 ? 14 : 16) : 8;
        else cpu.cycles += (op == 6 ? 8 : 12) + (sz == 2 ? 4 : 0) + ea_cyc(mode, reg, sz);
        break; }

    case 0x1: case 0x2: case 0x3: {
        sz = g == 1 ? 0 : g == 3 ? 1 : 2;
        int dmode = (ir >> 6) & 7, dreg = (ir >> 9) & 7;
        if (mode == 1 && sz == 0) { illegal(); break; }
        if (mode == 7 && reg > 4) { illegal(); break; }
        if (dmode == 1 ? sz == 0 : !is_data_alterable(dmode, dreg)) { illegal(); break; }
        ea_loc(&s, mode, reg, sz);
        v = loc_read(&s, sz);
        if (dmode == 1) {
            A(dreg) = sz == 1 ? (uint32_t)(int32_t)(int16_t)v : v;
            cpu.cycles += 4 + ea_cyc(mode, reg, sz);
        } else {
            ea_loc(&d, dmode, dreg, sz);
            loc_write(&d, sz, v);
            logic_flags(v, sz);
            cpu.cycles += 4 + ea_cyc(mode, reg, sz) + (dmode >= 2 ? ea_cyc(dmode, dreg, sz) : 0);
        }
        break; }

    case 0x4: {
        if (ir & 0x100) {
            int opm = (ir >> 6) & 7, dn = (ir >> 9) & 7;
            if (opm == 7) {
                if (!is_ctrl_mode(mode, reg)) { illegal(); break; }
                ea_loc(&s, mode, reg, 2); A(dn) = s.addr;
                cpu.cycles += 4 + (mode == 2 ? 0 : mode == 5 ? 4 : mode == 6 ? 8 : 4 + ((mode == 7 && reg == 1) ? 4 : 0) + ((mode == 7 && reg == 3) ? 4 : 0));
            } else if (opm == 6) {
                if (mode == 1) { illegal(); break; }
                ea_loc(&s, mode, reg, 1);
                int32_t bound = (int16_t)loc_read(&s, 1), val = (int16_t)D(dn);
                cpu.cycles += 10 + ea_cyc(mode, reg, 1);
                cpu.z = val == 0; cpu.v = 0; cpu.c = 0;
                if (val < 0) { cpu.n = 1; exception(6, 40); }
                else if (val > bound) { cpu.n = 0; exception(6, 40); }
            } else illegal();
            break;
        }
        sz = (ir >> 6) & 3;
        switch ((ir >> 8) & 0xF) {
        case 0x0:
            if (sz == 3) { /* MOVE from SR */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, 1); loc_write(&d, 1, cpu_get_sr());
                cpu.cycles += mode == 0 ? 6 : 8 + ea_cyc(mode, reg, 1);
            } else { /* NEGX */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, sz); v = loc_read(&d, sz);
                r = alu_sub(v, 0, sz, cpu.x, 1, 1); loc_write(&d, sz, r);
                cpu.cycles += mode == 0 ? (sz == 2 ? 6 : 4) : (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
            }
            break;
        case 0x2: /* CLR */
            if (sz == 3 || !is_data_alterable(mode, reg)) { illegal(); break; }
            ea_loc(&d, mode, reg, sz); loc_write(&d, sz, 0);
            cpu.n = 0; cpu.z = 1; cpu.v = 0; cpu.c = 0;
            cpu.cycles += mode == 0 ? (sz == 2 ? 6 : 4) : (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
            break;
        case 0x4:
            if (sz == 3) { /* MOVE to CCR */
                if (mode == 1) { illegal(); break; }
                ea_loc(&s, mode, reg, 1); v = loc_read(&s, 1); set_ccr((uint16_t)v);
                cpu.cycles += 12 + ea_cyc(mode, reg, 1);
            } else { /* NEG */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, sz); v = loc_read(&d, sz);
                r = alu_sub(v, 0, sz, 0, 0, 1); loc_write(&d, sz, r);
                cpu.cycles += mode == 0 ? (sz == 2 ? 6 : 4) : (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
            }
            break;
        case 0x6:
            if (sz == 3) { /* MOVE to SR */
                if (mode == 1) { illegal(); break; }
                if (!cpu.s) { privilege(); break; }
                ea_loc(&s, mode, reg, 1); v = loc_read(&s, 1); cpu_set_sr((uint16_t)v);
                cpu.cycles += 12 + ea_cyc(mode, reg, 1);
            } else { /* NOT */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, sz); v = loc_read(&d, sz);
                r = ~v & MASKS(sz); loc_write(&d, sz, r); logic_flags(r, sz);
                cpu.cycles += mode == 0 ? (sz == 2 ? 6 : 4) : (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
            }
            break;
        case 0x8:
            if (sz == 0) { /* NBCD */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, 0); v = loc_read(&d, 0);
                r = (0x9A - v - cpu.x) & 0xFF;
                if (r != 0x9A) {
                    uint32_t t = ~r;
                    if ((r & 0x0F) == 0x0A) r = (r & 0xF0) + 0x10;
                    r &= 0xFF; cpu.v = ((t & r) >> 7) & 1;
                    loc_write(&d, 0, r); if (r) cpu.z = 0; cpu.c = cpu.x = 1;
                } else { cpu.v = 0; cpu.c = cpu.x = 0; }
                cpu.n = (r >> 7) & 1;
                cpu.cycles += mode == 0 ? 6 : 8 + ea_cyc(mode, reg, 0);
            } else if (sz == 1) {
                if (mode == 0) { /* SWAP */
                    r = (D(reg) << 16) | (D(reg) >> 16); D(reg) = r; logic_flags(r, 2); cpu.cycles += 4;
                } else { /* PEA */
                    if (!is_ctrl_mode(mode, reg)) { illegal(); break; }
                    ea_loc(&s, mode, reg, 2); push32(s.addr);
                    cpu.cycles += 12 + (mode == 2 ? 0 : mode == 5 ? 4 : mode == 6 ? 8 : 4 + ((mode == 7 && reg == 1) ? 4 : 0) + ((mode == 7 && reg == 3) ? 4 : 0));
                }
            } else {
                if (mode == 0) { /* EXT */
                    if (sz == 2) { r = (uint32_t)(int32_t)(int8_t)D(reg); D(reg) = (D(reg) & 0xFFFF0000u) | (r & 0xFFFF); logic_flags(r, 1); }
                    else { r = (uint32_t)(int32_t)(int16_t)D(reg); D(reg) = r; logic_flags(r, 2); }
                    cpu.cycles += 4;
                } else {
                    if (mode == 3 || !(mode == 4 || (is_ctrl_mode(mode, reg) && !(mode == 7 && reg >= 2)))) { illegal(); break; }
                    op_movem(0, sz == 2 ? 1 : 2, mode, reg);
                }
            }
            break;
        case 0xA:
            if (sz == 3) {
                if (ir == 0x4AFC) { illegal(); break; }
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, 0); v = loc_read(&d, 0);
                logic_flags(v, 0); loc_write(&d, 0, v | 0x80);
                cpu.cycles += mode == 0 ? 4 : 10 + ea_cyc(mode, reg, 0);
            } else { /* TST */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&s, mode, reg, sz); v = loc_read(&s, sz); logic_flags(v, sz);
                cpu.cycles += 4 + ea_cyc(mode, reg, sz);
            }
            break;
        case 0xC: /* MOVEM mem->reg */
            if (sz < 2) { illegal(); break; }
            if (mode == 4 || !(is_ctrl_mode(mode, reg) || mode == 3)) { illegal(); break; }
            op_movem(1, sz == 2 ? 1 : 2, mode, reg);
            break;
        default: /* 0xE */
            if (ir & 0x80) { /* JSR / JMP */
                if (!is_ctrl_mode(mode, reg)) { illegal(); break; }
                ea_loc(&s, mode, reg, 2);
                int base = mode == 2 ? 8 : mode == 5 ? 10 : mode == 6 ? 14 : (mode == 7 && reg == 1) ? 12 : (mode == 7 && reg == 3) ? 14 : 10;
                if (!(ir & 0x40)) { push32(cpu.pc); base += 8; }
                cpu.pc = s.addr; cpu.cycles += base;
                break;
            }
            if (sz == 3 && !(ir & 0x80)) { /* 4E40-4E7F */ }
            switch (ir & 0x00F8) {
            case 0x40: case 0x48: /* TRAP */
                exception(32 + (ir & 15), 34); break;
            case 0x50: /* LINK */
                if (reg == 7) { A(7) -= 4; w32(A(7), A(7)); } else { push32(A(reg)); A(reg) = A(7); }
                A(7) += (uint32_t)(int32_t)(int16_t)fetch16(); cpu.cycles += 16; break;
            case 0x58: /* UNLK */
                { uint32_t sp = A(reg); uint32_t nv = r32(sp); A(7) = sp + 4; A(reg) = nv; } cpu.cycles += 12; break;
            case 0x60:
                if (!cpu.s) { privilege(); break; }
                cpu.usp = A(reg); cpu.cycles += 4; break;
            case 0x68:
                if (!cpu.s) { privilege(); break; }
                A(reg) = cpu.usp; cpu.cycles += 4; break;
            case 0x70:
                switch (ir & 7) {
                case 0: if (!cpu.s) { privilege(); break; } bus_reset_instr(); cpu.cycles += 132; break;
                case 1: cpu.cycles += 4; break;
                case 2: if (!cpu.s) { privilege(); break; } cpu_set_sr(fetch16()); cpu.stopped = 1; cpu.cycles += 4; break;
                case 3: if (!cpu.s) { privilege(); break; } { uint32_t sr = pop16(); cpu.pc = pop32(); cpu_set_sr((uint16_t)sr); } cpu.cycles += 20; break;
                case 5: cpu.pc = pop32(); cpu.cycles += 16; break;
                case 6: if (cpu.v) exception(7, 34); else cpu.cycles += 4; break;
                case 7: { uint32_t ccr = pop16(); cpu.pc = pop32(); set_ccr((uint16_t)ccr); } cpu.cycles += 20; break;
                default: illegal(); break;
                }
                break;
            default: illegal(); break;
            }
            break;
        }
        break; }

    case 0x5: {
        sz = (ir >> 6) & 3;
        if (sz == 3) {
            int cc = (ir >> 8) & 15;
            if (mode == 1) { /* DBcc */
                uint32_t base = cpu.pc;
                int16_t disp = (int16_t)fetch16();
                if (cond_true(cc)) { cpu.cycles += 12; }
                else {
                    uint16_t cnt = (uint16_t)(D(reg) - 1);
                    D(reg) = (D(reg) & 0xFFFF0000u) | cnt;
                    if (cnt != 0xFFFF) { cpu.pc = base + (uint32_t)(int32_t)disp; cpu.cycles += 10; }
                    else cpu.cycles += 14;
                }
            } else { /* Scc */
                if (!is_data_alterable(mode, reg)) { illegal(); break; }
                ea_loc(&d, mode, reg, 0);
                int t = cond_true(cc);
                loc_write(&d, 0, t ? 0xFF : 0);
                cpu.cycles += mode == 0 ? (t ? 6 : 4) : 8 + ea_cyc(mode, reg, 0);
            }
            break;
        }
        v = (ir >> 9) & 7; if (!v) v = 8;
        if (mode == 1) {
            if (sz == 0) { illegal(); break; }
            A(reg) = (ir & 0x100) ? A(reg) - v : A(reg) + v;
            cpu.cycles += 8;
        } else {
            if (!is_data_alterable(mode, reg)) { illegal(); break; }
            ea_loc(&d, mode, reg, sz);
            uint32_t o = loc_read(&d, sz);
            r = (ir & 0x100) ? alu_sub(v, o, sz, 0, 0, 1) : alu_add(v, o, sz, 0, 0);
            loc_write(&d, sz, r);
            cpu.cycles += mode == 0 ? (sz == 2 ? 8 : 4) : (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
        }
        break; }

    case 0x6: {
        int cc = (ir >> 8) & 15;
        uint32_t base = cpu.pc;
        int32_t disp = (int8_t)ir;
        int word = ((ir & 0xFF) == 0);
        if (word) disp = (int16_t)fetch16();
        if (cc == 1) { push32(cpu.pc); cpu.pc = base + (uint32_t)disp; cpu.cycles += 18; }
        else if (cond_true(cc)) { cpu.pc = base + (uint32_t)disp; cpu.cycles += 10; }
        else cpu.cycles += word ? 12 : 8;
        break; }

    case 0x7:
        if (ir & 0x100) { illegal(); break; }
        r = (uint32_t)(int32_t)(int8_t)ir; D((ir >> 9) & 7) = r; logic_flags(r, 2); cpu.cycles += 4;
        break;

    case 0x8: case 0xC: {
        int dn = (ir >> 9) & 7, opm = (ir >> 6) & 7;
        int isand = g == 0xC;
        if (opm == 3 || opm == 7) {
            if (mode == 1) { illegal(); break; }
            ea_loc(&s, mode, reg, 1);
            uint32_t src = loc_read(&s, 1);
            if (isand) { /* MULU / MULS */
                if (opm == 3) { r = (D(dn) & 0xFFFF) * (src & 0xFFFF); int b = 0; for (uint32_t t = src & 0xFFFF; t; t >>= 1) b += t & 1; cpu.cycles += 38 + 2 * b + ea_cyc(mode, reg, 1); }
                else { r = (uint32_t)((int32_t)(int16_t)D(dn) * (int32_t)(int16_t)src); uint32_t t = ((src & 0xFFFF) << 1) ^ (src & 0xFFFF); int b = 0; for (t &= 0x1FFFF; t; t >>= 1) b += t & 1; cpu.cycles += 38 + 2 * b + ea_cyc(mode, reg, 1); }
                D(dn) = r; cpu.n = (r >> 31) & 1; cpu.z = r == 0; cpu.v = 0; cpu.c = 0;
            } else if (opm == 3) { /* DIVU */
                uint32_t dv = src & 0xFFFF;
                if (!dv) { exception(5, 38); break; }
                uint32_t q = D(dn) / dv, rm = D(dn) % dv;
                if (q > 0xFFFF) { cpu.v = 1; }
                else { D(dn) = (rm << 16) | q; cpu.n = (q >> 15) & 1; cpu.z = q == 0; cpu.v = 0; cpu.c = 0; }
                cpu.cycles += 90 + ea_cyc(mode, reg, 1);
            } else { /* DIVS */
                int32_t dv = (int16_t)src;
                if (!dv) { exception(5, 38); break; }
                int64_t dd = (int32_t)D(dn);
                if (D(dn) == 0x80000000u && dv == -1) { D(dn) = 0; cpu.z = 0; cpu.n = 0; cpu.v = 0; cpu.c = 0; break; }
                int64_t q = dd / dv, rm = dd % dv;
                if (q > 32767 || q < -32768) { cpu.v = 1; }
                else { D(dn) = ((uint32_t)(rm & 0xFFFF) << 16) | ((uint32_t)q & 0xFFFF); cpu.n = (q < 0); cpu.z = q == 0; cpu.v = 0; cpu.c = 0; }
                cpu.cycles += 110 + ea_cyc(mode, reg, 1);
            }
            break;
        }
        if (opm == 4 && mode <= 1) { /* SBCD / ABCD */
            int rm = mode & 1;
            if (rm) {
                A(reg) -= (reg == 7) ? 2 : 1; uint32_t sa = A(reg);
                A(dn) -= (dn == 7) ? 2 : 1; uint32_t da = A(dn);
                uint32_t sv = bus_rd8(sa), dv = bus_rd8(da);
                r = isand ? bcd_add(sv, dv) : bcd_sub(sv, dv);
                bus_wr8(da, (uint8_t)r); cpu.cycles += 18;
            } else {
                r = isand ? bcd_add(D(reg) & 0xFF, D(dn) & 0xFF) : bcd_sub(D(reg) & 0xFF, D(dn) & 0xFF);
                D(dn) = (D(dn) & ~0xFFu) | r; cpu.cycles += 6;
            }
            break;
        }
        if (isand && opm >= 5 && mode <= 1 && (opm == 5 || opm == 6)) { /* EXG */
            if (opm == 5 && mode == 0) { uint32_t t = D(dn); D(dn) = D(reg); D(reg) = t; }
            else if (opm == 5 && mode == 1) { uint32_t t = A(dn); A(dn) = A(reg); A(reg) = t; }
            else if (opm == 6 && mode == 1) { uint32_t t = D(dn); D(dn) = A(reg); A(reg) = t; }
            else { illegal(); break; }
            cpu.cycles += 6; break;
        }
        sz = opm & 3;
        if (sz == 3) { illegal(); break; }
        if (opm < 3) {
            if (mode == 1) { illegal(); break; }
            ea_loc(&s, mode, reg, sz);
            v = loc_read(&s, sz);
            r = isand ? (v & D(dn)) : (v | D(dn));
            logic_flags(r, sz);
            { uint32_t m = MASKS(sz); D(dn) = (D(dn) & ~m) | (r & m); }
            cpu.cycles += (sz == 2 ? 6 : 4) + ea_cyc(mode, reg, sz) - ((sz == 2 && mode >= 2 && !(mode == 7 && reg == 4)) ? 0 : 0);
        } else {
            if (!is_mem_alterable(mode, reg)) { illegal(); break; }
            ea_loc(&d, mode, reg, sz);
            v = loc_read(&d, sz);
            r = isand ? (v & D(dn)) : (v | D(dn));
            logic_flags(r, sz); loc_write(&d, sz, r);
            cpu.cycles += (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
        }
        break; }

    case 0x9: case 0xD: {
        int dn = (ir >> 9) & 7, opm = (ir >> 6) & 7;
        int isadd = g == 0xD;
        if (opm == 3 || opm == 7) { /* ADDA / SUBA */
            sz = opm == 3 ? 1 : 2;
            ea_loc(&s, mode, reg, sz);
            v = loc_read(&s, sz);
            if (sz == 1) v = (uint32_t)(int32_t)(int16_t)v;
            A(dn) = isadd ? A(dn) + v : A(dn) - v;
            cpu.cycles += (sz == 2 && (mode <= 1 || (mode == 7 && reg == 4)) ? 8 : 4 + (sz == 2 ? 2 : 0)) + ea_cyc(mode, reg, sz) - ((sz == 2 && mode > 1 && !(mode == 7 && reg == 4)) ? 0 : 0);
            break;
        }
        sz = opm & 3;
        if (opm >= 4 && mode <= 1) { /* ADDX / SUBX */
            if (mode == 1) {
                A(reg) -= sz == 0 ? (reg == 7 ? 2 : 1) : sz == 1 ? 2 : 4; uint32_t sa = A(reg);
                A(dn) -= sz == 0 ? (dn == 7 ? 2 : 1) : sz == 1 ? 2 : 4; uint32_t da = A(dn);
                uint32_t sv = mem_r(sa, sz), dv = mem_r(da, sz);
                r = isadd ? alu_add(sv, dv, sz, cpu.x, 1) : alu_sub(sv, dv, sz, cpu.x, 1, 1);
                mem_w(da, sz, r); cpu.cycles += sz == 2 ? 30 : 18;
            } else {
                uint32_t m = MASKS(sz);
                r = isadd ? alu_add(D(reg), D(dn), sz, cpu.x, 1) : alu_sub(D(reg), D(dn), sz, cpu.x, 1, 1);
                D(dn) = (D(dn) & ~m) | (r & m); cpu.cycles += sz == 2 ? 8 : 4;
            }
            break;
        }
        if (opm < 3) {
            if (mode == 1 && sz == 0) { illegal(); break; }
            ea_loc(&s, mode, reg, sz);
            v = loc_read(&s, sz);
            uint32_t m = MASKS(sz);
            r = isadd ? alu_add(v, D(dn), sz, 0, 0) : alu_sub(v, D(dn), sz, 0, 0, 1);
            D(dn) = (D(dn) & ~m) | (r & m);
            cpu.cycles += (sz == 2 ? 6 : 4) + ea_cyc(mode, reg, sz);
        } else {
            if (!is_mem_alterable(mode, reg)) { illegal(); break; }
            ea_loc(&d, mode, reg, sz);
            v = loc_read(&d, sz);
            r = isadd ? alu_add(D(dn), v, sz, 0, 0) : alu_sub(D(dn), v, sz, 0, 0, 1);
            loc_write(&d, sz, r);
            cpu.cycles += (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
        }
        break; }

    case 0xB: {
        int dn = (ir >> 9) & 7, opm = (ir >> 6) & 7;
        if (opm == 3 || opm == 7) { /* CMPA */
            sz = opm == 3 ? 1 : 2;
            ea_loc(&s, mode, reg, sz);
            v = loc_read(&s, sz);
            if (sz == 1) v = (uint32_t)(int32_t)(int16_t)v;
            alu_sub(v, A(dn), 2, 0, 0, 0);
            cpu.cycles += 6 + ea_cyc(mode, reg, sz);
            break;
        }
        sz = opm & 3;
        if (opm < 3) { /* CMP */
            if (mode == 1 && sz == 0) { illegal(); break; }
            ea_loc(&s, mode, reg, sz);
            v = loc_read(&s, sz);
            alu_sub(v, D(dn), sz, 0, 0, 0);
            cpu.cycles += (sz == 2 ? 6 : 4) + ea_cyc(mode, reg, sz);
        } else if (mode == 1) { /* CMPM */
            uint32_t sa, da;
            { int inc = sz == 0 ? (reg == 7 ? 2 : 1) : sz == 1 ? 2 : 4; sa = A(reg); A(reg) += inc; }
            { int inc = sz == 0 ? (dn == 7 ? 2 : 1) : sz == 1 ? 2 : 4; da = A(dn); A(dn) += inc; }
            uint32_t sv = mem_r(sa, sz), dv = mem_r(da, sz);
            alu_sub(sv, dv, sz, 0, 0, 0);
            cpu.cycles += sz == 2 ? 20 : 12;
        } else { /* EOR */
            if (!is_data_alterable(mode, reg)) { illegal(); break; }
            ea_loc(&d, mode, reg, sz);
            v = loc_read(&d, sz);
            r = v ^ D(dn);
            logic_flags(r, sz); loc_write(&d, sz, r);
            cpu.cycles += mode == 0 ? (sz == 2 ? 8 : 4) : (sz == 2 ? 12 : 8) + ea_cyc(mode, reg, sz);
        }
        break; }

    case 0xE: {
        sz = (ir >> 6) & 3;
        int left = (ir >> 8) & 1;
        if (sz == 3) {
            if ((ir & 0x0800) || !is_mem_alterable(mode, reg)) { illegal(); break; }
            ea_loc(&d, mode, reg, 1);
            v = loc_read(&d, 1);
            r = do_shift((ir >> 9) & 3, left, v, 1, 1);
            loc_write(&d, 1, r);
            cpu.cycles += 8 + ea_cyc(mode, reg, 1);
        } else {
            int cnt = (ir >> 9) & 7;
            if (ir & 0x20) cnt = D(cnt) & 63; else if (!cnt) cnt = 8;
            uint32_t m = MASKS(sz);
            r = do_shift((ir >> 3) & 3, left, D(reg), cnt, sz);
            D(reg) = (D(reg) & ~m) | (r & m);
            cpu.cycles += (sz == 2 ? 8 : 6) + 2 * cnt;
        }
        break; }

    case 0xA:
        cpu.pc = cpu.ppc; exception(10, 34); break;
    default: /* 0xF */
        cpu.pc = cpu.ppc; exception(11, 34); break;
    }
}

/* ------------------------------------------------------- driver ------- */
void cpu_reset(uint32_t ssp, uint32_t pc) {
    for (int i = 0; i < 8; i++) { cpu.d[i] = 0; cpu.a[i] = 0; }
    cpu.s = 1; cpu.t = 0; cpu.imask = 7;
    cpu.x = cpu.n = cpu.z = cpu.v = cpu.c = 0;
    cpu.ssp = ssp; cpu.usp = 0; cpu.a[7] = ssp; cpu.pc = pc;
    cpu.stopped = cpu.halted = 0; cpu.fault = 0; cpu.nmi_edge = 0; cpu.irq_level = 0;
    cpu.cycles = 0;
}
void cpu_set_irq(int level) { cpu.irq_level = (uint8_t)level; }
void cpu_nmi(void) { cpu.nmi_edge = 1; }

int cpu_execute(int cycles) {
    cpu.cycles = 0;
    while (cpu.cycles < cycles) {
        if (cpu.halted) { cpu.cycles = cycles; break; }
        if (cpu.nmi_edge || cpu.irq_level > cpu.imask) {
            int lvl = cpu.nmi_edge ? 7 : cpu.irq_level;
            cpu.nmi_edge = 0;
            int vec = bus_irq_ack(lvl);
            uint16_t old = cpu_get_sr();
            enter_super(); cpu.t = 0; cpu.imask = (uint8_t)lvl; cpu.stopped = 0;
            A(7) -= 6; w32(A(7) + 2, cpu.pc); w16(A(7), old);
            cpu.pc = r32((uint32_t)vec * 4);
            cpu.cycles += 44;
            if (cpu.fault) { cpu.fault = 0; cpu.halted = 1; }
            continue;
        }
        if (cpu.stopped) { cpu.cycles = cycles; break; }
        int tr = cpu.t;
        step();
        if (cpu.fault) take_fault();
        else if (tr && !cpu.halted) exception(9, 34);
    }
    return cpu.cycles;
}
