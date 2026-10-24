/* cpu.h - MC68000 interpreter (original implementation, no libc required) */
#ifndef CPU_H
#define CPU_H
#include <stdint.h>

typedef struct {
    uint32_t d[8];
    uint32_t a[8];          /* a[7] is the active stack pointer */
    uint32_t usp, ssp;      /* storage for the inactive stack pointer */
    uint32_t pc;
    uint32_t ppc;           /* address of the instruction being executed */
    uint16_t ir;            /* current opcode */
    uint8_t  x, n, z, v, c; /* CCR flags (0/1) */
    uint8_t  t, s;          /* trace / supervisor */
    uint8_t  imask;         /* interrupt mask 0..7 */
    uint8_t  stopped;
    uint8_t  halted;
    uint8_t  irq_level;     /* level currently presented to the CPU (0..7) */
    uint8_t  nmi_edge;      /* level 7 edge-triggered latch */
    uint8_t  fault;         /* 0 none, 1 bus error, 2 address error */
    uint8_t  fault_rw;      /* 1 = read */
    uint8_t  fault_fc;      /* function code */
    uint32_t fault_addr;
    int32_t  cycles;        /* cycles consumed by cpu_execute so far */
} CPU;

extern CPU cpu;

/* provided by the machine (bus) */
uint8_t  bus_rd8(uint32_t a);
uint16_t bus_rd16(uint32_t a);
void     bus_wr8(uint32_t a, uint8_t v);
void     bus_wr16(uint32_t a, uint16_t v);
int      bus_irq_ack(int level);    /* returns vector number */
void     bus_reset_instr(void);     /* RESET instruction */

void     cpu_reset(uint32_t ssp, uint32_t pc);
int      cpu_execute(int cycles);   /* returns cycles actually executed */
uint16_t cpu_get_sr(void);
void     cpu_set_sr(uint16_t sr);
void     cpu_set_irq(int level);
void     cpu_nmi(void);

#define CPU_FC_UD 1
#define CPU_FC_UP 2
#define CPU_FC_SD 5
#define CPU_FC_SP 6
#endif
