// SPDX-License-Identifier: GPL-2.0
/*
 * Emulation of AMO / LR / SC instructions on memory that does not
 * support hardware atomics (RP2350 PSRAM).
 *
 * RP2350's Hazard3 cores implement AMOs as an exclusive read/write pair,
 * and the Global Exclusive Monitor only supports main SRAM (RP2350
 * datasheet section 3.1.5).  Any AMO/LR/SC targeting the PSRAM window
 * (0x11000000) raises a Store/AMO fault (mcause 6/7).  Because this
 * kernel runs from PSRAM, its first atomic operation (in
 * boot_cpu_init()->set_cpu_online()) faults before any console output.
 *
 * This trap-path emulator decodes the faulting instruction and performs
 * the read-modify-write with plain loads/stores.  Atomicity is safe on
 * this platform: single-hart (CONFIG_SMP=n) and M-mode traps disable
 * interrupts, so the load-modify-store cannot be interleaved.
 *
 * Limitations:
 *  - The LR/SC reservation is tracked in software; a plain (non-faulting)
 *    store between LR and SC does not invalidate the reservation.
 *  - Only the PSRAM window is emulated; other faulting AMOs fall through
 *    to the normal die() path.
 */

#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/types.h>

#include <asm/amo-emu.h>
#include <asm/csr.h>

#define AMO_OPCODE	0x2f
#define PSRAM_BASE	0x11000000UL
#define PSRAM_SIZE	0x00800000UL
#define RES_GRANULE	16

static unsigned long emu_res_addr = ~0UL;

/*
 * RISC-V pt_regs stores GPRs as named fields in ABI order (x1..x31,
 * x10..x17 = a0..a7).  Map an instruction register number to its slot.
 */
static unsigned long *reg_by_num(struct pt_regs *regs, unsigned int n)
{
	switch (n) {
	case 1:  return &regs->ra;
	case 2:  return &regs->sp;
	case 3:  return &regs->gp;
	case 4:  return &regs->tp;
	case 5:  return &regs->t0;
	case 6:  return &regs->t1;
	case 7:  return &regs->t2;
	case 8:  return &regs->s0;
	case 9:  return &regs->s1;
	case 10: return &regs->a0;
	case 11: return &regs->a1;
	case 12: return &regs->a2;
	case 13: return &regs->a3;
	case 14: return &regs->a4;
	case 15: return &regs->a5;
	case 16: return &regs->a6;
	case 17: return &regs->a7;
	case 18: return &regs->s2;
	case 19: return &regs->s3;
	case 20: return &regs->s4;
	case 21: return &regs->s5;
	case 22: return &regs->s6;
	case 23: return &regs->s7;
	case 24: return &regs->s8;
	case 25: return &regs->s9;
	case 26: return &regs->s10;
	case 27: return &regs->s11;
	case 28: return &regs->t3;
	case 29: return &regs->t4;
	case 30: return &regs->t5;
	case 31: return &regs->t6;
	default: return NULL;	/* x0 */
	}
}

static bool addr_in_psram(unsigned long addr)
{
	return addr >= PSRAM_BASE && addr < PSRAM_BASE + PSRAM_SIZE;
}

static void emu_clear_reservation(unsigned long addr)
{
	if (emu_res_addr != ~0UL &&
	    (addr & ~(RES_GRANULE - 1)) == (emu_res_addr & ~(RES_GRANULE - 1)))
		emu_res_addr = ~0UL;
}

bool try_amo_emulation(struct pt_regs *regs)
{
	u32 insn;
	unsigned int funct5, rs1num, rs2num, rdnum;
	unsigned long *r;
	unsigned long addr, rs2, old, val;

	/* Store/AMO access fault (6), Store/AMO page fault (7), store page fault (15) */
	if (regs->cause != EXC_STORE_MISALIGNED &&
	    regs->cause != EXC_STORE_ACCESS &&
	    regs->cause != EXC_STORE_PAGE_FAULT)
		return false;

	/* Fetch the faulting instruction (kernel text lives in readable PSRAM). */
	if (!addr_in_psram(regs->epc))
		return false;
	insn = *(u32 *)regs->epc;
	if ((insn & 0x7f) != AMO_OPCODE)
		return false; /* not an AMO/LR/SC -> normal fault handling */

	funct5 = (insn >> 27) & 0x1f;
	rs1num = (insn >> 15) & 0x1f;
	rs2num = (insn >> 20) & 0x1f;
	rdnum  = (insn >> 7) & 0x1f;

	r = reg_by_num(regs, rs1num);
	addr = r ? *r : 0;
	if (!addr_in_psram(addr))
		return false;

	r = reg_by_num(regs, rs2num);
	rs2 = r ? *r : 0;

	if (funct5 == 0x02) {			/* lr.w */
		old = *(volatile u32 *)addr;
		emu_res_addr = addr;
		if (rdnum)
			*reg_by_num(regs, rdnum) = old;
		regs->epc += 4;
		return true;
	}

	if (funct5 == 0x03) {			/* sc.w */
		if (emu_res_addr != ~0UL &&
		    (addr & ~(RES_GRANULE - 1)) == (emu_res_addr & ~(RES_GRANULE - 1))) {
			*(volatile u32 *)addr = rs2;
			val = 0;
		} else {
			val = 1;		/* reservation lost */
		}
		emu_res_addr = ~0UL;
		if (rdnum)
			*reg_by_num(regs, rdnum) = val;
		regs->epc += 4;
		return true;
	}

	/* AMO read-modify-write: rd gets the old memory value */
	old = *(volatile u32 *)addr;
	switch (funct5) {
	case 0x00: val = old + rs2; break;				/* amoadd.w */
	case 0x01: val = rs2; break;					/* amoswap.w */
	case 0x04: val = old ^ rs2; break;				/* amoxor.w */
	case 0x08: val = old | rs2; break;				/* amoor.w */
	case 0x0c: val = old & rs2; break;				/* amoand.w */
	case 0x10: val = min_t(s32, (s32)old, (s32)rs2); break;	/* amomin.w */
	case 0x14: val = max_t(s32, (s32)old, (s32)rs2); break;	/* amomax.w */
	case 0x18: val = min_t(u32, old, rs2); break;		/* amominu.w */
	case 0x1c: val = max_t(u32, old, rs2); break;		/* amomaxu.w */
	default:
		return false;
	}
	emu_clear_reservation(addr);
	*(volatile u32 *)addr = val;
	if (rdnum)
		*reg_by_num(regs, rdnum) = old;
	regs->epc += 4;
	return true;
}
