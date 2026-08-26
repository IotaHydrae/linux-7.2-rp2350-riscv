// SPDX-License-Identifier: GPL-2.0
/*
 * Emulation of AMO / LR / SC instructions on memory that does not
 * support hardware atomics (RP2350 PSRAM).
 *
 * RP2350's Hazard3 cores implement AMOs as an exclusive read/write pair,
 * and the Global Exclusive Monitor only supports main SRAM (RP2350
 * datasheet section 2.1.6).  An AMO targeting the PSRAM window
 * (0x11000000) raises a Store/AMO fault (mcause 6/7), but LR/SC do NOT
 * fault: the monitor treats exclusive accesses outside SRAM as normal
 * accesses and reports exclusivity failure, so sc.w silently returns a
 * non-zero value and every LR/SC retry loop spins forever.  Because this
 * kernel runs from PSRAM, its first atomic operation (in
 * boot_cpu_init()->set_cpu_online()) faults before any console output.
 *
 * To get around the LR/SC behaviour, cmpxchg.h is patched (under
 * CONFIG_RISCV_AMO_EMULATION) to use amocas.w (Zacas) for 32-bit
 * compare-and-swap.  amocas.w is an AMO encoding, so it faults -- and
 * since Hazard3 has no Zacas decoder it actually raises an illegal
 * instruction (mcause 2), which this emulator also handles.
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
 *
 */

#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/types.h>

#include <asm/amo-emu.h>
#include <asm/csr.h>

#define AMO_OPCODE	0x2f
#define AMO_FUNCT5_AMOCAS	0x05	/* Zacas amocas.w (reserved in base A) */
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
	bool illegal = (regs->cause == EXC_INST_ILLEGAL);

	/* Store/AMO faults (6/7/15) cover real AMOs on PSRAM.  Hazard3 has
	 * no Zacas decoder, so amocas.w arrives as an illegal-instruction
	 * trap (2) instead -- only that case is interesting from there. */
	if (!illegal &&
	    regs->cause != EXC_STORE_MISALIGNED &&
	    regs->cause != EXC_STORE_ACCESS &&
	    regs->cause != EXC_STORE_PAGE_FAULT)
		return false;

	/* Fetch the faulting instruction (kernel text lives in readable PSRAM).
	 *
	 * With the C (compressed) extension a 32-bit instruction may start at
	 * any 16-bit boundary, so mepc can be 2 mod 4 (e.g. an AMO right after
	 * a c.beqz).  RP2350's Hazard3 returns 0xf0000000 for misaligned 32-bit
	 * data loads, so never do a misaligned u32 read here: assemble the word
	 * from two 16-bit halfwords, both of which are 2-byte aligned.
	 * A 16-bit compressed instruction (first halfword low bits != 11) is
	 * never an AMO/LR/SC and is refused right away. */
	if (!addr_in_psram(regs->epc))
		return false;
	{
		u16 lo = *(u16 *)regs->epc;
		u16 hi = *(u16 *)(regs->epc + 2);

		insn = (u32)lo | ((u32)hi << 16);
		if ((insn & 0x3) != 0x3)
			return false;	/* 16-bit compressed instruction */
	}
	if ((insn & 0x7f) != AMO_OPCODE)
		return false;

	funct5 = (insn >> 27) & 0x1f;
	rs1num = (insn >> 15) & 0x1f;
	rs2num = (insn >> 20) & 0x1f;
	rdnum  = (insn >> 7) & 0x1f;

	/* From an illegal-instruction trap we only ever emulate amocas.w;
	 * any other illegal instruction falls through to the normal die(). */
	if (illegal && funct5 != AMO_FUNCT5_AMOCAS)
		return false;

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

	if (funct5 == AMO_FUNCT5_AMOCAS) {	/* amocas.w rd, rs2, (rs1) */
		unsigned long expected;
		unsigned long *rdp = reg_by_num(regs, rdnum);

		/* Zacas amocas.w requires funct3=010 (word); rv32 has no .d */
		if (((insn >> 12) & 0x7) != 0x2)
			return false;

		/* rd is both input (expected value) and output (old value):
		 * if memory equals expected, rs2 is stored; rd gets old. */
		expected = rdp ? *rdp : 0;
		old = *(volatile u32 *)addr;
		if (old == expected)
			*(volatile u32 *)addr = rs2;
		if (rdp)
			*rdp = old;
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
