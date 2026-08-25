/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_AMO_EMU_H
#define _ASM_RISCV_AMO_EMU_H

#include <asm/ptrace.h>

#ifdef CONFIG_RISCV_AMO_EMULATION
bool try_amo_emulation(struct pt_regs *regs);
#else
static inline bool try_amo_emulation(struct pt_regs *regs)
{
	return false;
}
#endif

#endif /* _ASM_RISCV_AMO_EMU_H */
