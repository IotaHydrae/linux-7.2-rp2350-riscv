// SPDX-License-Identifier: GPL-2.0
/*
 * RP2350 RISC-V platform timer (SIO MTIME/MTIMECMP)
 *
 * The RP2350 SIO block (0xd0000000) contains the standard RISC-V 64-bit
 * machine-mode timer (MTIME @ 0x1b0/0x1b4), a per-core comparator
 * (MTIMECMP @ 0x1b8/0x1bc) and a control register (MTIME_CTRL @ 0x1a4).
 * When MTIME >= MTIMECMP the interrupt asserts MIP.MTIP directly, so no
 * external interrupt controller is needed for the timer.
 *
 * Unlike the SiFive CLINT the counter is fed by the system tick generator
 * (1 MHz, derived from the fixed 12 MHz clk_ref) unless MTIME_CTRL
 * FULLSPEED is set.  We keep the default 1 MHz tick so the timebase is
 * independent of the CPU clock, which the bootloader may change.  If the
 * tick generator is not running (e.g. a non-SDK bootloader) we start it.
 */

#define pr_fmt(fmt) "rp2350-timer: " fmt

#include <linux/bitops.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/smp.h>

#include <asm/clint.h>
#include <asm/csr.h>
#include <asm/delay.h>
#include <asm/timex.h>

#define RP2350_SIO_MTIME_CTRL	0x1a4
#define RP2350_SIO_MTIME_LO	0x1b0
#define RP2350_SIO_MTIME_HI	0x1b4
#define RP2350_SIO_MTIMECMP_LO	0x1b8
#define RP2350_SIO_MTIMECMP_HI	0x1bc

/* Tick generator peripheral (datasheet 8.5): RISC-V platform timer tick. */
#define RP2350_TICKS_BASE	0x40108000
#define RP2350_TICKS_RISCV_CTRL	0x3c
#define RP2350_TICKS_RISCV_CYCLES	0x40
#define TICKS_RUNNING		BIT(1)
#define TICKS_ENABLE		BIT(0)

/* clk_ref is a fixed 12 MHz in RP2350; 12 cycles -> 1 MHz timebase. */
#define RP2350_CLK_REF_HZ	12000000

static void __iomem *mtime_lo;
static void __iomem *mtime_hi;
static void __iomem *mtimecmp_lo;
static void __iomem *mtimecmp_hi;
static void __iomem *ticks_ctrl;
static void __iomem *ticks_cycles;
static u32 timer_freq;
static unsigned int timer_irq;

static u64 notrace rp2350_get_cycles64(void)
{
	u32 hi, lo;

	/* Datasheet 3.1.8: retry until two upper-half reads agree. */
	do {
		hi = readl_relaxed(mtime_hi);
		lo = readl_relaxed(mtime_lo);
	} while (hi != readl_relaxed(mtime_hi));

	return ((u64)hi << 32) | lo;
}

static u64 rp2350_clocksource_read(struct clocksource *cs)
{
	return rp2350_get_cycles64();
}

static struct clocksource rp2350_clocksource = {
	.name		= "rp2350_mtime",
	.rating		= 300,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= rp2350_clocksource_read,
};

static int rp2350_clock_next_event(unsigned long delta,
				   struct clock_event_device *ce)
{
	u64 next = rp2350_get_cycles64() + delta;
	u32 hi = next >> 32;
	u32 lo = next & 0xffffffff;

	/*
	 * Datasheet 3.1.8: write all-ones to the low half, then the upper
	 * half, then the lower half, to avoid a spurious interrupt while
	 * the comparator temporarily wraps.
	 */
	writel_relaxed(0xffffffff, mtimecmp_lo);
	writel_relaxed(hi, mtimecmp_hi);
	writel_relaxed(lo, mtimecmp_lo);

	csr_set(CSR_IE, IE_TIE);
	return 0;
}

static DEFINE_PER_CPU(struct clock_event_device, rp2350_clock_event) = {
	.name			= "rp2350_mtimecmp",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 100,
	.set_next_event		= rp2350_clock_next_event,
};

static irqreturn_t rp2350_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evdev = this_cpu_ptr(&rp2350_clock_event);

	csr_clear(CSR_IE, IE_TIE);
	evdev->event_handler(evdev);

	return IRQ_HANDLED;
}

static int __init rp2350_timer_init(struct device_node *np)
{
	void __iomem *base;
	void __iomem *ticks;
	struct clock_event_device *ce = this_cpu_ptr(&rp2350_clock_event);
	int rc;

	timer_irq = irq_of_parse_and_map(np, 0);
	if (!timer_irq) {
		pr_err("%pOFP: timer irq not found\n", np);
		return -ENODEV;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%pOFP: could not map SIO registers\n", np);
		return -ENOMEM;
	}

	mtime_lo = base + RP2350_SIO_MTIME_LO;
	mtime_hi = base + RP2350_SIO_MTIME_HI;
	mtimecmp_lo = base + RP2350_SIO_MTIMECMP_LO;
	mtimecmp_hi = base + RP2350_SIO_MTIMECMP_HI;

	/* Make sure the 1 MHz tick is running (the SDK bootloader starts it). */
	ticks = ioremap(RP2350_TICKS_BASE, 0x1000);
	if (!ticks) {
		pr_err("could not map tick generator\n");
		rc = -ENOMEM;
		goto fail_iounmap;
	}
	ticks_ctrl = ticks + RP2350_TICKS_RISCV_CTRL;
	ticks_cycles = ticks + RP2350_TICKS_RISCV_CYCLES;
	if (!(readl_relaxed(ticks_ctrl) & TICKS_RUNNING)) {
		writel_relaxed(RP2350_CLK_REF_HZ / 1000000, ticks_cycles);
		writel_relaxed(TICKS_ENABLE, ticks_ctrl);
	}

	timer_freq = riscv_timebase;
	clint_time_val = (u64 __iomem *)mtime_lo;

	rc = clocksource_register_hz(&rp2350_clocksource, timer_freq);
	if (rc) {
		pr_err("clocksource register failed [%d]\n", rc);
		goto fail_tick_unmap;
	}

	sched_clock_register(rp2350_get_cycles64, 64, timer_freq);

	ce->cpumask = cpumask_of(smp_processor_id());
	rc = request_percpu_irq(timer_irq, rp2350_timer_interrupt,
				"rp2350-timer", &rp2350_clock_event);
	if (rc) {
		pr_err("registering timer irq failed [%d]\n", rc);
		goto fail_tick_unmap;
	}

	enable_percpu_irq(timer_irq, irq_get_trigger_type(timer_irq));
	clockevents_config_and_register(ce, timer_freq, 100, ULONG_MAX);

	pr_info("%pOFP: timer running at %u Hz\n", np, timer_freq);

	return 0;

fail_tick_unmap:
	iounmap(ticks);
fail_iounmap:
	iounmap(base);
	return rc;
}

TIMER_OF_DECLARE(rp2350_timer, "raspberrypi,rp2350-timer", rp2350_timer_init);
