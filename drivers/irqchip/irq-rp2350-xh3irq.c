// SPDX-License-Identifier: GPL-2.0
/*
 * RP2350 Xh3irq external interrupt controller
 *
 * Xh3irq is a Hazard3 custom interrupt controller (not a standard PLIC).
 * It multiplexes up to 52 system-level interrupt lines onto MIP.MEIP and
 * implements the interrupt enable/pending/priority arrays as CSR windows:
 *
 *   MEIEA 0xbe0  external interrupt enable array
 *   MEIPA 0xbe1  external interrupt pending array (read-only)
 *   MEIFA 0xbe2  external interrupt force array (software pending)
 *   MEIPRA 0xbe3  external interrupt priority array (4 bits per IRQ)
 *   MEINEXT 0xbe4 next interrupt to handle (IRQ number << 2), MSB=no IRQ
 *   MEICONTEXT 0xbe5 pre-emption priority / current IRQ context
 *
 * Array CSR idiom: low 5 bits of the written value select a 16-bit window
 * (index = irq/16), upper 16 bits are the data (bit = irq%16).  The
 * dispatch loop reads MEINEXT with csrrs (read + set UPDATE in one
 * instruction); the hardware then updates MEICONTEXT from the sampled IRQ.
 *
 * This driver keeps all IRQs at the same hardware priority (no nesting);
 * pre-emption priority management is a later extension.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <asm/csr.h>

#define XH3IRQ_NR_IRQS	52

#define RVCSR_MEIEA	0xbe0
#define RVCSR_MEINEXT	0xbe4

#define MEINEXT_NOIRQ		BIT(31)
#define MEINEXT_IRQ_MASK	0x7fc
#define MEINEXT_UPDATE		BIT(0)

static struct irq_domain *xh3irq_domain;

static void xh3irq_irq_mask(struct irq_data *d)
{
	/* Array CSR idiom: window = irq/16, data bit = irq%16 in upper half. */
	csr_clear(RVCSR_MEIEA, (d->hwirq / 16) | (BIT(d->hwirq % 16) << 16));
}

static void xh3irq_irq_unmask(struct irq_data *d)
{
	csr_set(RVCSR_MEIEA, (d->hwirq / 16) | (BIT(d->hwirq % 16) << 16));
}

static struct irq_chip xh3irq_chip = {
	.name		= "RP2350-Xh3irq",
	.irq_mask	= xh3irq_irq_mask,
	.irq_unmask	= xh3irq_irq_unmask,
};

static void xh3irq_handle_irq(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	/* Sample the highest-priority pending IRQ and let the hardware
	 * update MEICONTEXT in the same instruction (csrrs).  MSB set
	 * means no eligible IRQ remains at this priority level. */
	while (1) {
		u32 next = csr_read_set(RVCSR_MEINEXT, MEINEXT_UPDATE);
		struct irq_desc *d;
		unsigned int virq;
		unsigned int irq;

		if (next & MEINEXT_NOIRQ)
			break;

		irq = (next & MEINEXT_IRQ_MASK) >> 2;

		/*
		 * Level-triggered sources stay pending in MEIPA (and thus in
		 * MEINEXT) until the handler clears them at the source.  If no
		 * handler is registered, dispatch would loop forever, so mask
		 * such IRQs instead.  (e.g. USBCTRL_IRQ=14 asserts at reset on
		 * RP2350 and is not owned by any driver yet.)
		 */
		virq = irq_find_mapping(xh3irq_domain, irq);
		d = virq ? irq_to_desc(virq) : NULL;
		if (!d || !d->action) {
			csr_clear(RVCSR_MEIEA,
				  (irq / 16) | (BIT(irq % 16) << 16));
			pr_warn_ratelimited("xh3irq: masking unhandled irq %u\n",
					    irq);
			continue;
		}

		generic_handle_domain_irq(xh3irq_domain, irq);
	}

	chained_irq_exit(chip, desc);
}

static int xh3irq_domain_map(struct irq_domain *d, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &xh3irq_chip, handle_simple_irq);
	return 0;
}

static const struct irq_domain_ops xh3irq_domain_ops = {
	.map	= xh3irq_domain_map,
	.xlate	= irq_domain_xlate_onecell,
};

static int __init xh3irq_init(struct device_node *node,
			      struct device_node *parent)
{
	unsigned int parent_irq;
	int i;

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq) {
		pr_err("%pOFP: no parent IRQ (MEIP) found\n", node);
		return -ENODEV;
	}

	xh3irq_domain = irq_domain_add_linear(node, XH3IRQ_NR_IRQS,
					      &xh3irq_domain_ops, NULL);
	if (!xh3irq_domain) {
		pr_err("%pOFP: failed to allocate irq domain\n", node);
		return -ENOMEM;
	}

	/*
	 * Start with every external IRQ masked.  On RP2350 the MEIEA array
	 * is not zero at reset (e.g. IRQ 14 USBCTRL and IRQ 3 TIMER0 are
	 * found enabled), which makes the controller storm on lines whose
	 * sources assert at reset.  Drivers enable what they need later via
	 * request_irq()/irq_unmask().
	 */
	for (i = 0; i < (XH3IRQ_NR_IRQS + 15) / 16; i++)
		csr_write(RVCSR_MEIEA, i);

	irq_set_chained_handler_and_data(parent_irq, xh3irq_handle_irq, NULL);

	pr_info("%pOFP: %d external interrupts mapped\n", node,
		XH3IRQ_NR_IRQS);

	return 0;
}

IRQCHIP_DECLARE(rp2350_xh3irq, "raspberrypi,rp2350-xh3irq", xh3irq_init);
