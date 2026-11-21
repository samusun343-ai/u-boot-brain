#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>

void mxs_set_divcpu(uint8_t clkdiv) {
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	uint32_t clkctrl;
	clkctrl = readl(&clkctrl_regs->hw_clkctrl_cpu);
	clkctrl &= ~0x3F;
	clkctrl |= (clkdiv & 0x3F);
	writel(clkctrl, &clkctrl_regs->hw_clkctrl_cpu);
}

/*
 * Clock setup for boot paths where the SPL never runs.
 *
 * On the EBOOT -> nk.bin path, EBOOT loads U-Boot proper (u-boot.bin)
 * straight into DRAM and jumps to it, so the SPL clock and power
 * initialization (mxs_power_init() / mxs_mem_init()) is skipped
 * entirely.  Nothing else sets the CPU/HBUS/EMI clocks either: the
 * kernel only registers the clock tree and never programs rates.
 * The result is a machine running everything from the 24MHz XTAL.
 *
 * The functions below port the SPL sequences to U-Boot proper so the
 * board early init can establish the datasheet operating state
 * (i.MX28 datasheet, "Recommended Operating Conditions for Specific
 * Clock Targets"): CPU 454.73MHz, AHB 151.57MHz, EMI 205.71MHz at
 * VDDD 1.55V.
 */

/* Delay in microseconds, from the DIGCTL free-running counter.
 * Independent of the CPU clock, like the SPL early_delay(). */
static void sharp_delay_us(uint32_t us)
{
	struct mxs_digctl_regs *digctl_regs =
		(struct mxs_digctl_regs *)MXS_DIGCTL_BASE;
	uint32_t st = readl(&digctl_regs->hw_digctl_microseconds);

	while (readl(&digctl_regs->hw_digctl_microseconds) - st <= us)
		;
}

/* Port of the SPL mxs_get_vddd_power_source_off(): returns 1 if the
 * VDDD rail is currently supplied by the linear regulator, 0 if it
 * is supplied by the DC-DC converter. */
static int sharp_vddd_powered_by_linreg(void)
{
	struct mxs_power_regs *power_regs =
		(struct mxs_power_regs *)MXS_POWER_BASE;
	uint32_t tmp;

	tmp = readl(&power_regs->hw_power_vdddctrl);
	if (tmp & POWER_VDDDCTRL_DISABLE_FET) {
		if ((tmp & POWER_VDDDCTRL_LINREG_OFFSET_MASK) ==
			POWER_VDDDCTRL_LINREG_OFFSET_0STEPS) {
			return 1;
		}
	}

	if (readl(&power_regs->hw_power_sts) & POWER_STS_VDD5V_GT_VDDIO) {
		if (!(readl(&power_regs->hw_power_5vctrl) &
			POWER_5VCTRL_ENABLE_DCDC)) {
			return 1;
		}
	}

	if (!(tmp & POWER_VDDDCTRL_ENABLE_LINREG)) {
		if ((tmp & POWER_VDDDCTRL_LINREG_OFFSET_MASK) ==
			POWER_VDDDCTRL_LINREG_OFFSET_1STEPS_BELOW) {
			return 1;
		}
	}

	return 0;
}

/* Port of the SPL mxs_power_set_vddx() for the VDDD rail only
 * (step 25mV, lowest 800mV, as in the SPL mxs_vddd_cfg).  Ramps the
 * target in steps of at most 100mV, waiting for the DC-DC converter
 * between steps, and programs the brownout offset last. */
static void sharp_set_vddd(uint32_t new_target, uint32_t new_brownout)
{
	struct mxs_power_regs *power_regs =
		(struct mxs_power_regs *)MXS_POWER_BASE;
	uint32_t *reg = &power_regs->hw_power_vdddctrl;
	uint32_t cur_target, diff, bo_int = 0;
	uint32_t powered_by_linreg = 0;
	int adjust_up, tmp;

	new_brownout = DIV_ROUND_CLOSEST(new_target - new_brownout, 25);

	cur_target = readl(reg);
	cur_target &= POWER_VDDDCTRL_TRG_MASK;
	cur_target *= 25;
	cur_target += 800;

	adjust_up = new_target > cur_target;
	powered_by_linreg = sharp_vddd_powered_by_linreg();

	if (adjust_up) {
		if (powered_by_linreg) {
			bo_int = readl(reg);
			clrbits_le32(reg, POWER_CTRL_ENIRQ_VDDD_BO);
		}
		setbits_le32(reg, POWER_VDDDCTRL_BO_OFFSET_MASK);
	}

	do {
		if (abs(new_target - cur_target) > 100) {
			if (adjust_up)
				diff = cur_target + 100;
			else
				diff = cur_target - 100;
		} else {
			diff = new_target;
		}

		diff -= 800;
		diff /= 25;

		clrsetbits_le32(reg, POWER_VDDDCTRL_TRG_MASK, diff);

		if (powered_by_linreg ||
			(readl(&power_regs->hw_power_sts) &
				POWER_STS_VDD5V_GT_VDDIO))
			sharp_delay_us(500);
		else {
			/* Wait for DC_OK like the SPL does, but bounded:
			 * a boot path must never hang on a status bit. */
			struct mxs_digctl_regs *digctl_regs =
				(struct mxs_digctl_regs *)MXS_DIGCTL_BASE;
			uint32_t st = readl(&digctl_regs->hw_digctl_microseconds);

			for (;;) {
				tmp = readl(&power_regs->hw_power_sts);
				if (tmp & POWER_STS_DC_OK)
					break;
				if (readl(&digctl_regs->hw_digctl_microseconds) -
					st > 100000)
					break;
			}
		}

		cur_target = readl(reg);
		cur_target &= POWER_VDDDCTRL_TRG_MASK;
		cur_target *= 25;
		cur_target += 800;
	} while (new_target > cur_target);

	if (adjust_up && powered_by_linreg) {
		writel(POWER_CTRL_VDDD_BO_IRQ, &power_regs->hw_power_ctrl_clr);
		if (bo_int & POWER_CTRL_ENIRQ_VDDD_BO)
			setbits_le32(reg, POWER_CTRL_ENIRQ_VDDD_BO);
	}

	clrsetbits_le32(reg, POWER_VDDDCTRL_BO_OFFSET_MASK,
			new_brownout << POWER_VDDDCTRL_BO_OFFSET_OFFSET);
}

void mxs_setup_vddd(void)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;

	/* Make sure PLL0 is powered (the SPL does the same in
	 * mxs_power_clock2pll() before touching the CPU clock). */
	setbits_le32(&clkctrl_regs->hw_clkctrl_pll0ctrl0,
			CLKCTRL_PLL0CTRL0_POWER);
	sharp_delay_us(100);

	/* Both the 454MHz CPU state and the 205MHz EMI state require
	 * VDDD at 1.55V (brownout at 1.45V) per the i.MX28 datasheet
	 * operating-condition tables, so raise VDDD before any clock
	 * is switched to the PLL path.  Mirrors the SPL ordering:
	 * mxs_power_init() runs before mxs_mem_init(). */
	sharp_set_vddd(1550, 1450);
}

void mxs_setup_cpu_clock(void)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;

	/* Port of the SPL mxs_mem_setup_cpu_and_hbus(). */

	/* Set fractional divider for ref_cpu to 480 * 18 / 19 = 454MHz
	 * and ungate CPU clock */
	writeb(19 & CLKCTRL_FRAC_FRAC_MASK,
		(uint8_t *)&clkctrl_regs->hw_clkctrl_frac0[CLKCTRL_FRAC0_CPU]);

	/* Set CPU bypass */
	writel(CLKCTRL_CLKSEQ_BYPASS_CPU,
		&clkctrl_regs->hw_clkctrl_clkseq_set);

	/* HBUS = 151MHz */
	writel(CLKCTRL_HBUS_DIV_MASK, &clkctrl_regs->hw_clkctrl_hbus_set);
	writel(((~3) << CLKCTRL_HBUS_DIV_OFFSET) & CLKCTRL_HBUS_DIV_MASK,
		&clkctrl_regs->hw_clkctrl_hbus_clr);

	sharp_delay_us(10000);

	/* CPU clock divider = 1 */
	clrsetbits_le32(&clkctrl_regs->hw_clkctrl_cpu,
			CLKCTRL_CPU_DIV_CPU_MASK, 1);

	/* Disable CPU bypass */
	writel(CLKCTRL_CLKSEQ_BYPASS_CPU,
		&clkctrl_regs->hw_clkctrl_clkseq_clr);

	sharp_delay_us(15000);
}

void mxs_setup_emi_clock(void)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
#if defined(CONFIG_MX23)
	/* Fractional divider for ref_emi is 33 ; 480 * 18 / 33 = 266MHz */
	const unsigned char divider = 33;
#elif defined(CONFIG_MX28)
	/* Fractional divider for ref_emi is 21 ; 480 * 18 / 21 = 411MHz */
	const unsigned char divider = 21;
#endif

	/* Port of the SPL mxs_mem_init_clock(). */

	/* Gate EMI clock */
	writeb(CLKCTRL_FRAC_CLKGATE,
		&clkctrl_regs->hw_clkctrl_frac0_set[CLKCTRL_FRAC0_EMI]);

	/* Set fractional divider for ref_emi */
	writeb(CLKCTRL_FRAC_CLKGATE | (divider & CLKCTRL_FRAC_FRAC_MASK),
		&clkctrl_regs->hw_clkctrl_frac0[CLKCTRL_FRAC0_EMI]);

	/* Ungate EMI clock */
	writeb(CLKCTRL_FRAC_CLKGATE,
		&clkctrl_regs->hw_clkctrl_frac0_clr[CLKCTRL_FRAC0_EMI]);

	sharp_delay_us(11000);

	/* Set EMI clock divider for EMI clock to 411 / 2 = 205MHz */
	writel((2 << CLKCTRL_EMI_DIV_EMI_OFFSET) |
		(1 << CLKCTRL_EMI_DIV_XTAL_OFFSET),
		&clkctrl_regs->hw_clkctrl_emi);

	/* Unbypass EMI */
	writel(CLKCTRL_CLKSEQ_BYPASS_EMI,
		&clkctrl_regs->hw_clkctrl_clkseq_clr);

	sharp_delay_us(10000);
}
