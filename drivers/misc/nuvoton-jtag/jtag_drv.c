// SPDX-License-Identifier: GPL-2.0
/*
 * Description   : JTAG driver
 *
 * Copyright (C) 2018 NuvoTon Corporation
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/cdev.h>
#include "jtag_drv.h"

#ifdef _JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) \
		pr_debug("%s() " fmt, __func__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

/* GPIO Port Registers */
#define GPnDIN	0x04	/* Data In */
#define GPnDOUT	0x0C	/* Data Out */
#define GPnDOS	0x68	/* Data Out Set */
#define GPnDOC	0x6C	/* Data Out Clear */

#define high			1
#define low				0

/* default jtag speed in MHz */
#define JTAG_PSPI_SPEED		(10 * 1000000)

/* gcr register */
static struct regmap *gcr_regmap;
#define MFSEL1_OFFSET 0x00C
#define MFSEL3_OFFSET 0x064

/* PSPI registers */
void __iomem *pspi_virt_addr;
#define JTAG_PSPI_BASE_ADDR	(pspi_virt_addr)
#define PSPI_DATA		(JTAG_PSPI_BASE_ADDR + 0x00)
#define PSPI_CTL1		(JTAG_PSPI_BASE_ADDR + 0x02)
#define PSPI_STAT		(JTAG_PSPI_BASE_ADDR + 0x04)

#define PSPI_CTL1_SCDV6_0	9
#define PSPI_CTL1_SCIDL		8
#define PSPI_CTL1_SCM		7
#define PSPI_CTL1_EIW		6
#define PSPI_CTL1_EIR		5
#define PSPI_CTL1_SPIEN		0

#define PSPI_STAT_RBF		1
#define PSPI_STAT_BSY		0

#define PSPI_8BIT		8
#define PSPI_16BIT		16
static unsigned char reverse[16] = {
	0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
	0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF
};
#define REVERSE(x)  ((reverse[(x & 0x0f)] << 4) | reverse[(x & 0xf0) >> 4])

static DEFINE_SPINLOCK(jtag_state_lock);
static char *driver_name = JTAG_DRIVER_NAME;

struct gpio_jtag {
	const char *name;
	unsigned int gpio;
	struct gpio_desc *gpiod;
	void __iomem *reg_base;
	int bit_offset;
};
struct gpio_jtag jtags[pin_NUM];

struct jtag_info {
	u8 tapstate;
	bool is_open;
	struct device *dev;
	struct device *dev_p;
	struct cdev *dev_cdevp;
	struct class *jtag_class;
	struct class *ece_class;
	dev_t dev_t;
	u32 freq;
	u32 pspi_index; /* PSPI controller index */
	u8 tms_level;
	bool direct_gpio; /* control gpio by port register */
	bool enable_pspi; /* generate jtag signals by pspi */
	bool pspi_registered;
	u32 apb_clk_src;
};
struct jtag_info *nvt_jtag;

/* this structure represents a TMS cycle, as expressed in a set of bits and
 * a count of bits (note: there are no start->end state transitions that
 * require more than 1 byte of TMS cycles)
 */
struct TmsCycle {
	unsigned char tmsbits;
	unsigned char count;
};

/* this is the complete set TMS cycles for going from any TAP state to
 * any other TAP state, following a “shortest path” rule
 */
const struct TmsCycle _tmsCycleLookup[][16] = {
/*      TLR        RTI       SelDR      CapDR      SDR      */
/*      Ex1DR      PDR       Ex2DR      UpdDR      SelIR    */
/*      CapIR      SIR       Ex1IR      PIR        Ex2IR    */
/*      UpdIR                                               */
/* TLR */
	{
		{0x01, 1}, {0x00, 1}, {0x02, 2}, {0x02, 3}, {0x02, 4},
		{0x0a, 4}, {0x0a, 5}, {0x2a, 6}, {0x1a, 5}, {0x06, 3},
		{0x06, 4}, {0x06, 5}, {0x16, 5}, {0x16, 6}, {0x56, 7},
		{0x36, 6}
	},
/* RTI */
	{
		{0x07, 3}, {0x00, 0}, {0x01, 1}, {0x01, 2}, {0x01, 3},
		{0x05, 3}, {0x05, 4}, {0x15, 5}, {0x0d, 4}, {0x03, 2},
		{0x03, 3}, {0x03, 4}, {0x0b, 4}, {0x0b, 5}, {0x2b, 6},
		{0x1b, 5}
	},
/* SelDR */
	{
		{0x03, 2}, {0x03, 3}, {0x00, 0}, {0x00, 1}, {0x00, 2},
		{0x02, 2}, {0x02, 3}, {0x0a, 4}, {0x06, 3}, {0x01, 1},
		{0x01, 2}, {0x01, 3}, {0x05, 3}, {0x05, 4}, {0x15, 5},
		{0x0d, 4}
	},
/* CapDR */
	{
		{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x00, 0}, {0x00, 1},
		{0x01, 1}, {0x01, 2}, {0x05, 3}, {0x03, 2}, {0x0f, 4},
		{0x0f, 5}, {0x0f, 6}, {0x2f, 6}, {0x2f, 7}, {0xaf, 8},
		{0x6f, 7}
	},
/* SDR */
	{
		{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x00, 0},
		{0x01, 1}, {0x01, 2}, {0x05, 3}, {0x03, 2}, {0x0f, 4},
		{0x0f, 5}, {0x0f, 6}, {0x2f, 6}, {0x2f, 7}, {0xaf, 8},
		{0x6f, 7}
	},
/* Ex1DR */
	{
		{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x02, 3},
		{0x00, 0}, {0x00, 1}, {0x02, 2}, {0x01, 1}, {0x07, 3},
		{0x07, 4}, {0x07, 5}, {0x17, 5}, {0x17, 6}, {0x57, 7},
		{0x37, 6}
	},
/* PDR */
	{
		{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x01, 2},
		{0x05, 3}, {0x00, 1}, {0x01, 1}, {0x03, 2}, {0x0f, 4},
		{0x0f, 5}, {0x0f, 6}, {0x2f, 6}, {0x2f, 7}, {0xaf, 8},
		{0x6f, 7}
	},
/* Ex2DR */
	{
		{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x00, 1},
		{0x02, 2}, {0x02, 3}, {0x00, 0}, {0x01, 1}, {0x07, 3},
		{0x07, 4}, {0x07, 5}, {0x17, 5}, {0x17, 6}, {0x57, 7},
		{0x37, 6}
	},
/* UpdDR */
	{
		{0x07, 3}, {0x00, 1}, {0x01, 1}, {0x01, 2}, {0x01, 3},
		{0x05, 3}, {0x05, 4}, {0x15, 5}, {0x00, 0}, {0x03, 2},
		{0x03, 3}, {0x03, 4}, {0x0b, 4}, {0x0b, 5}, {0x2b, 6},
		{0x1b, 5}
	},
/* SelIR */
	{
		{0x01, 1}, {0x01, 2}, {0x05, 3}, {0x05, 4}, {0x05, 5},
		{0x15, 5}, {0x15, 6}, {0x55, 7}, {0x35, 6}, {0x00, 0},
		{0x00, 1}, {0x00, 2}, {0x02, 2}, {0x02, 3}, {0x0a, 4},
		{0x06, 3}
	},
/* CapIR */
	{
		{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x07, 5},
		{0x17, 5}, {0x17, 6}, {0x57, 7}, {0x37, 6}, {0x0f, 4},
		{0x00, 0}, {0x00, 1}, {0x01, 1}, {0x01, 2}, {0x05, 3},
		{0x03, 2}
	},
/* SIR */
	{
		{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x07, 5},
		{0x17, 5}, {0x17, 6}, {0x57, 7}, {0x37, 6}, {0x0f, 4},
		{0x0f, 5}, {0x00, 0}, {0x01, 1}, {0x01, 2}, {0x05, 3},
		{0x03, 2}
	},
/* Ex1IR */
	{
		{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x03, 4},
		{0x0b, 4}, {0x0b, 5}, {0x2b, 6}, {0x1b, 5}, {0x07, 3},
		{0x07, 4}, {0x02, 3}, {0x00, 0}, {0x00, 1}, {0x02, 2},
		{0x01, 1}
	},
/* PIR */
	{
		{0x1f, 5}, {0x03, 3}, {0x07, 3}, {0x07, 4}, {0x07, 5},
		{0x17, 5}, {0x17, 6}, {0x57, 7}, {0x37, 6}, {0x0f, 4},
		{0x0f, 5}, {0x01, 2}, {0x05, 3}, {0x00, 1}, {0x01, 1},
		{0x03, 2}
	},
/* Ex2IR */
	{
		{0x0f, 4}, {0x01, 2}, {0x03, 2}, {0x03, 3}, {0x03, 4},
		{0x0b, 4}, {0x0b, 5}, {0x2b, 6}, {0x1b, 5}, {0x07, 3},
		{0x07, 4}, {0x00, 1}, {0x02, 2}, {0x02, 3}, {0x00, 0},
		{0x01, 1}
	},
/* UpdIR */
	{
		{0x07, 3}, {0x00, 1}, {0x01, 1}, {0x01, 2}, {0x01, 3},
		{0x05, 3}, {0x05, 4}, {0x15, 5}, {0x0d, 4}, {0x03, 2},
		{0x03, 3}, {0x03, 4}, {0x0b, 4}, {0x0b, 5}, {0x2b, 6},
		{0x00, 0}
	},
};

static void set_gpio(unsigned int pin, int value)
{
	if (pin >= pin_NUM)
		return;
	if (value)
		writel(1 << jtags[pin].bit_offset,
			jtags[pin].reg_base + GPnDOS);
	else
		writel(1 << jtags[pin].bit_offset,
			jtags[pin].reg_base + GPnDOC);
}

static int get_gpio(unsigned int pin)
{
	unsigned int value = 0;

	if (pin >= pin_NUM)
		return 0;
	if (pin == pin_TDO)
		value = readl(jtags[pin].reg_base + GPnDIN);
	else
		value = readl(jtags[pin].reg_base + GPnDOUT);
	return (value & (1 << jtags[pin].bit_offset)) ? 1 : 0;
}

static u8 TCK_Cycle(unsigned char no_tdo, unsigned char TMS,
	unsigned char TDI)
{
	u32 tdo = 0;

	/* IEEE 1149.1
	 * TMS & TDI shall be sampled by the test logic on the rising edge
	 * test logic shall change TDO on the falling edge
	 */
	if (nvt_jtag->direct_gpio) {
		set_gpio(pin_TDI, (int)TDI);
		if (nvt_jtag->tms_level != (int)TMS) {
			set_gpio(pin_TMS, (int)TMS);
			nvt_jtag->tms_level = (int)TMS;
		}
		set_gpio(pin_TCK, (int)high);
		if (!no_tdo)
			tdo = get_gpio(pin_TDO);
		set_gpio(pin_TCK, (int)low);
	} else {
		gpiod_set_value(jtags[pin_TDI].gpiod, (int)TDI);
		if (nvt_jtag->tms_level != (int)TMS) {
			gpiod_set_value(jtags[pin_TMS].gpiod, (int)TMS);
			nvt_jtag->tms_level = (int)TMS;
		}
		gpiod_set_value(jtags[pin_TCK].gpiod, (int)high);
		if (!no_tdo)
			tdo = gpiod_get_value(jtags[pin_TDO].gpiod);
		gpiod_set_value(jtags[pin_TCK].gpiod, (int)low);
	}
	return tdo;
}

void nvt_jtag_bitbang(struct tck_bitbang *bitbang)
{
	bitbang->tdo = TCK_Cycle(0, bitbang->tms, bitbang->tdi);
}

static int nvt_jtag_set_tapstate(struct jtag_info *nvt_jtag,
	unsigned int tapstate)
{
	unsigned char i;
	unsigned char tmsbits;
	unsigned char count;

	/* ensure that the requested tap states are
	 * within 0 to 15.
	 */
	if (tapstate > 15)
		return -1;

	if (tapstate == JtagTLR) {
		for (i = 0; i < 9; i++)
			TCK_Cycle(1, 1, 0);
		nvt_jtag->tapstate = JtagTLR;
		return 0;
	}

	tmsbits = _tmsCycleLookup[nvt_jtag->tapstate][tapstate].tmsbits;
	count   = _tmsCycleLookup[nvt_jtag->tapstate][tapstate].count;

	if (count == 0)
		return 0;

	for (i = 0; i < count; i++) {
		TCK_Cycle(1, (tmsbits & 1), 0);
		tmsbits >>= 1;
	}
	JTAG_DBUG("jtag: change state %d -> %d\n",
		nvt_jtag->tapstate, tapstate);
	nvt_jtag->tapstate = tapstate;
	return 0;
}

static void jtag_switch_pspi(int enable)
{
	int divisor;

	if (enable) {
		divisor = (nvt_jtag->apb_clk_src / (2 * nvt_jtag->freq)) - 1;
		if (divisor <= 0) {
			pr_err("Requested PSPI frequency is too large.\n");
			return;
		}

		/* disable */
		writew(readw(PSPI_CTL1) & ~(0x1 << PSPI_CTL1_SPIEN), PSPI_CTL1);

		/* configure pin function to be pspi */
		regmap_update_bits(gcr_regmap, MFSEL3_OFFSET, (0x3 << 3),
				(0x2 << 3));

		/* configure Shift Clock Divider value */
		writew((readw(PSPI_CTL1) & ~(0x7f << PSPI_CTL1_SCDV6_0)) |
				(divisor << PSPI_CTL1_SCDV6_0),
				PSPI_CTL1);

		/* configure TCK to be low when idle */
		writew(readw(PSPI_CTL1) &
				~(0x1 << PSPI_CTL1_SCIDL),
				PSPI_CTL1);

		/* TDI is shifted out on the falling edge,
		 * TDO is sampled on the rising edge
		 */
		writew(readw(PSPI_CTL1) &
				~(0x1 << PSPI_CTL1_SCM),
				PSPI_CTL1);

		/* set 16 bit mode and enable pspi */
		writew(readw(PSPI_CTL1) | (0x1 << PSPI_CTL1_SPIEN) | (1 << 2),
				PSPI_CTL1);

		while (readb(PSPI_STAT) & (0x1 << PSPI_STAT_RBF))
			readb(PSPI_STAT);
	} else {
		writew(readw(PSPI_CTL1) & ~(0x1 << PSPI_CTL1_SPIEN), PSPI_CTL1);
		regmap_update_bits(gcr_regmap, MFSEL3_OFFSET, (0x3 << 3),
				(0x0 << 3));

		if (nvt_jtag->direct_gpio)
			nvt_jtag->tms_level = get_gpio(pin_TMS);
		else
			nvt_jtag->tms_level = gpiod_get_value(jtags[pin_TMS].gpiod);
	}
}

void nvt_jtag_readwrite_scan(struct jtag_info *nvt_jtag,
	struct scan_xfer *scan_xfer)
{
	unsigned int  bit_index = 0;
	unsigned char *tdi_p = scan_xfer->tdi;
	unsigned char *tdo_p = scan_xfer->tdo;
	int remain_bits = scan_xfer->length;
	unsigned int pspi = 0, gpio = 0;

	if ((nvt_jtag->tapstate != JtagShfDR) &&
		(nvt_jtag->tapstate != JtagShfIR)) {
		pr_err("readwrite_scan bad current tapstate = %d\n",
				nvt_jtag->tapstate);
		return;
	}
	if (scan_xfer->length == 0) {
		pr_err("readwrite_scan bad length 0\n");
		return;
	}

	if (scan_xfer->tdi == NULL && scan_xfer->tdi_bytes != 0) {
		pr_err("readwrite_scan null tdi with nonzero length %u!\n",
			scan_xfer->tdi_bytes);
		return;
	}

	if (scan_xfer->tdo == NULL && scan_xfer->tdo_bytes != 0) {
		pr_err("readwrite_scan null tdo with nonzero length %u!\n",
			scan_xfer->tdo_bytes);
		return;
	}

	if (nvt_jtag->enable_pspi && (remain_bits > PSPI_16BIT)) {
		jtag_switch_pspi(1);
		pspi = 1;
	}

	while (bit_index < scan_xfer->length) {
		int bit_offset = (bit_index % 8);
		int this_input_bit = 0;
		int tms_high_or_low;
		int this_output_bit;
		unsigned short tdo;

		/* last 16 bits or less are transmitted using gpio bitbang */
		if (!nvt_jtag->enable_pspi || (remain_bits < PSPI_16BIT) ||
			((remain_bits == PSPI_16BIT) &&
			(scan_xfer->end_tap_state != JtagShfDR)))
			gpio = 1;
		else
			gpio = 0;

		if (gpio) {
			/* transmit using gpio bitbang */
			if (pspi) {
				jtag_switch_pspi(0);
				pspi = 0;
			}
			if (bit_index / 8 < scan_xfer->tdi_bytes)
				this_input_bit = (*tdi_p >> bit_offset) & 1;

			/* If this is the last bit, leave TMS high */
			tms_high_or_low = (bit_index == scan_xfer->length - 1) &&
				(scan_xfer->end_tap_state != JtagShfDR) &&
				(scan_xfer->end_tap_state != JtagShfIR);
			this_output_bit = TCK_Cycle(0, tms_high_or_low, this_input_bit);
			/* If it was the last bit in the scan and the end_tap_state is
			 * something other than shiftDR or shiftIR then go to Exit1.
			 * IMPORTANT Note: if the end_tap_state is ShiftIR/DR and the
			 * next call to this function is a shiftDR/IR then the driver
			 * will not change state!
			 */
			if (tms_high_or_low) {
				nvt_jtag->tapstate = (nvt_jtag->tapstate == JtagShfDR) ?
					JtagEx1DR : JtagEx1IR;
			}
			if (bit_index / 8 < scan_xfer->tdo_bytes) {
				if (bit_index % 8 == 0) {
					/* Zero the output buffer before writing data */
					*tdo_p = 0;
				}
				*tdo_p |= this_output_bit << bit_offset;
			}
			/* reach byte boundary, approach to next byte */
			if (bit_offset == 7) {
				tdo_p++;
				tdi_p++;
			}
			bit_index++;
		} else {
			/* transmit using pspi */
			/* PSPI is 16 bit transfer mode */
			while (readb(PSPI_STAT) & (0x1 << PSPI_STAT_BSY))
				;

			if (((bit_index / 8) + 1) < scan_xfer->tdi_bytes)
				writew(REVERSE(*tdi_p) << 8 | REVERSE(*(tdi_p+1)),
					PSPI_DATA);
			else
				writew(0x0, PSPI_DATA);

			while (readb(PSPI_STAT) & (0x1 << PSPI_STAT_BSY))
				;

			while (!(readb(PSPI_STAT) & (0x1 << PSPI_STAT_RBF)))
				;

			tdo = readw(PSPI_DATA);
			if ((bit_index / 8) + 1 < scan_xfer->tdo_bytes) {
				*tdo_p = REVERSE((tdo >> 8) & 0xff);
				*(tdo_p + 1) = REVERSE(tdo & 0xff);
			}

			bit_index += 16;
			remain_bits -= 16;
			tdo_p += 2;
			tdi_p += 2;
		}
	}
	nvt_jtag_set_tapstate(nvt_jtag, scan_xfer->end_tap_state);
}

/* Run in current state for specific number of cycles */
void nvt_jtag_runtest(unsigned int run_cycles)
{
	int i;
	int cycles = run_cycles  / 16;
	int remain_bits = run_cycles % 16;

	if (!nvt_jtag->enable_pspi) {
		for (i = 0; i < run_cycles; i++)
			TCK_Cycle(0, 0, 0);
		return;
	}
	if (cycles > 0) {
		jtag_switch_pspi(1);
		for (i = 0; i < cycles; i++) {
			while (readb(PSPI_STAT) & (0x1 << PSPI_STAT_BSY))
				;

			writew(0x0, PSPI_DATA);

			while (readb(PSPI_STAT) & (0x1 << PSPI_STAT_BSY))
				;
			while (!(readb(PSPI_STAT) & (0x1 << PSPI_STAT_RBF)))
				;
			readw(PSPI_DATA);
		}
		jtag_switch_pspi(0);
	}
	if (remain_bits) {
		for (i = 0; i < remain_bits; i++)
			TCK_Cycle(0, 0, 0);
	}
}
/* Set All output GPIOs as inputs */
static inline void nvt_jtag_slave(void)
{
	regmap_update_bits(gcr_regmap, MFSEL1_OFFSET, (0x1 << 14),
			(0x1 << 14));
	regmap_update_bits(gcr_regmap, MFSEL3_OFFSET, (0x3 << 3),
			(0x0 << 3));
}


/* Set All output GPIOs as outputs */
static inline void nvt_jtag_master(void)
{
	int count;

	regmap_update_bits(gcr_regmap, MFSEL1_OFFSET, (0x1 << 14),
			(0x0 << 14));
	regmap_update_bits(gcr_regmap, MFSEL3_OFFSET, (0x3 << 3),
			(0x0 << 3));

	for (count = 0; count < pin_NUM; count++) {
		if (count == pin_TDO)
			gpiod_direction_input(jtags[count].gpiod);
		else if (count == pin_TCK)
			gpiod_direction_output(jtags[count].gpiod, low);
		else
			gpiod_direction_output(jtags[count].gpiod, high);
	}
	if (nvt_jtag->direct_gpio)
		nvt_jtag->tms_level = get_gpio(pin_TMS);
	else
		nvt_jtag->tms_level = gpiod_get_value(jtags[pin_TMS].gpiod);

	nvt_jtag_set_tapstate(nvt_jtag, JtagTLR);
}

static long jtag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct jtag_info *nvt_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct tck_bitbang bitbang;
	struct scan_xfer scan_xfer;

	switch (cmd) {
	case JTAG_SIOCFREQ:
		nvt_jtag->freq = arg;
		break;
	case JTAG_GIOCFREQ:
		ret = nvt_jtag->freq;
		break;
	case JTAG_BITBANG:
		if (copy_from_user(&bitbang, argp, sizeof(struct tck_bitbang)))
			ret = -EFAULT;
		else
			nvt_jtag_bitbang(&bitbang);
		if (copy_to_user(argp, &bitbang, sizeof(struct tck_bitbang)))
			ret = -EFAULT;
		break;
	case JTAG_SET_TAPSTATE:
		nvt_jtag_set_tapstate(nvt_jtag, (unsigned int)arg);
		break;
	case JTAG_RUNTEST:
		nvt_jtag_runtest((unsigned int)arg);
		break;
	case JTAG_READWRITESCAN:
		if (copy_from_user(&scan_xfer, argp, sizeof(struct scan_xfer)))
			ret = -EFAULT;
		else
			nvt_jtag_readwrite_scan(nvt_jtag, &scan_xfer);
		if (copy_to_user(argp, &scan_xfer, sizeof(struct scan_xfer)))
			ret = -EFAULT;
		break;
	case JTAG_SLAVECONTLR:
		if (arg)
			nvt_jtag_slave();
		else
			nvt_jtag_master();
		break;
	case JTAG_DIRECTGPIO:
		if (arg)
			nvt_jtag->direct_gpio = true;
		else
			nvt_jtag->direct_gpio = false;
		break;
	case JTAG_PSPI:
		if (arg)
			nvt_jtag->enable_pspi = true;
		else
			nvt_jtag->enable_pspi = false;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static int jtag_open(struct inode *inode, struct file *file)
{
	spin_lock(&jtag_state_lock);
	if (nvt_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	nvt_jtag->is_open = true;
	nvt_jtag->freq = JTAG_PSPI_SPEED;
	file->private_data = nvt_jtag;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static int jtag_release(struct inode *inode, struct file *file)
{
	struct jtag_info *drvdata = file->private_data;

	spin_lock(&jtag_state_lock);

	drvdata->is_open = false;

	spin_unlock(&jtag_state_lock);

	return 0;
}

const struct file_operations nvt_jtag_fops = {
	.open              = jtag_open,
	.unlocked_ioctl    = jtag_ioctl,
	.release           = jtag_release,
};

static int jtag_pspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct resource res;
	struct clk *apb_clk;
	int ret = 0;

	regmap_update_bits(gcr_regmap, MFSEL3_OFFSET, (0x3 << 3),
			(0x0 << 3));

	np = dev->of_node;
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		pr_err("of_address_to_resource fail ret %d\n", ret);
		return -EINVAL;
	}

	pspi_virt_addr = ioremap(res.start, resource_size(&res));

	if (!pspi_virt_addr) {
		pr_info("pspi_virt_addr fail\n");
		return -ENOMEM;
	}
	pspi_virt_addr = pspi_virt_addr + (nvt_jtag->pspi_index - 1) * 0x1000;

	apb_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(apb_clk)) {
		pr_err("JTAG PSPI probe failed: can't read clk.\n");
		return -ENODEV;
	}
	clk_prepare_enable(apb_clk);

	nvt_jtag->apb_clk_src = clk_get_rate(apb_clk);

	return 0;
}

static const struct of_device_id of_jtag_pspi_match[] = {
	{ .compatible = "nuvoton,npcm750-pspi"},
	{},
};

static struct platform_driver pspi_driver = {
	.probe		= jtag_pspi_probe,
	.driver		= {
		.name	= "jtag-pspi",
		.of_match_table = of_jtag_pspi_match,
	},
};

static int nvt_jtag_device_create(struct jtag_info *jtag)
{
	int ret;
	dev_t dev;
	struct cdev *dev_cdevp = jtag->dev_cdevp;

	ret = alloc_chrdev_region(&dev, 0, 1, driver_name);
	if (ret < 0) {
		pr_err("alloc_chrdev_region() failed for jtag\n");
		goto err;
	}

	dev_cdevp = kmalloc(sizeof(*dev_cdevp), GFP_KERNEL);
	if (!dev_cdevp)
		goto err;

	cdev_init(dev_cdevp, &nvt_jtag_fops);
	dev_cdevp->owner = THIS_MODULE;
	jtag->dev_t = dev;
	ret = cdev_add(dev_cdevp, MKDEV(MAJOR(dev),  MINOR(dev)), 1);
	if (ret < 0) {
		pr_err("add chr dev failed\n");
		goto err;
	}

	jtag->jtag_class = class_create(THIS_MODULE, driver_name);
	if (IS_ERR(jtag->jtag_class)) {
		ret = PTR_ERR(jtag->jtag_class);
		pr_err("Unable to create jtag class; errno = %d\n", ret);
		jtag->jtag_class = NULL;
		goto err;
	}

	jtag->dev = device_create(jtag->jtag_class, jtag->dev_p,
				 MKDEV(MAJOR(dev),  MINOR(dev)),
				 jtag, driver_name);
	if (IS_ERR(jtag->dev)) {
		pr_err("Unable to create device for jtag; errno = %ld\n",
		       PTR_ERR(jtag->dev));
		jtag->dev = NULL;
		goto err;
	}
	return 0;

err:
	if (!dev_cdevp)
		kfree(dev_cdevp);
	return ret;
}


static int nvt_jtag_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	int count, i = 0, ret;
	u32 value;
	void __iomem *gpio_base;
	u32 baseadr[2];
	u32 offset;

	nvt_jtag = kzalloc(sizeof(struct jtag_info), GFP_KERNEL);
	if (!nvt_jtag)
		return -ENOMEM;

	gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
	if (IS_ERR(gcr_regmap)) {
		dev_err(&pdev->dev, "can't find npcm750-gcr\n");
		ret = PTR_ERR(gcr_regmap);
		goto err;
	}

	/* check if PSPI-jtag is enabled */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "enable_pspi_jtag", &value);
	if (ret < 0)
		nvt_jtag->enable_pspi = false;
	else
		nvt_jtag->enable_pspi = value ? true : false;

	if (nvt_jtag->enable_pspi) {
		value = 1;
		ret = of_property_read_u32(pdev->dev.of_node,
				"pspi-index", &value);
		if (ret < 0 || value < 1 || value > 2)
			dev_err(&pdev->dev,
					"Could not read pspi index\n");
		nvt_jtag->pspi_index = value;
	}

	count = device_get_child_node_count(dev);
	if (!count) {
		ret = -ENODEV;
		goto err;
	}

	device_for_each_child_node(dev, child) {
		const char *name = NULL;
		struct gpio_desc *gpiod;

		ret = fwnode_property_read_string(child, "label", &name);
		if (!name) {
			fwnode_handle_put(child);
			ret = -EINVAL;
			goto err;
		}

		ret = fwnode_property_read_u32_array(child,
				"reg", &baseadr[0], 2);
		if (ret != 0) {
			dev_err(&pdev->dev, "No GPIO port base address\n");
			ret = -ENXIO;
			goto err;
		}
		gpio_base = ioremap(baseadr[0], baseadr[1]);
		if (!gpio_base) {
			dev_err(&pdev->dev, "failed to remap I/O memory\n");
			ret = -ENXIO;
			goto err;
		}
		ret = fwnode_property_read_u32_array(child,
				"offset", &offset, 1);
		if (ret != 0) {
			dev_err(&pdev->dev, "No GPIO bit offset\n");
			ret = -EINVAL;
			goto err;
		}
		gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL, child,
							GPIOD_ASIS, name);
		if (IS_ERR(gpiod)) {
			fwnode_handle_put(child);
			dev_err(dev, "no JTAG GPIO\n");
			ret = -ENXIO;
			goto err;
		}

		if (!strcmp(name, "tdi"))
			i = pin_TDI;
		else if (!strcmp(name, "tms"))
			i = pin_TMS;
		else if (!strcmp(name, "tdo"))
			i = pin_TDO;
		else if (!strcmp(name, "tck"))
			i = pin_TCK;
		else {
			ret = -EINVAL;
			goto err;
		}

		jtags[i].bit_offset = offset;
		jtags[i].reg_base = gpio_base;
		jtags[i].name = name;
		jtags[i].gpiod = gpiod;
	}
	nvt_jtag->direct_gpio = true;

	ret = nvt_jtag_device_create(nvt_jtag);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to create device\n",
			__func__);
		goto err;
	}
	nvt_jtag_slave();

	if (nvt_jtag->enable_pspi) {
		ret = platform_driver_register(&pspi_driver);
		if (ret) {
			dev_err(&pdev->dev, "Failed to init pspi_driver.\n");
			goto err;
		}
		nvt_jtag->pspi_registered = true;
	}
	platform_set_drvdata(pdev, nvt_jtag);

	return 0;
err:
	kfree(nvt_jtag);
	return ret;
}

static int nvt_jtag_remove(struct platform_device *pdev)
{
	struct jtag_info *jtag = platform_get_drvdata(pdev);

	if (!jtag)
		return 0;
	if (jtag->pspi_registered)
		platform_driver_unregister(&pspi_driver);
	device_destroy(jtag->jtag_class, jtag->dev_t);
	nvt_jtag_slave();
	kfree(jtag);

	return 0;
}


static const struct of_device_id nvt_jtag_id[] = {
	{ .compatible = "nuvoton,npcm750-jtag", },
	{},
};
MODULE_DEVICE_TABLE(of, nvt_jtag_id);

static struct platform_driver nvt_jtag_driver = {
	.probe          = nvt_jtag_probe,
	.remove			= nvt_jtag_remove,
	.driver         = {
		.name   = "jtag-master",
		.owner	= THIS_MODULE,
		.of_match_table = nvt_jtag_id,
	},
};

module_platform_driver(nvt_jtag_driver);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION("JTAG Master Driver");
MODULE_LICENSE("GPL");

