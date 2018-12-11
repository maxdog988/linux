/* SPDX-License-Identifier: GPL-2.0
 *
 * Description   : JTAG driver
 *
 * Copyright (C) 2018 NuvoTon Corporation
 *
 */

#ifndef __JTAG_DRV_H__
#define __JTAG_DRV_H__

#define JTAG_SCAN_LEN	256
struct tck_bitbang {
	unsigned char     tms;
	unsigned char     tdi;        // TDI bit value to write
	unsigned char     tdo;        // TDO bit value to read
};

struct scan_xfer {
	unsigned int     length;      // number of bits
	unsigned char    tdi[JTAG_SCAN_LEN];
	unsigned int     tdi_bytes;
	unsigned char    tdo[JTAG_SCAN_LEN];
	unsigned int     tdo_bytes;
	unsigned int     end_tap_state;
};

enum JtagStates {
	JtagTLR,
	JtagRTI,
	JtagSelDR,
	JtagCapDR,
	JtagShfDR,
	JtagEx1DR,
	JtagPauDR,
	JtagEx2DR,
	JtagUpdDR,
	JtagSelIR,
	JtagCapIR,
	JtagShfIR,
	JtagEx1IR,
	JtagPauIR,
	JtagEx2IR,
	JtagUpdIR
};

enum JTAG_PIN {
	pin_TDI,
	pin_TMS,
	pin_TCK,
	pin_TDO,
	pin_NUM,
};


#define JTAGIOC_BASE    'T'

/* ioctl definitions */
#define JTAG_DRIVER_NAME "jtag_drv"

#define JTAG_SIOCFREQ         _IOW(JTAGIOC_BASE, 3, unsigned int)
#define JTAG_GIOCFREQ         _IOR(JTAGIOC_BASE, 4, unsigned int)
#define JTAG_BITBANG          _IOWR(JTAGIOC_BASE, 5, struct tck_bitbang)
#define JTAG_SET_TAPSTATE     _IOW(JTAGIOC_BASE, 6, unsigned int)
#define JTAG_READWRITESCAN    _IOWR(JTAGIOC_BASE, 7, struct scan_xfer)
#define JTAG_SLAVECONTLR      _IOW(JTAGIOC_BASE, 8, unsigned int)
#define JTAG_RUNTEST          _IOW(JTAGIOC_BASE, 9, unsigned int)
#define JTAG_DIRECTGPIO       _IOW(JTAGIOC_BASE, 10, unsigned int)
#define JTAG_PSPI             _IOW(JTAGIOC_BASE, 11, unsigned int)

#endif
