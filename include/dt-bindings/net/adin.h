/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Device Tree constants for Analog Devices Industrial Ethernet PHYs
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _DT_BINDINGS_ADIN_H
#define _DT_BINDINGS_ADIN_H

/* RGMII internal delay settings for rx and tx for ADIN1300 */
#define ADIN1300_RGMII_1_60_NS		0x1
#define ADIN1300_RGMII_1_80_NS		0x2
#define	ADIN1300_RGMII_2_00_NS		0x0
#define	ADIN1300_RGMII_2_20_NS		0x6
#define	ADIN1300_RGMII_2_40_NS		0x7

/* RMII fifo depth values */
#define ADIN1300_RMII_4_BITS		0x0
#define ADIN1300_RMII_8_BITS		0x1
#define ADIN1300_RMII_12_BITS		0x2
#define ADIN1300_RMII_16_BITS		0x3
#define ADIN1300_RMII_20_BITS		0x4
#define ADIN1300_RMII_24_BITS		0x5

#endif
