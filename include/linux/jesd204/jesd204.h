/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _JESD204_H_
#define _JESD204_H_

struct jesd204_dev;

#if IS_ENABLED(CONFIG_JESD204)

void jesd204_dev_unregister(struct jesd204_dev *jdev);

#else /* !IS_ENABLED(CONFIG_JESD204) */

static inline void jesd204_dev_unregister(struct jesd204_dev *jdev) {}

#endif /* IS_ENABLED(CONFIG_JESD204) */

#endif
