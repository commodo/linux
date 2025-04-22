/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Find closest element functions
 */
#ifndef _LINUX_FIND_CLOSEST_H_
#define _LINUX_FIND_CLOSEST_H_

#include <linux/types.h>

unsigned int find_closest(int x, const int *a, unsigned int as);
unsigned int find_closest_descending(int x, const int *a, unsigned int as);

#endif /* _LINUX_FIND_CLOSEST_H_ */
