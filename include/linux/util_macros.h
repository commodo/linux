/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_HELPER_MACROS_H_
#define _LINUX_HELPER_MACROS_H_

#include <linux/math.h>
#include <linux/find_closest.h>

/**
 * is_insidevar - check if the @ptr points inside the @var memory range.
 * @ptr:	the pointer to a memory address.
 * @var:	the variable which address and size identify the memory range.
 *
 * Evaluates to true if the address in @ptr lies within the memory
 * range allocated to @var.
 */
#define is_insidevar(ptr, var)						\
	((uintptr_t)(ptr) >= (uintptr_t)(var) &&			\
	 (uintptr_t)(ptr) <  (uintptr_t)(var) + sizeof(var))

#endif
