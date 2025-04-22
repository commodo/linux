// SPDX-License-Identifier: GPL-2.0
/*
 * Find closest element functions
 *
 * Based on previous util_macros.h implementation
 */

#include <linux/find_closest.h>
#include <linux/module.h>

/**
 * find_closest - locate the closest element in a sorted array
 * @x: The reference value.
 * @a: The array in which to look for the closest element. Must be sorted
 *  in ascending order.
 * @as: Size of 'a'.
 *
 * Returns the index of the element closest to 'x'.
 */
unsigned int find_closest(int x, const int *a, unsigned int as)
{
	unsigned int array_size = as - 1;
	int mid_x, left, right;
	unsigned int i;

	for (i = 0; i < array_size; i++) {
		mid_x = (a[i] + a[i + 1]) / 2;
		if (x <= mid_x) {
			left = x - a[i];
			right = a[i + 1] - x;
			if (right < left)
				i++;
			break;
		}
	}

	return i;
}
EXPORT_SYMBOL_GPL(find_closest);

/**
 * find_closest_descending - locate the closest element in a sorted array
 * @x: The reference value.
 * @a: The array in which to look for the closest element. Must be sorted
 *  in descending order.
 * @as: Size of 'a'.
 *
 * Similar to find_closest() but 'a' is expected to be sorted in descending
 * order. The iteration is done in reverse order, so that the comparison
 * of 'right' & 'left' also works for unsigned numbers.
 */
unsigned int find_closest_descending(int x, const int *a, unsigned int as)
{
	unsigned int array_size = as - 1;
	int mid_x, left, right;
	unsigned int i;

	for (i = array_size; i >= 1; i--) {
		mid_x = (a[i] + a[i - 1]) / 2;
		if (x <= mid_x) {
			left = x - a[i];
			right = a[i - 1] - x;
			if (right < left)
				i--;
			break;
		}
	}

	return i;
}
EXPORT_SYMBOL_GPL(find_closest_descending);
