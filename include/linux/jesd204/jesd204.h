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

enum jesd204_state_change_result {
	JESD204_STATE_CHANGE_ERROR = -1,
	JESD204_STATE_CHANGE_STARTED = 0,
	JESD204_STATE_CHANGE_DONE,
};

typedef int (*jesd204_cb)(struct jesd204_dev *jdev);

/**
 * struct jesd204_link - JESD204 link configuration settings
 */
struct jesd204_link {
	u64 sample_rate;

	u8 num_lanes;
	u8 num_converters;
	u8 octets_per_frame;
	u8 frames_per_multiframe;

	u8 bits_per_sample;

	u8 converter_resolution;
	u8 subclass;

	u8 did;
	u8 bid;

	bool scrambling;

	u8 *lane_ids;
};

/**
 * struct jesd204_dev_ops - JESD204 device operations
 */
struct jesd204_dev_ops {
	int (*init_clocks)(struct jesd204_dev *jdev);
	int (*disable_clocks)(struct jesd204_dev *jdev);
	int (*enable_clocks)(struct jesd204_dev *jdev);
	bool (*link_supported)(struct jesd204_dev *jdev,
			       unsigned int link_id,
			       struct jesd204_link *lnk);
	int (*setup_link)(struct jesd204_dev *jdev,
			  unsigned int link_id,
			  struct jesd204_link *lnk);
	int (*disable_link)(struct jesd204_dev *jdev,
			    unsigned int link_id);
	int (*enable_link)(struct jesd204_dev *jdev,
			   unsigned int link_id);
};

/**
 * struct jesd204_dev_data - JESD204 device initialization data
 * @ops			JESD204 operations this device passes to the framework
 */
struct jesd204_dev_data {
	const struct jesd204_dev_ops			*ops;
	const struct jesd204_link			*links;
	unsigned int					num_links;
};

#if IS_ENABLED(CONFIG_JESD204)

struct jesd204_dev *jesd204_dev_register(struct device *dev,
					 const struct jesd204_dev_data *init);
struct jesd204_dev *devm_jesd204_dev_register(struct device *dev,
					      const struct jesd204_dev_data *i);

void jesd204_dev_unregister(struct jesd204_dev *jdev);
void devm_jesd204_unregister(struct device *dev, struct jesd204_dev *jdev);

struct device *jesd204_dev_to_device(struct jesd204_dev *jdev);
struct jesd204_dev *jesd204_dev_from_device(struct device *dev);

#else /* !IS_ENABLED(CONFIG_JESD204) */

static inline struct jesd204_dev *jesd204_dev_register(
		struct device *dev, const struct jesd204_dev_data *init)
{
	return NULL;
}

static inline void jesd204_dev_unregister(struct jesd204_dev *jdev) {}

static inline struct jesd204_dev *devm_jesd204_dev_register(
		struct device *dev, const struct jesd204_dev_data *init)
{
	return NULL;
}

static inline void devm_jesd204_unregister(struct device *dev,
	       struct jesd204_dev *jdev) {}

static inline struct device *jesd204_dev_to_device(struct jesd204_dev *jdev)
{
	return NULL;
}

static inline struct jesd204_dev *jesd204_dev_from_device(struct device *dev)
{
	return NULL;
}

#endif /* IS_ENABLED(CONFIG_JESD204) */

#endif
