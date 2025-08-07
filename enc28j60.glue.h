/* -*- mode: c; tab-width: 8 -*-
 */

#include <stdint.h>

typedef struct {
	void *rx;
	const void *tx;
	uint32_t len;
} spidev_xfer_t;

typedef void (*spidev_t)(spidev_xfer_t *, uint32_t);

extern spidev_t spidev_init(void);

extern void exti_init(void *);
