/* -*- mode: c; tab-width: 8 -*-
 */

#include <stdint.h>

typedef struct {
	void *rx;
	const void *tx;
	int len;
} spidev_xfer_t;

typedef void (*spidev_t)(const spidev_xfer_t *, int);

extern spidev_t spidev_init(void);

extern void exti_init(void *);
