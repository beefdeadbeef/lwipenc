/* -*- mode: c; tab-width: 8 -*-
 */

#include <stdint.h>

typedef enum {
	XFER_CONT = (1 << 0)
} spidev_xfer_flags_t;

typedef struct {
	void *rx;
	const void *tx;
	uint16_t len;
	uint16_t flags;
} spidev_xfer_t;

typedef void (*spidev_t)(const spidev_xfer_t *, int);

extern spidev_t spidev_init(void);

extern void exti_init(void *);
