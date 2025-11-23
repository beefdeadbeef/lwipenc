/* -*- mode: c; tab-width: 8 -*-
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>

#include <bugurt.h>
#include <native.h>

#include "enc28j60.glue.h"

#define	EXTI_REQ		EXTI5
#define	EXTI_PORT		GPIOB
#define	EXTI_GPIO		GPIO5
#define	EXTI_IRQ		NVIC_EXTI9_5_IRQ
#define	EXTI_ISR		exti9_5_isr

#define	EXTI_IRQ_PRIO		((BGRT_CONFIG_CRITSEC_PRIO + 2) <<4)

#define	SPI_DEV			SPI4
#define	SPI_DEV_GMUX		AFIO_GMUX_SPI4_B6
#define	SPI_DEV_CLK		RCC_SPI4
#define	SPI_DEV_RST		RST_SPI4
#define	SPI_DMA			DMA2

#define	SPI_GPIO_PORT		GPIOB
#define	SPI_GPIO_SCK		GPIO4
#define	SPI_GPIO_NSS		GPIO6
#define	SPI_GPIO_SCKIN		GPIO7
#define	SPI_GPIO_MISO		GPIO8
#define	SPI_GPIO_MOSI		GPIO9

#define	SPI_CLK_FREQ		20000000u
#define	SPI_CLK_PERIOD		(2 * rcc_apb1_frequency / SPI_CLK_FREQ)

#define	SPI_TIM_MASTER		TIM4
#define	SPI_TIM_MASTER_GMUX	AFIO_MUX_TIM4_B6
#define	SPI_TIM_MASTER_CLK	RCC_TIM4
#define	SPI_TIM_MASTER_IC	TIM_IC2			/* B7 <= B4 */

#define	SPI_TIM_SLAVE		TIM3
#define	SPI_TIM_SLAVE_GMUX	AFIO_GMUX_TIM3_B4
#define	SPI_TIM_SLAVE_CLK	RCC_TIM3
#define	SPI_TIM_SLAVE_OC	TIM_OC1			/* B4 => B7 */
#define	SPI_TIM_SLAVE_TRIGGER	TIM_SMCR_TS_ITR3

#define	SPI_DMA_RX_CH		DMA_CHANNEL1
#define	SPI_DMA_RX_REQ		DMA_REQ_SPI4_RX
#define	SPI_DMA_RX_IRQ		NVIC_DMA2_CHANNEL1_IRQ
#define	SPI_DMA_RX_ISR		dma2_channel1_isr

#define	SPI_DMA_TX_CH		DMA_CHANNEL2
#define	SPI_DMA_TX_REQ		DMA_REQ_SPI4_TX
#define	SPI_DMA_TX_IRQ		NVIC_DMA2_CHANNEL2_IRQ
#define	SPI_DMA_TX_ISR		dma2_channel2_isr

#define	SPI_DMA_IRQ_PRIO	((BGRT_CONFIG_CRITSEC_PRIO + 1) <<4)

#define	SPIDEV_LOOPBACK_TEST	0

static bgrt_sem_t extint_sem;
static bgrt_vint_t extint_vint;

static bgrt_sem_t spidev_sem;
static bgrt_vint_t spidev_vint;

static const spidev_xfer_t *current;
static volatile int nxfers;

static void spidev_xfer_setup(const spidev_xfer_t *);

BGRT_ISR(EXTI_ISR)
{
	exti_reset_request(EXTI_REQ);
	nvic_disable_irq(EXTI_IRQ);
	bgrt_vint_push(&extint_vint, &bgrt_kernel.kblock.vic);
}

BGRT_ISR(SPI_DMA_RX_ISR)
{
	dma_clear_interrupt_flags(SPI_DMA, SPI_DMA_RX_CH, DMA_TCIF);
	dma_disable_channel(SPI_DMA, SPI_DMA_RX_CH);
	if (--nxfers) {
		current++;
		spidev_xfer_setup(current);
		timer_enable_counter(SPI_TIM_MASTER);
	} else {
		gpio_set(SPI_GPIO_PORT, SPI_GPIO_NSS);
		bgrt_vint_push(&spidev_vint, &bgrt_kernel.kblock.vic);
	}
}

void SPI_DMA_TX_ISR()
{
	dma_clear_interrupt_flags(SPI_DMA, SPI_DMA_TX_CH, DMA_TCIF);
	dma_disable_channel(SPI_DMA, SPI_DMA_TX_CH);
}

static void spidev_signal(void *ctx)
{
	bgrt_sem_t *sem = ctx;

	bgrt_sem_free_cs(sem);
}

/*
 *
 */
static void exti_ll_init()
{
	gpio_set_mode(EXTI_PORT, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      EXTI_GPIO);

	exti_select_source(EXTI_REQ, EXTI_PORT);
	exti_set_trigger(EXTI_REQ, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI_REQ);

	nvic_set_priority(EXTI_IRQ, EXTI_IRQ_PRIO);
	nvic_enable_irq(EXTI_IRQ);
}

static void spidev_ll_init()
{
	rcc_periph_clock_enable(SPI_DEV_CLK);
	rcc_periph_clock_enable(SPI_TIM_MASTER_CLK);
	rcc_periph_clock_enable(SPI_TIM_SLAVE_CLK);

	gpio_set_mux(SPI_DEV_GMUX);
	gpio_set_mux(SPI_TIM_MASTER_GMUX);
	gpio_set_mux(SPI_TIM_SLAVE_GMUX);

	dma_channel_reset(SPI_DMA, SPI_DMA_RX_CH);
	dma_set_channel_request(SPI_DMA, SPI_DMA_RX_CH, SPI_DMA_RX_REQ);
	dma_set_priority(SPI_DMA, SPI_DMA_RX_CH, DMA_CCR_PL_MEDIUM);
	dma_set_read_from_peripheral(SPI_DMA, SPI_DMA_RX_CH);
	dma_set_memory_size(SPI_DMA, SPI_DMA_RX_CH, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(SPI_DMA, SPI_DMA_RX_CH, DMA_CCR_PSIZE_8BIT);
	dma_set_peripheral_address(SPI_DMA, SPI_DMA_RX_CH, (uint32_t)&SPI_DR(SPI_DEV));
	dma_enable_transfer_complete_interrupt(SPI_DMA, SPI_DMA_RX_CH);
	nvic_set_priority(SPI_DMA_RX_IRQ, SPI_DMA_IRQ_PRIO);
	nvic_enable_irq(SPI_DMA_RX_IRQ);

	dma_channel_reset(SPI_DMA, SPI_DMA_TX_CH);
	dma_set_channel_request(SPI_DMA, SPI_DMA_TX_CH, SPI_DMA_TX_REQ);
	dma_set_priority(SPI_DMA, SPI_DMA_TX_CH, DMA_CCR_PL_MEDIUM);
	dma_set_read_from_memory(SPI_DMA, SPI_DMA_TX_CH);
	dma_set_memory_size(SPI_DMA, SPI_DMA_TX_CH, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(SPI_DMA, SPI_DMA_TX_CH, DMA_CCR_PSIZE_8BIT);
	dma_set_peripheral_address(SPI_DMA, SPI_DMA_TX_CH, (uint32_t)&SPI_DR(SPI_DEV));
	dma_enable_transfer_complete_interrupt(SPI_DMA, SPI_DMA_TX_CH);
	nvic_set_priority(SPI_DMA_TX_IRQ, SPI_DMA_IRQ_PRIO);
	nvic_enable_irq(SPI_DMA_TX_IRQ);

	timer_one_shot_mode(SPI_TIM_MASTER);
	timer_set_master_mode(SPI_TIM_MASTER, TIM_CR2_MMS_ENABLE);
	timer_slave_set_trigger(SPI_TIM_MASTER, TIM_SMCR_TS_TI2FP2);
	timer_slave_set_mode(SPI_TIM_MASTER, TIM_SMCR_SMS_ECM1);
	timer_ic_set_input(SPI_TIM_MASTER, SPI_TIM_MASTER_IC, TIM_IC_IN_TI2);
	timer_ic_set_polarity(SPI_TIM_MASTER, SPI_TIM_MASTER_IC, TIM_IC_FALLING);

	timer_set_period(SPI_TIM_SLAVE, SPI_CLK_PERIOD - 1);
	timer_slave_set_trigger(SPI_TIM_SLAVE, SPI_TIM_SLAVE_TRIGGER);
	timer_slave_set_mode(SPI_TIM_SLAVE, TIM_SMCR_SMS_GM);
	timer_set_oc_mode(SPI_TIM_SLAVE, SPI_TIM_SLAVE_OC, TIM_OCM_PWM2);
	timer_set_oc_value(SPI_TIM_SLAVE, SPI_TIM_SLAVE_OC, SPI_CLK_PERIOD / 2);
	timer_enable_oc_output(SPI_TIM_SLAVE, SPI_TIM_SLAVE_OC);
	timer_enable_counter(SPI_TIM_SLAVE);

	rcc_periph_reset_pulse(SPI_DEV_RST);
	spi_enable_software_slave_management(SPI_DEV);
	spi_set_nss_low(SPI_DEV);
	spi_enable_rx_dma(SPI_DEV);
	spi_enable_tx_dma(SPI_DEV);
	spi_enable(SPI_DEV);

	gpio_set_mode(SPI_GPIO_PORT, GPIO_MODE_OUTPUT_10_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      SPI_GPIO_NSS);

	gpio_set_mode(SPI_GPIO_PORT, GPIO_MODE_OUTPUT_10_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      SPI_GPIO_SCK|SPI_GPIO_MISO);

	gpio_set_mode(SPI_GPIO_PORT, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      SPI_GPIO_SCKIN|SPI_GPIO_MOSI);

	gpio_set(SPI_GPIO_PORT, SPI_GPIO_NSS);
}

static void dma_setup_channel(uint32_t dma, uint8_t chan,
			      const void *p, uint32_t len)
{
	static volatile uint32_t dummy;

	if (p) {
		dma_enable_memory_increment_mode(dma, chan);
	} else {
		dma_disable_memory_increment_mode(dma, chan);
	}

	dma_set_memory_address(dma, chan, (uint32_t)(p ? p : &dummy));
	dma_set_number_of_data(dma, chan, len);
	dma_enable_channel(dma, chan);
}

static void spidev_xfer_setup(const spidev_xfer_t *xfer)
{
	dma_setup_channel(SPI_DMA, SPI_DMA_RX_CH, xfer->rx, xfer->len);
	dma_setup_channel(SPI_DMA, SPI_DMA_TX_CH, xfer->tx, xfer->len);
	timer_set_counter(SPI_TIM_MASTER, 2);
	timer_set_period(SPI_TIM_MASTER, xfer->len << 3);
}

static void spidev_xfer_start(const spidev_xfer_t *xfer)
{
	spidev_xfer_setup(xfer);
	gpio_clear(SPI_GPIO_PORT, SPI_GPIO_NSS);
	timer_enable_counter(SPI_TIM_MASTER);
}

static void spidev_transceive(const spidev_xfer_t *s, int count)
{
	BGRT_ASSERT(s, "s != NULL");
	BGRT_ASSERT(count, "count != 0");

	current = s;
	nxfers = count;
	spidev_xfer_start(s);
	bgrt_sem_lock(&spidev_sem);
}

static void extint_wait()
{
	nvic_enable_irq(EXTI_IRQ);
	bgrt_sem_lock(&extint_sem);
}

extint_t extint_init(void)
{
	exti_ll_init();
	bgrt_sem_init(&extint_sem, 0);
	bgrt_vint_init(&extint_vint, 2, spidev_signal, &extint_sem);

	return extint_wait;
}

#if SPIDEV_LOOPBACK_TEST
static uint8_t rxbuf[256];
static uint8_t txbuf[256];
#include <string.h>
#endif

spidev_t spidev_init(void)
{
	spidev_ll_init();
	bgrt_sem_init(&spidev_sem, 0);
	bgrt_vint_init(&spidev_vint, 2, spidev_signal, &spidev_sem);

#if SPIDEV_LOOPBACK_TEST
	for (unsigned i = 0; i < sizeof(txbuf); i++)
		txbuf[i] = i&0xff;

	spidev_transceive(&(spidev_xfer_t) {
			.rx = rxbuf,
			.tx = txbuf,
			.len = sizeof(rxbuf)
		}, 1);

	BGRT_ASSERT(!memcmp(rxbuf, txbuf, sizeof(rxbuf)), "rx == tx");
#endif
	return spidev_transceive;
}
