// SPDX-License-Identifier: GPL-2.0+
/*
 * Microchip ENC28J60 ethernet driver (MAC + PHY)
 * LwIP adaptation by Sergey Bolshakov <beefdeadbeef@gmail.com>
 * based on Linux driver by:
 *
 * Copyright (C) 2007 Eurek srl
 * Author: Claudio Lanconelli <lanconelli.claudio@eptar.com>
 * based on enc28j60.c written by David Anders for 2.4 kernel version
 *
 * $Id: enc28j60.c,v 1.22 2007/12/20 10:47:01 claudio Exp $
 */

#include <alloca.h>

#include <lwip/debug.h>
#include <lwip/dhcp.h>
#include <lwip/err.h>
#include <lwip/memp.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/pbuf.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>

#include "enc28j60.h"
#include "enc28j60.glue.h"

#ifndef	ENC28J60_DEBUG
#define	ENC28J60_DEBUG		LWIP_DBG_ON
#endif

#define SPI_OPLEN		1
#define TX_QUEUE_MAX		4
/* Max TX retries in case of collision as suggested by errata datasheet */
#define MAX_TX_RETRYCOUNT	16

enum {
	RXFILTER_NORMAL,
	RXFILTER_MULTI,
	RXFILTER_PROMISC
};

static struct enc28j60 {
	spidev_t spidev;
	sys_mutex_t lock;
	sys_sem_t rxready;
	sys_sem_t txready;
	sys_thread_t rxhandler;
	sys_thread_t txhandler;
	sys_mbox_t txqueue;
	int rxfilter;
	uint16_t next_pk_ptr;		/* next packet pointer within FIFO */
	uint16_t tx_retry_count;
	uint8_t bank;			/* current register bank selected */
	bool hw_enable;
} enc28j60;

/*
 * SPI read buffer
 */
static void spi_read_buf(struct enc28j60 *priv, uint16_t len, uint8_t *data)
{
	uint32_t buf = ENC28J60_READ_BUF_MEM;
	spidev_xfer_t s[] = {
		{ .rx = &buf, .tx = &buf, .len = SPI_OPLEN },
		{ .rx = data, .len = len }
	};

	(priv->spidev)(s, 2);
}

/*
 * SPI read pbuf
 */
static void spi_read_pbuf(struct enc28j60 *priv, struct pbuf *p)
{
	uint32_t nxfers, buf = ENC28J60_READ_BUF_MEM;
	spidev_xfer_t *s, *ss;

	nxfers = pbuf_clen(p) + 1;
	s = ss = alloca(nxfers * sizeof(spidev_xfer_t));

	s->rx = &buf;
	s->tx = &buf;
	s->len = SPI_OPLEN;
	s++;

	for (struct pbuf *q = p; q != NULL; q = q->next) {
		s->rx = q->payload;
		s->tx = NULL;
		s->len = q->len;
		s++;
	}

	(priv->spidev)(ss, nxfers);
}

/*
 * SPI write pbuf
 */
static void spi_write_pbuf(struct enc28j60 *priv, struct pbuf *p)
{
	uint32_t nxfers, buf = ENC28J60_WRITE_BUF_MEM;
	spidev_xfer_t *s, *ss;

	nxfers = pbuf_clen(p) + 1;
	s = ss = alloca(nxfers * sizeof(spidev_xfer_t));

	s->rx = &buf;
	s->tx = &buf;
	s->len = SPI_OPLEN;
	s++;

	for (struct pbuf *q = p; q != NULL; q = q->next) {
		s->rx = NULL;
		s->tx = q->payload;
		s->len = q->len;
		s++;
	}

	(priv->spidev)(ss, nxfers);
}

/*
 * basic SPI read operation
 */
static uint8_t spi_read_op(struct enc28j60 *priv,
			   uint8_t op, uint8_t addr)
{
	uint32_t buf = op | (addr & ADDR_MASK);
	uint16_t len = (addr & SPRD_MASK) ? SPI_OPLEN + 2 : SPI_OPLEN + 1;
	spidev_xfer_t s = { .rx = &buf, .tx = &buf, .len = len };

	(priv->spidev)(&s, 1);

	return 0xff & ((addr & SPRD_MASK) ? buf >> 16 : buf >> 8);
}

/*
 * basic SPI write operation
 */
static void spi_write_op(struct enc28j60 *priv,
			 uint8_t op, uint8_t addr, uint8_t val)
{
	uint32_t buf = op | (addr & ADDR_MASK) | val << 8;
	spidev_xfer_t s = { .rx = &buf, .tx = &buf, .len = SPI_OPLEN + 1 };

	(priv->spidev)(&s, 1);
}

/*
 * select the current register bank if necessary
 */
static void enc28j60_set_bank(struct enc28j60 *priv, uint8_t addr)
{
	uint8_t b = (addr & BANK_MASK) >> 5;

	/* These registers (EIE, EIR, ESTAT, ECON2, ECON1)
	 * are present in all banks, no need to switch bank.
	 */
	if (addr >= EIE && addr <= ECON1)
		return;

	/* Clear or set each bank selection bit as needed */
	if ((b & ECON1_BSEL0) != (priv->bank & ECON1_BSEL0))
		spi_write_op(priv,
			     b & ECON1_BSEL0 ?
			     ENC28J60_BIT_FIELD_SET :
			     ENC28J60_BIT_FIELD_CLR,
			     ECON1, ECON1_BSEL0);

	if ((b & ECON1_BSEL1) != (priv->bank & ECON1_BSEL1))
		spi_write_op(priv,
			     b & ECON1_BSEL1 ?
			     ENC28J60_BIT_FIELD_SET :
			     ENC28J60_BIT_FIELD_CLR,
			     ECON1, ECON1_BSEL1);
	priv->bank = b;
}

/*
 * Register access routines through the SPI bus.
 * Every register access comes in two flavours:
 * - nolock_xxx: caller needs to invoke mutex_lock, usually to access
 *   atomically more than one register
 * - locked_xxx: caller doesn't need to invoke mutex_lock, single access
 *
 * Some registers can be accessed through the bit field clear and
 * bit field set to avoid a read modify write cycle.
 */

/*
 * Register bit field Set
 */
static void nolock_reg_bfset(struct enc28j60 *priv, uint8_t addr, uint8_t mask)
{
	enc28j60_set_bank(priv, addr);
	spi_write_op(priv, ENC28J60_BIT_FIELD_SET, addr, mask);
}

static void locked_reg_bfset(struct enc28j60 *priv, uint8_t addr, uint8_t mask)
{
	sys_mutex_lock(&priv->lock);
	nolock_reg_bfset(priv, addr, mask);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Register bit field Clear
 */
static void nolock_reg_bfclr(struct enc28j60 *priv, uint8_t addr, uint8_t mask)
{
	enc28j60_set_bank(priv, addr);
	spi_write_op(priv, ENC28J60_BIT_FIELD_CLR, addr, mask);
}

static void locked_reg_bfclr(struct enc28j60 *priv, uint8_t addr, uint8_t mask)
{
	sys_mutex_lock(&priv->lock);
	nolock_reg_bfclr(priv, addr, mask);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Register byte read
 */
static uint8_t nolock_regb_read(struct enc28j60 *priv, uint8_t addr)
{
	enc28j60_set_bank(priv, addr);
	return spi_read_op(priv, ENC28J60_READ_CTRL_REG, addr);
}

static uint8_t locked_regb_read(struct enc28j60 *priv, uint8_t addr)
{
	uint8_t ret;

	sys_mutex_lock(&priv->lock);
	ret = nolock_regb_read(priv, addr);
	sys_mutex_unlock(&priv->lock);

	return ret;
}

/*
 * Register word read
 */
static uint16_t nolock_regw_read(struct enc28j60 *priv, uint8_t addr)
{
	uint8_t rl, rh;

	enc28j60_set_bank(priv, addr);
	rl = spi_read_op(priv, ENC28J60_READ_CTRL_REG, addr);
	rh = spi_read_op(priv, ENC28J60_READ_CTRL_REG, addr + 1);
	return (rh << 8) | rl;
}

static uint16_t locked_regw_read(struct enc28j60 *priv, uint8_t addr)
{
	uint16_t ret;

	sys_mutex_lock(&priv->lock);
	ret = nolock_regw_read(priv, addr);
	sys_mutex_unlock(&priv->lock);

	return ret;
}

/*
 * Register byte write
 */
static void nolock_regb_write(struct enc28j60 *priv,
			      uint8_t addr, uint8_t data)
{
	enc28j60_set_bank(priv, addr);
	spi_write_op(priv, ENC28J60_WRITE_CTRL_REG, addr, data);
}

static void locked_regb_write(struct enc28j60 *priv,
			      uint8_t addr, uint8_t data)
{
	sys_mutex_lock(&priv->lock);
	nolock_regb_write(priv, addr, data);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Register word write
 */
static void nolock_regw_write(struct enc28j60 *priv,
			      uint8_t addr, uint16_t data)
{
	enc28j60_set_bank(priv, addr);
	spi_write_op(priv, ENC28J60_WRITE_CTRL_REG, addr, (uint8_t)data);
	spi_write_op(priv, ENC28J60_WRITE_CTRL_REG, addr + 1, (uint8_t)(data >> 8));
}

static void locked_regw_write(struct enc28j60 *priv,
			      uint8_t addr, uint16_t data)
{
	sys_mutex_lock(&priv->lock);
	nolock_regw_write(priv, addr, data);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Buffer memory read
 * Select the starting address and execute a SPI buffer read.
 */
static void enc28j60_mem_read(struct enc28j60 *priv,
			      uint16_t addr, uint16_t len, uint8_t *data)
{
	sys_mutex_lock(&priv->lock);
	nolock_regw_write(priv, ERDPTL, addr);
	spi_read_buf(priv, len, data);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Buffer memory read, pbuf version
 * Select the starting address and execute a SPI buffer read.
 */
static void enc28j60_mem_pbuf(struct enc28j60 *priv,
			      uint16_t addr, struct pbuf *p)
{
	sys_mutex_lock(&priv->lock);
	nolock_regw_write(priv, ERDPTL, addr);
	spi_read_pbuf(priv, p);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Write packet to enc28j60 TX buffer memory
 */
static void enc28j60_packet_write(struct enc28j60 *priv, struct pbuf *p)
{
	sys_mutex_lock(&priv->lock);
	/* Set the write pointer to start of transmit buffer area */
	nolock_regw_write(priv, EWRPTL, TXSTART_INIT);
	/* Set the TXND pointer to correspond to the packet size given */
	nolock_regw_write(priv, ETXNDL, TXSTART_INIT + p->tot_len);
	/* write per-packet control byte */
	spi_write_op(priv, ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	/* copy the packet into the transmit buffer */
	spi_write_pbuf(priv, p);
	sys_mutex_unlock(&priv->lock);
}

static int poll_ready(struct enc28j60 *priv,
		      uint8_t reg, uint8_t mask, uint8_t val)
{
	/* 20 msec timeout read */
	unsigned retries = 20;

	while ((nolock_regb_read(priv, reg) & mask) != val) {
		if (--retries == 0) {
			LWIP_DEBUGF(ENC28J60_DEBUG,
				    ("reg %02x ready timeout!\n", reg));
			return -1;
		}
		sys_arch_msleep(1);
	}
	return 0;
}

/*
 * Wait until the PHY operation is complete.
 */
static int wait_phy_ready(struct enc28j60 *priv)
{
	return poll_ready(priv, MISTAT, MISTAT_BUSY, 0) ? 0 : 1;
}

/*
 * PHY register read
 * PHY registers are not accessed directly, but through the MII.
 */
static uint16_t enc28j60_phy_read(struct enc28j60 *priv, uint8_t address)
{
	uint16_t ret;

	sys_mutex_lock(&priv->lock);
	/* set the PHY register address */
	nolock_regb_write(priv, MIREGADR, address);
	/* start the register read operation */
	nolock_regb_write(priv, MICMD, MICMD_MIIRD);
	/* wait until the PHY read completes */
	wait_phy_ready(priv);
	/* quit reading */
	nolock_regb_write(priv, MICMD, 0x00);
	/* return the data */
	ret = nolock_regw_read(priv, MIRDL);
	sys_mutex_unlock(&priv->lock);

	return ret;
}

static int enc28j60_phy_write(struct enc28j60 *priv,
			      uint8_t address, uint16_t data)
{
	int ret;

	sys_mutex_lock(&priv->lock);
	/* set the PHY register address */
	nolock_regb_write(priv, MIREGADR, address);
	/* write the PHY data */
	nolock_regw_write(priv, MIWRL, data);
	/* wait until the PHY write completes and return */
	ret = wait_phy_ready(priv);
	sys_mutex_unlock(&priv->lock);

	return ret;
}

/*
 * Program the hardware MAC address
 */
static void enc28j60_set_hw_macaddr(struct enc28j60 *priv, uint8_t *dev_addr)
{
	LWIP_ASSERT("hw_enable", !priv->hw_enable);
	sys_mutex_lock(&priv->lock);
	/* NOTE: MAC address in ENC28J60 is byte-backward */
	nolock_regb_write(priv, MAADR5, dev_addr[0]);
	nolock_regb_write(priv, MAADR4, dev_addr[1]);
	nolock_regb_write(priv, MAADR3, dev_addr[2]);
	nolock_regb_write(priv, MAADR2, dev_addr[3]);
	nolock_regb_write(priv, MAADR1, dev_addr[4]);
	nolock_regb_write(priv, MAADR0, dev_addr[5]);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Debug routine to dump useful register contents
 */
static void enc28j60_dump_regs(struct enc28j60 *priv)
{
	sys_mutex_lock(&priv->lock);

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("HwRevID: 0x%02x\n"
		     "Cntrl: ECON1 ECON2 ESTAT  EIR  EIE\n"
		     "       0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n"
		     "MAC  : MACON1 MACON3 MACON4\n"
		     "       0x%02x   0x%02x   0x%02x\n"
		     "Rx   : ERXST  ERXND  ERXWRPT ERXRDPT ERXFCON EPKTCNT MAMXFL\n"
		     "       0x%04x 0x%04x 0x%04x  0x%04x  "
		     "0x%02x    0x%02x    0x%04x\n"
		     "Tx   : ETXST  ETXND  MACLCON1 MACLCON2 MAPHSUP\n"
		     "       0x%04x 0x%04x 0x%02x     0x%02x     0x%02x\n",
		     nolock_regb_read(priv, EREVID),

		     nolock_regb_read(priv, ECON1),
		     nolock_regb_read(priv, ECON2),
		     nolock_regb_read(priv, ESTAT),
		     nolock_regb_read(priv, EIR),
		     nolock_regb_read(priv, EIE),

		     nolock_regb_read(priv, MACON1),
		     nolock_regb_read(priv, MACON3),
		     nolock_regb_read(priv, MACON4),

		     nolock_regw_read(priv, ERXSTL),
		     nolock_regw_read(priv, ERXNDL),
		     nolock_regw_read(priv, ERXWRPTL),
		     nolock_regw_read(priv, ERXRDPTL),

		     nolock_regb_read(priv, ERXFCON),
		     nolock_regb_read(priv, EPKTCNT),
		     nolock_regw_read(priv, MAMXFLL),

		     nolock_regw_read(priv, ETXSTL),
		     nolock_regw_read(priv, ETXNDL),
		     nolock_regb_read(priv, MACLCON1),
		     nolock_regb_read(priv, MACLCON2),
		     nolock_regb_read(priv, MAPHSUP)));

	sys_mutex_unlock(&priv->lock);
}

/*
 * ERXRDPT need to be set always at odd addresses, refer to errata datasheet
 */
static uint16_t erxrdpt_workaround(uint16_t next_packet_ptr,
				   uint16_t start, uint16_t end)
{
	uint16_t erxrdpt;

	if ((next_packet_ptr - 1 < start) || (next_packet_ptr - 1 > end))
		erxrdpt = end;
	else
		erxrdpt = next_packet_ptr - 1;

	return erxrdpt;
}

/*
 * Calculate wrap around when reading beyond the end of the RX buffer
 */
static uint16_t rx_packet_start(uint16_t ptr)
{
	if (ptr + RSV_SIZE > RXEND_INIT)
		return (ptr + RSV_SIZE) - (RXEND_INIT - RXSTART_INIT + 1);
	else
		return ptr + RSV_SIZE;
}

static void nolock_rxfifo_init(struct enc28j60 *priv,
			       uint16_t start, uint16_t end)
{
	uint16_t erxrdpt;

	LWIP_ERROR("RXFIFO bad parameters\n",
		   (end <= 0x1FFF && start < end),
		   return);

	/* set receive buffer start + end */
	priv->next_pk_ptr = start;
	nolock_regw_write(priv, ERXSTL, start);
	erxrdpt = erxrdpt_workaround(priv->next_pk_ptr, start, end);
	nolock_regw_write(priv, ERXRDPTL, erxrdpt);
	nolock_regw_write(priv, ERXNDL, end);
}

static void nolock_txfifo_init(struct enc28j60 *priv,
			       uint16_t start, uint16_t end)
{
	LWIP_ERROR("TXFIFO bad parameters\n",
		   (end <= 0x1FFF && start < end),
		   return);

	/* set transmit buffer start + end */
	nolock_regw_write(priv, ETXSTL, start);
	nolock_regw_write(priv, ETXNDL, end);
}

static void enc28j60_soft_reset(struct enc28j60 *priv)
{
	uint8_t reg = ENC28J60_SOFT_RESET;
	spidev_xfer_t s = { .rx = &reg, .tx = &reg, .len = SPI_OPLEN };

	(priv->spidev)(&s, 1);
	sys_msleep(1);

	reg = nolock_regb_read(priv, EREVID);
	LWIP_ASSERT("sane RevId", (reg !=0x00 && reg != 0xff));
}

static void enc28j60_hw_init(struct enc28j60 *priv)
{
	sys_mutex_lock(&priv->lock);

	/* Reset chip */
	enc28j60_soft_reset(priv);

	/* Clear ECON1 */
	spi_write_op(priv, ENC28J60_WRITE_CTRL_REG, ECON1, 0x00);
	priv->bank = 0;
	priv->hw_enable = false;
	priv->tx_retry_count = 0;
	priv->rxfilter = RXFILTER_NORMAL;

	/* enable address auto increment and voltage regulator powersave */
	nolock_regb_write(priv, ECON2, ECON2_AUTOINC | ECON2_VRPS);

	nolock_rxfifo_init(priv, RXSTART_INIT, RXEND_INIT);
	nolock_txfifo_init(priv, TXSTART_INIT, TXEND_INIT);
	sys_mutex_unlock(&priv->lock);

	/* default filter mode: (unicast OR broadcast) AND crc valid */
	locked_regb_write(priv, ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

	/* enable MAC receive */
	locked_regb_write(priv, MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

	/* enable automatic padding and CRC operations */
	locked_regb_write(priv, MACON3,
			  MACON3_PADCFG0 | MACON3_TXCRCEN |
			  MACON3_FRMLNEN | MACON3_FULDPX);

	/* set inter-frame gap (non-back-to-back) */
	locked_regb_write(priv, MAIPGL, 0x12);

	/* set inter-frame gap (back-to-back) */
	locked_regb_write(priv, MABBIPG, 0x15);

	/*
	 * MACLCON1 (default)
	 * MACLCON2 (default)
	 * Set the maximum packet size which the controller will accept.
	 */
	locked_regw_write(priv, MAMXFLL, MAX_FRAMELEN);

	/* Configure LEDs */
	if (!enc28j60_phy_write(priv, PHLCON, ENC28J60_LAMPS_MODE))
		goto err;
	if (!enc28j60_phy_write(priv, PHCON1, PHCON1_PDPXMD))
		goto err;
	if (!enc28j60_phy_write(priv, PHCON2, 0x00))
		goto err;

	enc28j60_dump_regs(priv);

	return;
err:
	LWIP_ASSERT("phy init", 0);
}

static void enc28j60_hw_enable(struct enc28j60 *priv)
{
	/* enable interrupts */
	enc28j60_phy_write(priv, PHIE, PHIE_PGEIE | PHIE_PLNKIE);

	sys_mutex_lock(&priv->lock);
	nolock_reg_bfclr(priv, EIR, EIR_DMAIF | EIR_LINKIF |
			 EIR_TXIF | EIR_TXERIF | EIR_RXERIF | EIR_PKTIF);
	nolock_regb_write(priv, EIE, EIE_INTIE | EIE_PKTIE | EIE_LINKIE |
			  EIE_TXIE | EIE_TXERIE | EIE_RXERIE);

	/* enable receive logic */
	nolock_reg_bfset(priv, ECON1, ECON1_RXEN);
	priv->hw_enable = true;
	sys_mutex_unlock(&priv->lock);
}

static void enc28j60_hw_disable(struct enc28j60 *priv)
{
	sys_mutex_lock(&priv->lock);
	/* disable interrupts and packet reception */
	nolock_regb_write(priv, EIE, 0x00);
	nolock_reg_bfclr(priv, ECON1, ECON1_RXEN);
	priv->hw_enable = false;
	sys_mutex_unlock(&priv->lock);
}

/*
 * Read the Transmit Status Vector
 */
static void enc28j60_read_tsv(struct enc28j60 *priv, uint8_t tsv[TSV_SIZE])
{
	uint16_t endptr;

	endptr = locked_regw_read(priv, ETXNDL);
	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("reading TSV at addr:0x%04x\n", endptr + 1));
	enc28j60_mem_read(priv, endptr + 1, TSV_SIZE, tsv);
}

static void enc28j60_dump_tsv(struct enc28j60 *priv, uint8_t tsv[TSV_SIZE])
{
	uint16_t tmp1, tmp2;
	(void)priv;

	LWIP_DEBUGF(ENC28J60_DEBUG, ("TSV:\n"));

	tmp1 = tsv[1];
	tmp1 <<= 8;
	tmp1 |= tsv[0];

	tmp2 = tsv[5];
	tmp2 <<= 8;
	tmp2 |= tsv[4];

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("ByteCount: %d, CollisionCount: %d, TotByteOnWire: %d\n",
		     tmp1, tsv[2] & 0x0f, tmp2));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("TxDone: %d, CRCErr:%d, LenChkErr: %d, LenOutOfRange: %d\n",
		     TSV_GETBIT(tsv, TSV_TXDONE),
		     TSV_GETBIT(tsv, TSV_TXCRCERROR),
		     TSV_GETBIT(tsv, TSV_TXLENCHKERROR),
		     TSV_GETBIT(tsv, TSV_TXLENOUTOFRANGE)));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("Multicast: %d, Broadcast: %d, PacketDefer: %d, ExDefer: %d\n",
		     TSV_GETBIT(tsv, TSV_TXMULTICAST),
		     TSV_GETBIT(tsv, TSV_TXBROADCAST),
		     TSV_GETBIT(tsv, TSV_TXPACKETDEFER),
		     TSV_GETBIT(tsv, TSV_TXEXDEFER)));
	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("ExCollision: %d, LateCollision: %d, Giant: %d, Underrun: %d\n",
		     TSV_GETBIT(tsv, TSV_TXEXCOLLISION),
		     TSV_GETBIT(tsv, TSV_TXLATECOLLISION),
		     TSV_GETBIT(tsv, TSV_TXGIANT),
		     TSV_GETBIT(tsv, TSV_TXUNDERRUN)));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("ControlFrame: %d, PauseFrame: %d, BackPressApp: %d, VLanTagFrame: %d\n",
		     TSV_GETBIT(tsv, TSV_TXCONTROLFRAME),
		     TSV_GETBIT(tsv, TSV_TXPAUSEFRAME),
		     TSV_GETBIT(tsv, TSV_BACKPRESSUREAPP),
		     TSV_GETBIT(tsv, TSV_TXVLANTAGFRAME)));
}

/*
 * Receive Status vector
 */
static void enc28j60_dump_rsv(struct enc28j60 *priv,
			      uint16_t pk_ptr, int len, uint16_t sts)
{
	(void)priv;

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("NextPk: 0x%04x - RSV:\n", pk_ptr));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("ByteCount: %d, DribbleNibble: %d\n",
		     len, RSV_GETBIT(sts, RSV_DRIBBLENIBBLE)));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("RxOK: %d, CRCErr:%d, LenChkErr: %d, LenOutOfRange: %d\n",
		     RSV_GETBIT(sts, RSV_RXOK),
		     RSV_GETBIT(sts, RSV_CRCERROR),
		     RSV_GETBIT(sts, RSV_LENCHECKERR),
		     RSV_GETBIT(sts, RSV_LENOUTOFRANGE)));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("Multicast: %d, Broadcast: %d, LongDropEvent: %d, CarrierEvent: %d\n",
		     RSV_GETBIT(sts, RSV_RXMULTICAST),
		     RSV_GETBIT(sts, RSV_RXBROADCAST),
		     RSV_GETBIT(sts, RSV_RXLONGEVDROPEV),
		     RSV_GETBIT(sts, RSV_CARRIEREV)));

	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("ControlFrame: %d, PauseFrame: %d, UnknownOp: %d, VLanTagFrame: %d\n",
		     RSV_GETBIT(sts, RSV_RXCONTROLFRAME),
		     RSV_GETBIT(sts, RSV_RXPAUSEFRAME),
		     RSV_GETBIT(sts, RSV_RXUNKNOWNOPCODE),
		     RSV_GETBIT(sts, RSV_RXTYPEVLAN)));
}

/*
 * Hardware receive function.
 * Read the buffer memory, update the FIFO pointer to free the buffer,
 * check the status vector and decrement the packet counter.
 */
static void enc28j60_hw_rx(struct netif *iface)
{
	struct enc28j60 *priv = iface->state;
	uint16_t erxrdpt, next_packet, rxstat;
	uint8_t rsv[RSV_SIZE];
	int len;

	if (priv->next_pk_ptr > RXEND_INIT) {
		LWIP_DEBUGF(ENC28J60_DEBUG,
			    ("Invalid packet address 0x%04x\n",
			     priv->next_pk_ptr));

		/* packet address corrupted: reset RX logic */
		sys_mutex_lock(&priv->lock);
		nolock_reg_bfclr(priv, ECON1, ECON1_RXEN);
		nolock_reg_bfset(priv, ECON1, ECON1_RXRST);
		nolock_reg_bfclr(priv, ECON1, ECON1_RXRST);
		nolock_rxfifo_init(priv, RXSTART_INIT, RXEND_INIT);
		nolock_reg_bfclr(priv, EIR, EIR_RXERIF);
		nolock_reg_bfset(priv, ECON1, ECON1_RXEN);
		sys_mutex_unlock(&priv->lock);
		LINK_STATS_INC(link.err);
		return;
	}

	/* Read next packet pointer and rx status vector */
	enc28j60_mem_read(priv, priv->next_pk_ptr, sizeof(rsv), rsv);

	next_packet = rsv[1];
	next_packet <<= 8;
	next_packet |= rsv[0];

	len = rsv[3];
	len <<= 8;
	len |= rsv[2];

	rxstat = rsv[5];
	rxstat <<= 8;
	rxstat |= rsv[4];

	if (!RSV_GETBIT(rxstat, RSV_RXOK) || len > MAX_FRAMELEN) {
		LWIP_DEBUGF(ENC28J60_DEBUG, ("Rx Error %04x\n", rxstat));
		enc28j60_dump_rsv(priv, next_packet, len, rxstat);
		LINK_STATS_INC(link.err);
	} else {
		struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
		if (!p) {
			LWIP_DEBUGF(ENC28J60_DEBUG, ("Rx: out of memory\n"));
			LINK_STATS_INC(link.memerr);
			LINK_STATS_INC(link.drop);
		} else {
			enc28j60_mem_pbuf(priv,
					  rx_packet_start(priv->next_pk_ptr),
					  p);
			/* update statistics */
			LINK_STATS_INC(link.recv);
			/* feed into lwip */
			if (iface->input(p, iface) != ERR_OK) {
				pbuf_free(p);
			}
		}
	}
	/*
	 * Move the RX read pointer to the start of the next
	 * received packet.
	 * This frees the memory we just read out.
	 */
	erxrdpt = erxrdpt_workaround(next_packet, RXSTART_INIT, RXEND_INIT);
	sys_mutex_lock(&priv->lock);
	nolock_regw_write(priv, ERXRDPTL, erxrdpt);
	priv->next_pk_ptr = next_packet;
	/* we are done with this packet, decrement the packet counter */
	nolock_reg_bfset(priv, ECON2, ECON2_PKTDEC);
	sys_mutex_unlock(&priv->lock);
}

/*
 * Calculate free space in RxFIFO
 */
static int enc28j60_get_free_rxfifo(struct enc28j60 *priv)
{
	int epkcnt, erxst, erxnd, erxwr, erxrd;
	int free_space;

	sys_mutex_lock(&priv->lock);
	epkcnt = nolock_regb_read(priv, EPKTCNT);
	if (epkcnt >= 255)
		free_space = -1;
	else {
		erxst = nolock_regw_read(priv, ERXSTL);
		erxnd = nolock_regw_read(priv, ERXNDL);
		erxwr = nolock_regw_read(priv, ERXWRPTL);
		erxrd = nolock_regw_read(priv, ERXRDPTL);

		if (erxwr > erxrd)
			free_space = (erxnd - erxst) - (erxwr - erxrd);
		else if (erxwr == erxrd)
			free_space = (erxnd - erxst);
		else
			free_space = erxrd - erxwr - 1;
	}
	sys_mutex_unlock(&priv->lock);

	return free_space;
}

/*
 * Access the PHY to determine link status
 */
static void enc28j60_check_link_status(struct netif *iface)
{
	struct enc28j60 *priv = iface->state;
	uint16_t reg;

	reg = enc28j60_phy_read(priv, PHSTAT2);
	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("PHSTAT1: %04x, PHSTAT2: %04x\n",
		     enc28j60_phy_read(priv, PHSTAT1), reg));

	if (reg & PHSTAT2_LSTAT) {
		netifapi_netif_set_link_up(iface);
	} else {
		netifapi_netif_set_link_down(iface);
	}
}

static void enc28j60_tx_clear(struct enc28j60 *priv, bool err)
{
	if (err) {
		LINK_STATS_INC(link.err);
	} else {
		LINK_STATS_INC(link.xmit);
	}

	locked_reg_bfclr(priv, ECON1, ECON1_TXRTS);
	/* wake tx queue */
	sys_sem_signal(&priv->txready);
}

/*
 * RX handler
 * Ignore PKTIF because is unreliable! (Look at the errata datasheet)
 * Check EPKTCNT is the suggested workaround.
 * We don't need to clear interrupt flag, automatically done when
 * enc28j60_hw_rx() decrements the packet counter.
 * Returns how many packet processed.
 */
static int enc28j60_rx_interrupt(struct netif *iface)
{
	struct enc28j60 *priv = iface->state;
	int pk_counter, ret;

	ret = pk_counter = locked_regb_read(priv, EPKTCNT);
	while (pk_counter-- > 0)
		enc28j60_hw_rx(iface);

	return ret;
}

static void enc28j60_rx_handler(void *ctx)
{
	struct netif *iface = ctx;
	struct enc28j60 *priv = iface->state;
	int intflags, loop;

	enc28j60_check_link_status(iface);

loop:	sys_sem_wait(&priv->rxready);

	/* disable further interrupts */
	locked_reg_bfclr(priv, EIE, EIE_INTIE);

	do {
		loop = 0;
		intflags = locked_regb_read(priv, EIR);
		/* DMA interrupt handler (not currently used) */
		if ((intflags & EIR_DMAIF) != 0) {
			loop++;
			locked_reg_bfclr(priv, EIR, EIR_DMAIF);
		}
		/* LINK changed handler */
		if ((intflags & EIR_LINKIF) != 0) {
			loop++;
			LWIP_DEBUGF(ENC28J60_DEBUG, ("*Int Link\n"));
			enc28j60_check_link_status(iface);
			/* read PHIR to clear the flag */
			enc28j60_phy_read(priv, PHIR);
		}
		/* TX complete handler */
		if (((intflags & EIR_TXIF) != 0) &&
		    ((intflags & EIR_TXERIF) == 0)) {
			bool err = false;
			loop++;
			//LWIP_DEBUGF(ENC28J60_DEBUG, ("*Int Tx\n"));
			priv->tx_retry_count = 0;
			if (locked_regb_read(priv, ESTAT) & ESTAT_TXABRT) {
				LWIP_DEBUGF(ENC28J60_DEBUG, ("TX error (abrt)\n"));
				err = true;
			}
			enc28j60_tx_clear(priv, err);
			locked_reg_bfclr(priv, EIR, EIR_TXIF);
		}
		/* TX Error handler */
		if ((intflags & EIR_TXERIF) != 0) {
			uint8_t tsv[TSV_SIZE];

			loop++;
			LWIP_DEBUGF(ENC28J60_DEBUG, ("*Int TxErr\n"));
			locked_reg_bfclr(priv, ECON1, ECON1_TXRTS);
			enc28j60_read_tsv(priv, tsv);
			/* Reset TX logic */
			sys_mutex_lock(&priv->lock);
			nolock_reg_bfset(priv, ECON1, ECON1_TXRST);
			nolock_reg_bfclr(priv, ECON1, ECON1_TXRST);
			nolock_txfifo_init(priv, TXSTART_INIT, TXEND_INIT);
			sys_mutex_unlock(&priv->lock);
			/* Transmit Late collision check for retransmit */
			if (TSV_GETBIT(tsv, TSV_TXLATECOLLISION)) {
				LWIP_DEBUGF(ENC28J60_DEBUG,
					    ("LateCollision TXErr: %d\n",
					     priv->tx_retry_count));
				if (priv->tx_retry_count++ < MAX_TX_RETRYCOUNT)
					locked_reg_bfset(priv, ECON1, ECON1_TXRTS);
				else
					enc28j60_tx_clear(priv, true);
			} else
				enc28j60_tx_clear(priv, true);
			locked_reg_bfclr(priv, EIR, EIR_TXERIF | EIR_TXIF);
		}
		/* RX Error handler */
		if ((intflags & EIR_RXERIF) != 0) {
			loop++;
			LWIP_DEBUGF(ENC28J60_DEBUG, ("*Int RXErr\n"));
			/* Check free FIFO space to flag RX overrun */
			if (enc28j60_get_free_rxfifo(priv) <= 0) {
				LWIP_DEBUGF(ENC28J60_DEBUG, ("RX Overrun"));
				LINK_STATS_INC(link.drop);
			}
			locked_reg_bfclr(priv, EIR, EIR_RXERIF);
		}
		/* RX handler */
		if (enc28j60_rx_interrupt(iface)) {
			loop++;
			//LWIP_DEBUGF(ENC28J60_DEBUG, ("*Int RX\n"));
		}
	} while (loop);

	/* re-enable interrupts */
	locked_reg_bfset(priv, EIE, EIE_INTIE);

	goto loop;
}

/*
 * Hardware transmit function.
 * Fill the buffer memory and send the contents of the transmit buffer
 * onto the network
 */
static void enc28j60_tx_handler(void *ctx)
{
	struct netif *iface = ctx;
	struct enc28j60 *priv = iface->state;
	struct pbuf *p;

loop:	sys_mbox_fetch(&priv->txqueue, (void **)&p);
	sys_sem_wait(&priv->txready);
	enc28j60_packet_write(priv, p);
	/* set TX request flag */
	locked_reg_bfset(priv, ECON1, ECON1_TXRTS);
	LINK_STATS_INC(link.xmit);
	pbuf_free(p);

	goto loop;
}

static err_t enc28j60_output(struct netif *iface, struct pbuf *p)
{
	struct enc28j60 *priv = iface->state;

	pbuf_ref(p);
	sys_mbox_post(&priv->txqueue, p);

	return ERR_OK;
}

static err_t enc28j60_init(struct netif *iface)
{
	struct enc28j60 *priv = iface->state;

	sys_mutex_new(&priv->lock);
	sys_sem_new(&priv->rxready, 0);
	sys_sem_new(&priv->txready, 1);
	sys_mbox_new(&priv->txqueue, TX_QUEUE_MAX);

	priv->spidev = spidev_init();

	enc28j60_hw_init(priv);
	enc28j60_set_hw_macaddr(priv, iface->hwaddr);

	priv->rxhandler = sys_thread_new("enc28j60_rx",
					 enc28j60_rx_handler, iface,
					 DEFAULT_THREAD_STACKSIZE,
					 DEFAULT_THREAD_PRIO - 2);

	priv->txhandler = sys_thread_new("enc28j60_tx",
					 enc28j60_tx_handler, iface,
					 DEFAULT_THREAD_STACKSIZE,
					 DEFAULT_THREAD_PRIO - 1);
	exti_init(&priv->rxready);
	enc28j60_hw_enable(priv);

	iface->linkoutput = enc28j60_output;
	iface->output = etharp_output;
	iface->mtu = 1500;
	iface->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

	netif_set_up(iface);

	return ERR_OK;
}

#if LWIP_NETIF_LINK_CALLBACK
static void link_cb(struct netif *iface)
{
	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("link is %s\n", netif_is_link_up(iface) ? "up" : "down"));
}
#endif

#if LWIP_NETIF_STATUS_CALLBACK
static void status_cb(struct netif *iface)
{
	LWIP_DEBUGF(ENC28J60_DEBUG,
		    ("address: %s\n", ip4addr_ntoa(netif_ip4_addr(iface))));
}
#endif

void iface_init(struct netif *iface)
{
	netif_add_noaddr(iface, &enc28j60, enc28j60_init, tcpip_input);
	netif_set_default(iface);
#if LWIP_NETIF_STATUS_CALLBACK
	netif_set_status_callback(iface, status_cb);
#endif
#if LWIP_NETIF_LINK_CALLBACK
	netif_set_link_callback(iface, link_cb);
#endif
	dhcp_start(iface);
}
