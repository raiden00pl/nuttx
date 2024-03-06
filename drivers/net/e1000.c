/*****************************************************************************
 * drivers/net/e1000.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/addrenv.h>
#include <nuttx/spinlock.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/e1000.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "e1000.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

/* Packet buffer size */

#define PKTBUF_SIZE      (2048)
#define E1000_RCTL_BSIZE E1000_RCTL_BSIZE_2048

/* TX and RX descriptors */

#define E1000_TX_DESC 256
#define E1000_RX_DESC 256

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->dev.d_buf)

/* PCI BARs */

#define E1000_MMIO_BAR  (0)
#define E1000_FLASH_BAR (1)
#define E1000_IO_BAR    (2)
#define E1000_MSIX_BAR  (3)

/* E1000 interrupts */

#define E1000_INTERRUPTS (E1000_IC_RXO | E1000_IC_RXT0 | \
                          E1000_IC_RXDMT0 | E1000_IC_LSC| \
                          E1000_IC_TXDW)

/* NIC specific Flags */

#define E1000_RESET_BROKEN (1 << 0)

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Extend default PCI devie type */

struct e1000_pci_type_s
{
  FAR const struct pci_dev_type_s *type; /* PCI device type */
  int                              irq;
  bool                             msix;
  uint32_t                         flags;
};

/* E1000 private data */

struct e1000_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;

  /* Driver state */

  bool bifup;

  /* Buffers */

  FAR uint8_t *txbuf;
  FAR uint8_t *pktbuf;

  /* Descriptors */

  FAR struct e1000_tx_leg_s *tx;
  FAR struct e1000_rx_leg_s *rx;

  size_t tx_now;
  size_t tx_done;
  size_t tx_free;
  size_t rx_now;

  /* Work queues */

  struct work_s irqwork;
  struct work_s pollwork;

  /* PCI data */

  struct pci_dev_s             pcidev;
  FAR struct e1000_pci_type_s *type;
  uint64_t                     mimobase;
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

/* Helpers */

static uint32_t e1000_getreg_mem(FAR struct e1000_driver_s *priv,
                                 unsigned int offset);
static void e1000_putreg_mem(FAR struct e1000_driver_s *priv,
                             unsigned int offset,
                             uint32_t value);
#ifdef CONFIG_DEBUG_NET_INFO
static void e1000_dump_reg(FAR struct e1000_driver_s *priv,
                           FAR char *msg, unsigned int offset);
static void e1000_dump_mem(FAR struct e1000_driver_s *priv, FAR char *msg);
#endif

/* Common TX logic */

static void e1000_transmit(FAR struct net_driver_s *dev);
static int e1000_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void e1000_reply(FAR struct net_driver_s *dev);
static void e1000_receive(FAR struct net_driver_s *dev);
static void e1000_txdone(FAR struct net_driver_s *dev);

static void e1000_interrupt_work(FAR void *arg);
static int e1000_interrupt(int irq, FAR void *context, FAR void *arg);

/* NuttX callback functions */

static int e1000_ifup(FAR struct net_driver_s *dev);
static int e1000_ifdown(FAR struct net_driver_s *dev);

static void e1000_txavail_work(FAR void *arg);
static int e1000_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int e1000_addmac(FAR struct net_driver_s *dev,
                        FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int e1000_rmmac(FAR struct net_driver_s *dev,
                       FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int e1000_ioctl(FAR struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

/* Initialization */

static int e1000_reset(FAR struct e1000_driver_s *priv);
static int e1000_enable(FAR struct e1000_driver_s *priv);
static int e1000_initialize(FAR struct e1000_driver_s *priv);
static int e1000_probe(FAR struct pci_bus_s *bus,
                       FAR const struct pci_dev_type_s *type,
                       uint16_t bdf);

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifdef CONFIG_NET_E1000_82574L
/* Intel 82574L */

const struct pci_dev_type_s g_pci_type_82574l =
{
  .vendor    = 0x8086,
  .device    = 0x10d3,
  .class_rev = PCI_ID_ANY,
  .name      = "Intel 82574L",
  .probe     = e1000_probe
};

static struct e1000_pci_type_s g_pci_82574l =
{
  .type = &g_pci_type_82574l,
  .irq  = CONFIG_NET_E1000_82574L_IRQ,
  .msix = false,
  .flags = 0
};
#endif

#ifdef CONFIG_NET_E1000_82540EM
/* Intel 82801IB */

const struct pci_dev_type_s g_pci_type_82540em =
{
  .vendor    = 0x8086,
  .device    = 0x100e,
  .class_rev = PCI_ID_ANY,
  .name      = "Intel 82540EM",
  .probe     = e1000_probe
};

static struct e1000_pci_type_s g_pci_82540em =
{
  .type  = &g_pci_type_82540em,
  .irq   = CONFIG_NET_E1000_82540EM_IRQ,
  .msix  = false,
  .flags = 0
};
#endif

#ifdef CONFIG_NET_E1000_I219
/* Intel I219 */

const struct pci_dev_type_s g_pci_type_i219 =
{
  .vendor    = 0x8086,
  .device    = 0x1a1e,
  .class_rev = PCI_ID_ANY,
  .name      = "Intel I219",
  .probe     = e1000_probe
};

static struct e1000_pci_type_s g_pci_i219 =
{
  .type  = &g_pci_type_i219,
  .irq   = CONFIG_NET_E1000_I219_IRQ,
  .msix  = false,
  .flags = E1000_RESET_BROKEN
};
#endif

#ifdef CONFIG_NET_E1000_I225LM
/* Intel I225LM */

const struct pci_dev_type_s g_pci_type_i225lm =
{
  .vendor    = 0x8086,
  .device    = 0x15f2,
  .class_rev = PCI_ID_ANY,
  .name      = "Intel I225",
  .probe     = e1000_probe
};

static struct e1000_pci_type_s g_pci_i225lm =
{
  .type  = &g_pci_type_i225lm,
  .irq   = CONFIG_NET_E1000_I225LM_IRQ,
  .msix  = true,
  .flags = 0
};
#endif

/* PCI devices */

static struct e1000_pci_type_s *g_pci_e1000_devs[] =
{
#ifdef CONFIG_NET_E1000_82574L
  &g_pci_82574l,
#endif
#ifdef CONFIG_NET_E1000_82540EM
  &g_pci_82540em,
#endif
#ifdef CONFIG_NET_E1000_I219
  &g_pci_i219,
#endif
#ifdef CONFIG_NET_E1000_I225LM
  &g_pci_i225lm,
#endif
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: e1000_getreg_mem
 *****************************************************************************/

static uint32_t e1000_getreg_mem(FAR struct e1000_driver_s *priv,
                                 unsigned int offset)
{
  uintptr_t addr = priv->mimobase + offset;
  return *((FAR volatile uint32_t *)(addr));
}

/*****************************************************************************
 * Name: e1000_putreg_mem
 *****************************************************************************/

static void e1000_putreg_mem(FAR struct e1000_driver_s *priv,
                             unsigned int offset,
                             uint32_t value)
{
  uintptr_t addr = priv->mimobase + offset;
  *((FAR volatile uint32_t *)(addr)) = value;
}

#ifdef CONFIG_DEBUG_NET_INFO
/*****************************************************************************
 * Name: e1000_dump_reg
 *****************************************************************************/

static void e1000_dump_reg(FAR struct e1000_driver_s *priv,
                               FAR char *msg, unsigned int offset)
{
  ninfo("\t%s:\t\t0x%" PRIx32 "\n", msg, e1000_getreg_mem(priv, offset));
}

/*****************************************************************************
 * Name: e1000_dump_mem
 *****************************************************************************/

static void e1000_dump_mem(FAR struct e1000_driver_s *priv, FAR char *msg)
{
  ninfo("\nDump: %s\n", msg);

  ninfo("\nGeneral registers:\n");
  e1000_dump_reg(priv, "CTRL", E1000_CTRL);
  e1000_dump_reg(priv, "STATUS", E1000_STATUS);

  ninfo("Interrupt registers:\n");
  e1000_dump_reg(priv, "ICS", E1000_ICS);
  e1000_dump_reg(priv, "IMS", E1000_IMS);

  ninfo("Transmit registers:\n");
  e1000_dump_reg(priv, "TCTL", E1000_TCTL);
  e1000_dump_reg(priv, "TIPG", E1000_TIPG);
  e1000_dump_reg(priv, "AIT", E1000_AIT);
  e1000_dump_reg(priv, "TDBAL", E1000_TDBAL);
  e1000_dump_reg(priv, "TDBAH", E1000_TDBAH);
  e1000_dump_reg(priv, "TDLEN", E1000_TDLEN);
  e1000_dump_reg(priv, "TDH", E1000_TDH);
  e1000_dump_reg(priv, "TDT", E1000_TDT);
  e1000_dump_reg(priv, "TARC", E1000_TARC);
  e1000_dump_reg(priv, "TIDV", E1000_TIDV);
  e1000_dump_reg(priv, "TXDCTL", E1000_TXDCTL);
  e1000_dump_reg(priv, "TADV", E1000_TADV);

  ninfo("Receive registers:\n");
  e1000_dump_reg(priv, "RCTL", E1000_RCTL);
  e1000_dump_reg(priv, "RDBAL", E1000_RDBAL);
  e1000_dump_reg(priv, "RDBAH", E1000_RDBAH);
  e1000_dump_reg(priv, "RDLEN", E1000_RDLEN);
  e1000_dump_reg(priv, "RDH", E1000_RDH);
  e1000_dump_reg(priv, "RDT", E1000_RDT);
  e1000_dump_reg(priv, "RDTR", E1000_RDTR);
  e1000_dump_reg(priv, "RXDCTL", E1000_RXDCTL);
  e1000_dump_reg(priv, "RADV", E1000_RADV);
  e1000_dump_reg(priv, "RSRPD", E1000_RSRPD);
  e1000_dump_reg(priv, "RAID", E1000_RAID);
  e1000_dump_reg(priv, "RXCSUM", E1000_RXCSUM);
  e1000_dump_reg(priv, "RFCTL", E1000_RFCTL);
  e1000_dump_reg(priv, "RAL", E1000_RAL);
  e1000_dump_reg(priv, "RAH", E1000_RAH);

  ninfo("Statistic registers:\n");
  e1000_dump_reg(priv, "CRCERRS", E1000_CRCERRS);
  e1000_dump_reg(priv, "ALGNERRC", E1000_ALGNERRC);
  e1000_dump_reg(priv, "RXERRC", E1000_RXERRC);
  e1000_dump_reg(priv, "MPC", E1000_MPC);
  e1000_dump_reg(priv, "SCC", E1000_SCC);
  e1000_dump_reg(priv, "ECOL", E1000_ECOL);
  e1000_dump_reg(priv, "MCC", E1000_MCC);
  e1000_dump_reg(priv, "LATECOL", E1000_LATECOL);
  e1000_dump_reg(priv, "COLC", E1000_COLC);
  e1000_dump_reg(priv, "DC", E1000_DC);
  e1000_dump_reg(priv, "TNCRS", E1000_TNCRS);
  e1000_dump_reg(priv, "CEXTERR", E1000_CEXTERR);
  e1000_dump_reg(priv, "RLEC", E1000_RLEC);
  e1000_dump_reg(priv, "XONRXC", E1000_XONRXC);
  e1000_dump_reg(priv, "XONTXC", E1000_XONTXC);
  e1000_dump_reg(priv, "XOFFRXC", E1000_XOFFRXC);
  e1000_dump_reg(priv, "XOFFTXC", E1000_XOFFTXC);
  e1000_dump_reg(priv, "FCRUC", E1000_FCRUC);
  e1000_dump_reg(priv, "PRC64", E1000_PRC64);
  e1000_dump_reg(priv, "PRC127", E1000_PRC127);
  e1000_dump_reg(priv, "PRC255", E1000_PRC255);
  e1000_dump_reg(priv, "PRC511", E1000_PRC511);
  e1000_dump_reg(priv, "PRC1023", E1000_PRC1023);
  e1000_dump_reg(priv, "PRC1522", E1000_PRC1522);
  e1000_dump_reg(priv, "GPRC", E1000_GPRC);
  e1000_dump_reg(priv, "BPRC", E1000_BPRC);
  e1000_dump_reg(priv, "MPRC", E1000_MPRC);
  e1000_dump_reg(priv, "GPTC", E1000_GPTC);
  e1000_dump_reg(priv, "GORCL", E1000_GORCL);
  e1000_dump_reg(priv, "GORCH", E1000_GORCH);
  e1000_dump_reg(priv, "GOTCL", E1000_GOTCL);
  e1000_dump_reg(priv, "GOTCH", E1000_GOTCH);
  e1000_dump_reg(priv, "RNBC", E1000_RNBC);
  e1000_dump_reg(priv, "RUC", E1000_RUC);
  e1000_dump_reg(priv, "RFC", E1000_RFC);
  e1000_dump_reg(priv, "ROC", E1000_ROC);
  e1000_dump_reg(priv, "RJC", E1000_RJC);
  e1000_dump_reg(priv, "MNGPRC", E1000_MNGPRC);
  e1000_dump_reg(priv, "MPDC", E1000_MPDC);
  e1000_dump_reg(priv, "MPTC", E1000_MPTC);
  e1000_dump_reg(priv, "TORL", E1000_TORL);
  e1000_dump_reg(priv, "TORH", E1000_TORH);
  e1000_dump_reg(priv, "TOT", E1000_TOT);
  e1000_dump_reg(priv, "TPR", E1000_TPR);
  e1000_dump_reg(priv, "TPT", E1000_TPT);
  e1000_dump_reg(priv, "PTC64", E1000_PTC64);
  e1000_dump_reg(priv, "PTC127", E1000_PTC127);
  e1000_dump_reg(priv, "PTC255", E1000_PTC255);
  e1000_dump_reg(priv, "PTC511", E1000_PTC511);
  e1000_dump_reg(priv, "PTC1023", E1000_PTC1023);
  e1000_dump_reg(priv, "PTC1522", E1000_PTC1522);
  e1000_dump_reg(priv, "MPTC", E1000_MPTC);
  e1000_dump_reg(priv, "BPTC", E1000_BPTC);
  e1000_dump_reg(priv, "TSCTC", E1000_TSCTC);
  e1000_dump_reg(priv, "TSCTFC", E1000_TSCTFC);
  e1000_dump_reg(priv, "IAC", E1000_IAC);

  ninfo("Management registers:\n");
  e1000_dump_reg(priv, "WUC", E1000_WUC);
  e1000_dump_reg(priv, "WUFC", E1000_WUFC);
  e1000_dump_reg(priv, "WUS", E1000_WUS);
  e1000_dump_reg(priv, "MFUTP01", E1000_MFUTP01);
  e1000_dump_reg(priv, "MFUTP23", E1000_MFUTP23);
  e1000_dump_reg(priv, "IPAV", E1000_IPAV);

  ninfo("MSI-X registers: TODO\n");

  ninfo("Diagnostic registers:\n");
  e1000_dump_reg(priv, "POEMB", E1000_POEMB);
  e1000_dump_reg(priv, "RDFH", E1000_RDFH);
  e1000_dump_reg(priv, "FDFT", E1000_FDFT);
  e1000_dump_reg(priv, "RDFHS", E1000_RDFHS);
  e1000_dump_reg(priv, "RDFTS", E1000_RDFTS);
  e1000_dump_reg(priv, "RDFPC", E1000_RDFPC);
  e1000_dump_reg(priv, "TDFH", E1000_TDFH);
  e1000_dump_reg(priv, "TDFT", E1000_TDFT);
  e1000_dump_reg(priv, "TDFHS", E1000_TDFHS);
  e1000_dump_reg(priv, "TDFTS", E1000_TDFTS);
  e1000_dump_reg(priv, "TDFPC", E1000_TDFPC);
  e1000_dump_reg(priv, "PBM", E1000_PBM);
  e1000_dump_reg(priv, "PBS", E1000_PBS);
}
#endif

/*****************************************************************************
 * Name: e1000_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static void e1000_transmit(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;
  uint64_t pa                     = 0;
  int      buf                    = PKTBUF_SIZE * priv->tx_now - 1;
  int      desc                   = priv->tx_now;

  ninfo("transmit\n");

  /* Prepare next TX descriptor */

  priv->tx_now = (priv->tx_now + 1) % E1000_TX_DESC;

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->dev);

  /* Copy packet data to TX buffer */

  memcpy(&priv->txbuf[buf], priv->dev.d_buf, priv->dev.d_len);

  /* Setup TX descriptor */

  pa = up_addrenv_va_to_pa(&priv->txbuf[buf]);

  priv->tx[desc].addr   = pa;
  priv->tx[desc].len    = priv->dev.d_len;
  priv->tx[desc].cmd    = (TDESC_CMD_EOP | TDESC_CMD_IFCS |
                           TDESC_CMD_RS | TDESC_CMD_RPS);
  priv->tx[desc].cso    = 0;
  priv->tx[desc].status = 0;

  SP_DSB();

  /* Update TX tail */

  e1000_putreg_mem(priv, E1000_TDT, priv->tx_now);
  priv->tx_free -= 1;

  ninfodumpbuffer("Transmitted:", priv->dev.d_buf, priv->dev.d_len);
}

/*****************************************************************************
 * Name: e1000_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  /* Send the packet */

  e1000_transmit(dev);

  /* Check if there is room in the device to hold another packet.
   * If not, return a non-zero value to terminate the poll.
   */

  return priv->tx_free > 0;
}

/*****************************************************************************
 * Name: e1000_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return with an outgoing packet.  This function checks for that case
 *   and performs the transmission if necessary.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static void e1000_reply(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* And send the packet */

      e1000_transmit(dev);
    }
}

/*****************************************************************************
 * Name: e1000_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static void e1000_receive(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv   =
    (FAR struct e1000_driver_s *)dev->d_private;
  FAR struct e1000_rx_leg_s *rx     = NULL;
  FAR void                  *buf    = NULL;
  int                        desc   = 0;

  do
    {
      desc = priv->rx_now;

      /* Get descriptor */

      rx = &priv->rx[desc];

      /* Check if descriptor done */

      if (!(rx->status & RDESC_STATUS_DD))
        {
          break;
        }

      priv->rx_now = (priv->rx_now + 1) % E1000_RX_DESC;

      /* Reset descriptor status */

      rx->status = 0;

      /* Update RX tail */

      e1000_putreg_mem(priv, E1000_RDT, desc);

      /* Handle errros */

      if (rx->errors)
        {
          nerr("RX error reported (%"PRIu8")\n", rx->errors);
          priv->dev.d_len = 0;
          NETDEV_RXERRORS(&priv->dev);
          continue;
        }

      /* Read packet length (without CRC) */

      priv->dev.d_len = rx->len - 4;

      /* Get data */

      buf = up_addrenv_pa_to_va((uintptr_t)rx->addr);

      /* Copy the data data from the hardware to priv->dev.d_buf.  Set
       * amount of data in priv->dev.d_len
       */

      if (priv->dev.d_len <= CONFIG_NET_ETH_PKTSIZE)
        {
          memcpy(priv->dev.d_buf, buf, priv->dev.d_len);
        }

      /* Check for errors and update statistics */

      if (priv->dev.d_len > PKTBUF_SIZE ||
          priv->dev.d_len < ETH_HDRLEN)
        {
          nerr("Bad packet size dropped (%"PRIu16")\n", priv->dev.d_len);
          priv->dev.d_len = 0;
          NETDEV_RXERRORS(&priv->dev);
          continue;
        }

#ifdef CONFIG_DEBUG_NET_INFO
      ninfodumpbuffer("Received Packet:",
                      priv->dev.d_buf,
                      priv->dev.d_len);
#endif

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(&priv->dev);
#endif

#ifdef CONFIG_NET_IPv4
      /* Check for an IPv4 packet */

      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->dev);

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* Check for a reply to the IPv4 packet */

          e1000_reply(dev);
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      /* Check for an IPv6 packet */

      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(&priv->dev);

          /* Dispatch IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* Check for a reply to the IPv6 packet */

          e1000_reply(dev);
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      /* Check for an ARP packet */

      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Dispatch ARP packet to the network layer */

          arp_input(&priv->dev);
          NETDEV_RXARP(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value
           * > 0.
           */

          if (priv->dev.d_len > 0)
            {
              e1000_transmit(dev);
            }
        }
      else
#endif
        {
          ninfo("Dropped frame\n");

          NETDEV_RXDROPPED(&priv->dev);
        }
    }
  while (true); /* While there are more packets to be processed */
}

/*****************************************************************************
 * Name: e1000_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static void e1000_txdone(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  while (!(priv->tx_now == priv->tx_done))
    {
      if (priv->tx[priv->tx_done].status == 0)
        {
          break;
        }

      if (priv->tx[priv->tx_done].status & TDESC_STATUS_DD)
        {
          /* Check for errors and update statistics */

          NETDEV_TXDONE(priv->dev);
        }
      else
        {
          nerr("tx failed: 0x%" PRIx32 "\n", priv->tx[priv->tx_done].status);
          NETDEV_TXERRORS(priv->dev);
        }

      /* Next descriptor */

      priv->tx_done = (priv->tx_done + 1) % E1000_TX_DESC;
      priv->tx_free += 1;
    }

  /* In any event, poll the network for new TX data */

  devif_poll(&priv->dev, e1000_txpoll);
}

/*****************************************************************************
 * Name: e1000_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 *****************************************************************************/

static void e1000_interrupt_work(FAR void *arg)
{
  FAR struct e1000_driver_s *priv   = (FAR struct e1000_driver_s *)arg;
  uint32_t                   status = 0;

  status = e1000_getreg_mem(priv, E1000_ICR);
  ninfo("irq status = 0x%" PRIx32 "\n", status);

  if (status == 0)
    {
      /* Ignore spurious interrupts */

      goto out;
    }

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Receiver Timer Interrupt or Receive Descriptor Minimum Threshold Reached
   */

  if (status & E1000_IC_RXT0 ||
      status & E1000_IC_RXDMT0)
    {
      e1000_receive(&priv->dev);
    }

  /* Link Status Change */

  if (status & E1000_IC_LSC)
    {
      ninfo("Link Status Change\n");
    }

  /* Receiver Overrun */

  if (status & E1000_IC_RXO)
    {
      nerr("Receiver Overrun\n");
      e1000_receive(&priv->dev);
    }

  /* Transmit Descriptor Written Back */

  if (status & E1000_IC_TXDW)
    {
      e1000_txdone(&priv->dev);
    }

  net_unlock();

out:

  /* Enable interrupts back */

  e1000_putreg_mem(priv, E1000_IMS, E1000_INTERRUPTS);
}

/*****************************************************************************
 * Name: e1000_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 *****************************************************************************/

static int e1000_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct e1000_driver_s *priv   = (FAR struct e1000_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  ninfo("interrupt!\n");

  /* Disable interrupts until we handle work */

  e1000_putreg_mem(priv, E1000_IMC, E1000_INTERRUPTS);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, e1000_interrupt_work, priv, 0);
  return OK;
}

/*****************************************************************************
 * Name: e1000_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_ifup(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));
#endif

#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Enable the Ethernet */

  e1000_enable(priv);
  priv->bifup = true;

  return OK;
}

/*****************************************************************************
 * Name: e1000_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the e1000_ifup() always
   * successfully brings the interface back up.
   */

  e1000_reset(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

/*****************************************************************************
 * Name: e1000_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Runs on a work queue thread.
 *
 *****************************************************************************/

static void e1000_txavail_work(FAR void *arg)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another packet */

      /* If so, then poll the network for new XMIT data */

      devif_poll(&priv->dev, e1000_txpoll);
    }

  net_unlock();
}

/*****************************************************************************
 * Name: e1000_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_txavail(FAR struct net_driver_s *dev)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, e1000_txavail_work, dev, 0);
    }

  return OK;
}

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
/*****************************************************************************
 * Name: e1000_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 *   IEEE (CRC32) from http://www.microchip.com/wwwproducts/en/LAN91C111
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 *****************************************************************************/

static int e1000_addmac(FAR struct net_driver_s *dev,
                        FAR const uint8_t *mac)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  /* TODO: Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif  /* defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6) */

#ifdef CONFIG_NET_MCASTGROUP
/*****************************************************************************
 * Name: e1000_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 *****************************************************************************/

static int e1000_rmmac(FAR struct net_driver_s *dev,
                       FAR const uint8_t *mac)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  /* TODO: Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif  /* CONFIG_NET_MCASTGROUP */

#ifdef CONFIG_NETDEV_IOCTL
/*****************************************************************************
 * Name: e1000_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_ioctl(FAR struct net_driver_s *dev, int cmd,
                       unsigned long arg)
{
  FAR struct e1000_driver_s *priv =
    (FAR struct e1000_driver_s *)dev->d_private;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      /* Add cases here to support the IOCTL commands */

      default:
        {
          nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
          return -ENOTTY;
        }
    }

  return OK;
}
#endif

/*****************************************************************************
 * Name: e1000_reset
 *
 * Description:
 *   Reset device to known state.
 *
 *****************************************************************************/

static int e1000_reset(FAR struct e1000_driver_s *priv)
{
  /* Reset Tx tail */

  e1000_putreg_mem(priv, E1000_TDH, 0);
  e1000_putreg_mem(priv, E1000_TDT, 0);

  /* Reset Rx tail */

  e1000_putreg_mem(priv, E1000_RDH, 0);
  e1000_putreg_mem(priv, E1000_RDT, 0);

  /* Disable interrupts */

  e1000_putreg_mem(priv, E1000_IMC, E1000_INTERRUPTS);
  up_disable_irq(priv->type->irq);

  /* Disable Transmiter */

  e1000_putreg_mem(priv, E1000_TCTL, 0);

  /* Disable Receiver */

  e1000_putreg_mem(priv, E1000_RCTL, 0);

  return OK;
}

/*****************************************************************************
 * Name: e1000_enable
 *
 * Description:
 *   Enable device.
 *
 *****************************************************************************/

static int e1000_enable(FAR struct e1000_driver_s *priv)
{
  uint64_t pa        = 0;
  uint32_t regval    = 0;
  uint32_t fextnvm11 = 0;

  /* Errata for i219 - this is undocumented by Intel bug.
   * Linux and BSD drivers include this workaround which was optimised by
   * ipxe but no one except Intel engineers knows exactly wtha this does.
   * For some exaplanaition look at 546dd51de8459d4d09958891f426fa2c73ff090d
   * commit in ipxe.
   */

  if (priv->type->flags & E1000_RESET_BROKEN)
    {
      fextnvm11 = e1000_getreg_mem(priv, E1000_FEXTNVM11);
      fextnvm11 |= E1000_FEXTNVM11_MAGIC;
      e1000_putreg_mem(priv, E1000_FEXTNVM11, fextnvm11);
    }

  /* Setup TX descriptor */

  /* The address passed to the NIC must be physical */

  pa = up_addrenv_va_to_pa(priv->tx);

  regval = (uint32_t)pa;
  e1000_putreg_mem(priv, E1000_TDBAL, regval);
  regval = (uint32_t)(pa >> 32);
  e1000_putreg_mem(priv, E1000_TDBAH, regval);

  regval = E1000_TX_DESC * sizeof(struct e1000_tx_leg_s);
  e1000_putreg_mem(priv, E1000_TDLEN, regval);

  priv->tx_now  = 0;
  priv->tx_free = E1000_TX_DESC;

  /* Reset TX tail */

  e1000_putreg_mem(priv, E1000_TDH, 0);
  e1000_putreg_mem(priv, E1000_TDT, 0);

  /* Setup RX descriptor */

  /* The address passed to the NIC must be physical */

  pa = up_addrenv_va_to_pa(priv->rx);

  regval = (uint32_t)(pa & 0xffffffff);
  e1000_putreg_mem(priv, E1000_RDBAL, regval);
  regval = (uint32_t)(pa >> 32);
  e1000_putreg_mem(priv, E1000_RDBAH, regval);

  regval = E1000_RX_DESC * sizeof(struct e1000_rx_leg_s);
  e1000_putreg_mem(priv, E1000_RDLEN, regval);

  priv->rx_now = 0;

  /* Reset RX tail */

  e1000_putreg_mem(priv, E1000_RDH, 0);
  e1000_putreg_mem(priv, E1000_RDT, E1000_RX_DESC);

  /* Enable interrupts */

  e1000_putreg_mem(priv, E1000_IMS, E1000_INTERRUPTS);
  up_enable_irq(priv->type->irq);

  /* Set link up and auto-detect speed */

  regval = E1000_CTRL_SLU | E1000_CTRL_ASDE;
  e1000_putreg_mem(priv, E1000_CTRL, regval);

  /* Setup and enable Transmiter */

  regval = e1000_getreg_mem(priv, E1000_TCTL);
  regval |= E1000_TCTL_EN | E1000_TCTL_PSP;
  e1000_putreg_mem(priv, E1000_TCTL, regval);

  /* Setup and enable Receiver */

  regval = (E1000_RCTL_EN | E1000_RCTL_MPE |
            (E1000_RCTL_BSIZE << E1000_RCTL_BSIZE_SHIFT));
  e1000_putreg_mem(priv, E1000_RCTL, regval);

  /* REVISIT: Set granuality to Descriptors */

  regval = e1000_getreg_mem(priv, E1000_RXDCTL);
  regval |= E1000_TXDCTL_GRAN;
  e1000_putreg_mem(priv, E1000_RXDCTL, regval);

#ifdef CONFIG_DEBUG_NET_INFO
  /* Dump memory */

  e1000_dump_mem(priv, "enabled");
#endif

  return OK;
}

/*****************************************************************************
 * Name: e1000_initialize
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int e1000_initialize(FAR struct e1000_driver_s *priv)
{
  uint32_t regval = 0;
  uint64_t mac    = 0;
  int      ret    = OK;

  /* Attach IRQ */

  irq_attach(priv->type->irq, e1000_interrupt, priv);

  /* Configure MSI if supported */

  ret = pci_msi_connect(&priv->pcidev, priv->type->irq, 1);
  if (ret != OK)
    {
      nerr("Failed to connect MSI %d\n", ret);
      return -ENOTSUP;
    }

  /* Get MAC if valid */

  regval = e1000_getreg_mem(priv, E1000_RAH);
  if (regval & E1000_RAH_AV)
    {
      mac = ((uint64_t)regval & E1000_RAH_RAH_MASK) << 32;
      mac |= e1000_getreg_mem(priv, E1000_RAL);
      memcpy(&priv->dev.d_mac.ether, &mac, sizeof(struct ether_addr));
    }
  else
    {
      nwarn("Receive Address not vaild!\n");
    }

  return OK;
}

/*****************************************************************************
 * Name: e1000_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int e1000_probe(FAR struct pci_bus_s *bus,
                       FAR const struct pci_dev_type_s *type,
                       uint16_t bdf)
{
  FAR struct e1000_pci_type_s *drv_priv = NULL;
  FAR struct e1000_driver_s   *priv     = NULL;
  FAR struct net_driver_s     *netdev   = NULL;
  struct pci_dev_s            *pcidev   = NULL;
  uintptr_t                    bar_addr = 0;
  uint32_t                     bar      = 0;
  uint8_t                      bar_id   = 0;
  int                          ret      = 0;
  int                          i        = 0;
  uint8_t                     *buff     = NULL;
  uint64_t                     pa       = 0;

  /* Get priv data associated with this PCI device type */

  for (i = 0; i < sizeof(g_pci_e1000_devs) / sizeof(uintptr_t); i++)
    {
      if (type == g_pci_e1000_devs[i]->type)
        {
          drv_priv = g_pci_e1000_devs[i];
          break;
        }
    }

  /* Not found private data */

  if (drv_priv == NULL)
    {
      return -ENODEV;
    }

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Allocate TX descriptors */

  priv->tx = kmm_memalign(16, E1000_TX_DESC * sizeof(struct e1000_tx_leg_s));
  if (priv->tx == NULL)
    {
      nerr("alloc tx failed %d\n", errno);
      ret = -ENOMEM;
      goto errout;
    }

  /* Allocate RX descriptors */

  priv->rx = kmm_memalign(16, E1000_RX_DESC * sizeof(struct e1000_rx_leg_s));
  if (priv->rx == NULL)
    {
      nerr("alloc rx failed %d\n", errno);
      ret = -ENOMEM;
      goto errout;
    }

  for (i = 0; i < E1000_RX_DESC; i += 1)
    {
      buff = kmm_memalign(16, PKTBUF_SIZE);
      if (buff == NULL)
        {
          nerr("alloc rxbuf failed %d\n", errno);
          ret = -ENOMEM;
          goto errout;
        }

      pa = up_addrenv_va_to_pa(buff);

      priv->rx[i].addr   = pa;
      priv->rx[i].len    = 0;
      priv->rx[i].status = 0;
    }

  /* Allocate TX buffer */

  priv->txbuf = kmm_memalign(16, E1000_TX_DESC * PKTBUF_SIZE);
  if (priv->txbuf == NULL)
    {
      nerr("alloc txbuf failed %d\n", errno);
      ret = -ENOMEM;
      goto errout;
    }

  /* Allocate packet buffer */

  priv->pktbuf = kmm_memalign(16, E1000_TX_DESC * PKTBUF_SIZE);
  if (priv->pktbuf == NULL)
    {
      nerr("alloc pktbuf failed %d\n", errno);
      ret = -ENOMEM;
      goto errout;
    }

  /* Get devices */

  netdev     = &priv->dev;
  pcidev     = &priv->pcidev;
  priv->type = drv_priv;

  /* Get PCI dev */

  pcidev->bus  = bus;
  pcidev->type = type;
  pcidev->bdf  = bdf;

  pci_enable_bus_master(pcidev);
  pci_enable_io(pcidev, PCI_SYS_RES_MEM);
  pci_enable_io(pcidev, PCI_SYS_RES_IOPORT);

  /* Get MMIO BAR */

  bar_id = E1000_MMIO_BAR;

  if (pci_bar_valid(pcidev, bar_id) != OK)
    {
      ret = -ENODEV;
      goto errout;
    }

  bar = bus->ops->pci_cfg_read(pcidev,
                               PCI_HEADER_NORM_BAR0 + (bar_id * 4), 4);
  bar_addr = pci_bar_addr(pcidev, bar_id);
  if ((bar & PCI_BAR_LAYOUT_MASK) != PCI_BAR_LAYOUT_MEM)
    {
      pcierr("Invalid MMIO bar = 0x%" PRIx32 "\n", bar_addr);
      ret = -ENODEV;
      goto errout;
    }

  /* If the BAR is MMIO then it must be mapped */

  bus->ops->pci_map_bar(bar_addr, pci_bar_size(pcidev, bar_id));

  /* Store MMIO address */

  priv->mimobase = bar_addr;

  pciinfo("MMIO address = %p\n", bar_addr);

  /* Initialize the driver structure */

  netdev->d_buf     = (FAR uint8_t *)priv->pktbuf;
  netdev->d_ifup    = e1000_ifup;
  netdev->d_ifdown  = e1000_ifdown;
  netdev->d_txavail = e1000_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  netdev->d_addmac  = e1000_addmac;
  netdev->d_rmmac   = e1000_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  netdev->d_ioctl   = e1000_ioctl;
#endif
  netdev->d_private = priv;

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  ret = e1000_initialize(priv);
  if (ret != OK)
    {
      nerr("e1000_initialize failed %d\n", ret);
      return ret;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(netdev, NET_LL_ETHERNET);
  return OK;

errout:
  if (priv->tx)
    {
      kmm_free(priv->tx);
    }

  if (priv->rx)
    {
      kmm_free(priv->rx);
    }

  if (priv->pktbuf)
    {
      kmm_free(priv->pktbuf);
    }

  if (priv)
    {
      kmm_free(priv);
    }

  return ret;
}
