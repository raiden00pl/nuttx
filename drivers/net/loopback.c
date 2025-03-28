/****************************************************************************
 * drivers/net/loopback.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/loopback.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#ifdef CONFIG_NET_LOOPBACK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We need to have the work queue to handle interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lo_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lo_driver_s
{
  bool lo_bifup;               /* true:ifup false:ifdown */
  struct work_s lo_work;       /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s lo_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lo_driver_s g_loopback;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* NuttX callback functions */

static int lo_ifup(FAR struct net_driver_s *dev);
static int lo_ifdown(FAR struct net_driver_s *dev);
static void lo_txavail_work(FAR void *arg);
static int lo_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lo_ifup
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
 *
 ****************************************************************************/

static int lo_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        NTOHS(dev->d_ipv6addr[0]), NTOHS(dev->d_ipv6addr[1]),
        NTOHS(dev->d_ipv6addr[2]), NTOHS(dev->d_ipv6addr[3]),
        NTOHS(dev->d_ipv6addr[4]), NTOHS(dev->d_ipv6addr[5]),
        NTOHS(dev->d_ipv6addr[6]), NTOHS(dev->d_ipv6addr[7]));
#endif

  priv->lo_bifup = true;
  netdev_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: lo_ifdown
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
 *
 ****************************************************************************/

static int lo_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  netdev_carrier_off(dev);

  /* Mark the device "down" */

  priv->lo_bifup = false;
  return OK;
}

/****************************************************************************
 * Name: lo_txavail_work
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
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void lo_txavail_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->lo_bifup)
    {
      /* Reuse the devif_loopback() logic, Polling all pending events until
       * return stop
       */

      while (devif_poll(&priv->lo_dev, NULL));
    }

  net_unlock();
}

/****************************************************************************
 * Name: lo_txavail
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
 *   Called in normal user mode
 *
 ****************************************************************************/

static int lo_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->lo_work))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPWORK, &priv->lo_work, lo_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lo_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Name: lo_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: localhost_initialize
 *
 * Description:
 *   Initialize the localhost, loopback network driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int localhost_initialize(void)
{
  FAR struct lo_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  priv = &g_loopback;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lo_driver_s));
  priv->lo_dev.d_ifup    = lo_ifup;      /* I/F up (new IP address) callback */
  priv->lo_dev.d_ifdown  = lo_ifdown;    /* I/F down callback */
  priv->lo_dev.d_txavail = lo_txavail;   /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->lo_dev.d_addmac  = lo_addmac;    /* Add multicast MAC address */
  priv->lo_dev.d_rmmac   = lo_rmmac;     /* Remove multicast MAC address */
#endif
  priv->lo_dev.d_private = priv;         /* Used to recover private state from dev */

  /* Register the loopabck device with the OS so that socket IOCTLs can b
   * performed.
   */

  netdev_register(&priv->lo_dev, NET_LL_LOOPBACK);

  /* Set the local loopback IP address */

#ifdef CONFIG_NET_IPv4
  net_ipv4addr_copy(priv->lo_dev.d_ipaddr, g_lo_ipv4addr);
  net_ipv4addr_copy(priv->lo_dev.d_draddr, g_lo_ipv4addr);
  net_ipv4addr_copy(priv->lo_dev.d_netmask, g_lo_ipv4mask);
#endif

#ifdef CONFIG_NET_IPv6
  net_ipv6addr_copy(priv->lo_dev.d_ipv6addr, g_lo_ipv6addr);
  net_ipv6addr_copy(priv->lo_dev.d_ipv6draddr, g_lo_ipv6addr);
  net_ipv6addr_copy(priv->lo_dev.d_ipv6netmask, g_lo_ipv6mask);
#endif

  /* Put the network in the UP state */

  IFF_SET_UP(priv->lo_dev.d_flags);
  return lo_ifup(&priv->lo_dev);
}

#endif /* CONFIG_NET_LOOPBACK */
