/****************************************************************************
 * net/netforward/netfwd_poll.c
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

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "sixlowpan/sixlowpan.h"
#include "netforward/netforward.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netfwd_dequeue
 *
 * Description:
 *   Remove the oldest packet queued for forwarding on the device.
 *
 ****************************************************************************/

static FAR struct forward_s *netfwd_dequeue(FAR struct net_driver_s *dev)
{
  FAR struct forward_s *fwd;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->d_fwdlock);
  fwd = (FAR struct forward_s *)sq_remfirst(&dev->d_fwdq);
  if (fwd != NULL)
    {
      dev->d_fwdcount--;
    }

  spin_unlock_irqrestore(&dev->d_fwdlock, flags);

  return fwd;
}

/****************************************************************************
 * Name: forward_ipselect
 *
 * Description:
 *   If both IPv4 and IPv6 support are enabled, then we will need to select
 *   which one to use when generating the outgoing packet.  If only one
 *   domain is selected, then the setup is already in place and we need do
 *   nothing.
 *
 * Input Parameters:
 *   fwd - The forwarding state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device is locked.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
static inline void forward_ipselect(FAR struct forward_s *fwd)
{
  FAR struct net_driver_s *dev = fwd->f_dev;

  /* Select IPv4 or IPv6 */

  if (fwd->f_domain == PF_INET)
    {
      /* Clear a bit in the d_flags to distinguish this from an IPv6 packet */

      IFF_SET_IPv4(dev->d_flags);

      /* Set the offset to the beginning of the UDP data payload */

      dev->d_appdata = IPBUF(IPv4UDP_HDRLEN);
    }
  else if (fwd->f_domain == PF_INET6)
    {
      /* Set a bit in the d_flags to distinguish this from an IPv6 packet */

      IFF_SET_IPv6(dev->d_flags);

      /* Set the offset to the beginning of the UDP data payload */

      dev->d_appdata = IPBUF(IPv6UDP_HDRLEN);
    }
}
#endif

/****************************************************************************
 * Name: ipfwd_packet_proto
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 (and
 *   other, non-standard packet radios) for now but this is a point where
 *   support for other conversions may be provided.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static int ipfwd_packet_proto(FAR struct net_driver_s *dev)
{
  FAR struct ipv6_hdr_s *ipv6;

  /* Make sure the there is something in buffer that is at least as large as
   * the IPv6_HDR.
   */

  if (dev->d_len > (IPv6_HDRLEN + NET_LL_HDRLEN(dev)))
    {
      if (dev->d_lltype == NET_LL_IEEE802154 ||
          dev->d_lltype == NET_LL_PKTRADIO)
        {
          /* There should be an IPv6 packet at the beginning of the buffer */

          ipv6 = IPv6BUF;
          if ((ipv6->vtc & IP_VERSION_MASK) == IPv6_VERSION)
            {
              /* Yes.. return the L2 protocol of the packet */

              return ipv6->proto;
            }
        }
    }

  return -EPROTO;
}
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Name: ipfwd_packet_conversion
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 (and
 *   other, non-standard packet radios) for now but this is a point where
 *   support for other conversions may be provided.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static void ipfwd_packet_conversion(FAR struct net_driver_s *dev, int proto)
{
  if (dev->d_len > 0)
    {
      /* Check if this is a device served by 6LoWPAN */

      if (dev->d_lltype == NET_LL_IEEE802154 ||
          dev->d_lltype == NET_LL_PKTRADIO)
        {
          FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)dev->d_buf;

#ifdef CONFIG_NET_TCP
          if (proto == IP_PROTO_TCP)
            {
              /* Let 6LoWPAN convert IPv6 TCP output into radio frames. */

              sixlowpan_tcp_send(dev, dev, ipv6);
            }
          else
#endif
#ifdef CONFIG_NET_UDP
          if (proto == IP_PROTO_UDP)
            {
              /* Let 6LoWPAN convert IPv6 UDP output into radio frames. */

              sixlowpan_udp_send(dev, dev, ipv6);
            }
          else
#endif
#ifdef CONFIG_NET_ICMPv6
          if (proto == IP_PROTO_ICMP6)
            {
              /* Let 6LoWPAN convert IPv6 UDP output into radio frames. */

              sixlowpan_icmpv6_send(dev, dev, ipv6);
            }
          else
#endif
            {
              nwarn("WARNING: Unsupported protocol (%u). Packet dropped\n",
                    proto);
            }

          dev->d_len = 0;
        }
    }
}
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netfwd_poll
 *
 * Description:
 *   Move the next packet queued for forwarding on the device into the
 *   device buffer.
 *
 * Returned Value:
 *   True if a packet was dequeued, false if the queue is empty.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll() with the device locked.
 *
 ****************************************************************************/

bool netfwd_poll(FAR struct net_driver_s *dev)
{
  FAR struct forward_s *fwd;
#ifdef CONFIG_NET_6LOWPAN
  int proto;
#endif

  fwd = netfwd_dequeue(dev);
  if (fwd == NULL)
    {
      return false;
    }

  DEBUGASSERT(fwd->f_dev == dev && fwd->f_iob != NULL);

  /* Setup for the send (most of these do not apply) */

  dev->d_appdata = NULL;
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Move the IOB chain of the packet into the device buffer */

  devif_forward(fwd);

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  /* If both IPv4 and IPv6 support are enabled, then we will need to
   * select which one to use when generating the outgoing packet.
   * If only one domain is selected, then the setup is already in
   * place and we need do nothing.
   */

  forward_ipselect(fwd);
#endif

  /* And release the forwarding state structure */

  netfwd_free(fwd);

#ifdef CONFIG_NET_6LOWPAN
  /* Get the L2 protocol of packet in the device's d_buf */

  proto = ipfwd_packet_proto(dev);
  if (proto >= 0)
    {
      /* Perform any necessary conversions on the forwarded packet */

      ipfwd_packet_conversion(dev, proto);
    }
#endif

  return true;
}

/****************************************************************************
 * Name: netfwd_cleanup
 *
 * Description:
 *   Drop all packets queued for forwarding on the device, which is going
 *   down or being unregistered.
 *
 * Input Parameters:
 *   dev    - The network device
 *   closed - The device is unregistered, queue nothing more on it
 *
 ****************************************************************************/

void netfwd_cleanup(FAR struct net_driver_s *dev, bool closed)
{
  FAR struct forward_s *fwd;
  irqstate_t flags;

  if (closed)
    {
      flags = spin_lock_irqsave(&dev->d_fwdlock);
      dev->d_fwdclosed = true;
      spin_unlock_irqrestore(&dev->d_fwdlock, flags);
    }

  while ((fwd = netfwd_dequeue(dev)) != NULL)
    {
      nwarn("WARNING: Network is down... Dropping\n");
      netfwd_dropstats(fwd);
      iob_free_chain(fwd->f_iob);
      netfwd_free(fwd);
    }
}
