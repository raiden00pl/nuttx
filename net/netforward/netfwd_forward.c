/****************************************************************************
 * net/netforward/netfwd_forward.c
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

#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <net/if.h>

#include <nuttx/spinlock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netforward/netforward.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netfwd_forward
 *
 * Description:
 *   Called by the IP forwarding logic when a packet is received on one
 *   network device, but must be forwarded on another network device.
 *
 *   Queue the packet on the forwarding device and notify its driver.  The
 *   forwarding device sends the packet asynchronously from its own poll,
 *   so this function returns without waiting for the send to complete.
 *   The caller must not access fwd after a successful return.
 *
 * Input Parameters:
 *   dev - The device on which the packet was received.
 *   fwd - An initialized instance of the common forwarding structure that
 *         includes everything needed to perform the forwarding operation.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forwarded;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller should free the IOB list and drop the packet.
 *
 ****************************************************************************/

int netfwd_forward(FAR struct net_driver_s *dev, FAR struct forward_s *fwd)
{
  FAR struct net_driver_s *fwddev;
  FAR struct iob_s *iob;
  FAR uint8_t *buf;
  irqstate_t flags;
  int len;

  DEBUGASSERT(fwd != NULL && fwd->f_iob != NULL && fwd->f_dev != NULL);
  fwddev = fwd->f_dev;

  /* Queue the packet on the forwarding device.  Its own poll sends the
   * packet with its own lock held, see netfwd_poll().  A device that is
   * not running or is unregistered gets nothing queued, see
   * netfwd_cleanup().  The queue is bounded, so that packets waiting for a
   * slower device do not take all IOBs.
   */

  flags = spin_lock_irqsave(&fwddev->d_fwdlock);
  if (fwddev->d_fwdclosed || !IFF_IS_RUNNING(fwddev->d_flags))
    {
      spin_unlock_irqrestore(&fwddev->d_fwdlock, flags);
      return -ENETDOWN;
    }

  if (fwddev->d_fwdcount >= CONFIG_NET_FORWARD_NSTRUCT)
    {
      spin_unlock_irqrestore(&fwddev->d_fwdlock, flags);
      return -EBUSY;
    }

  sq_addlast((FAR sq_entry_t *)fwd, &fwddev->d_fwdq);
  fwddev->d_fwdcount++;
  spin_unlock_irqrestore(&fwddev->d_fwdlock, flags);

  /* Save the current state of the device, avoid damaging the current
   * d_iob and other resources when forwarding recursion
   */

  iob = dev->d_iob;
  buf = dev->d_buf;
  len = dev->d_len;

  dev->d_iob = NULL;
  dev->d_buf = NULL;
  dev->d_len = 0;

  /* Notify the device driver of the availability of TX data.  fwd may
   * already be sent and freed by the forwarding device.
   */

  if (fwddev->d_txavail != NULL)
    {
      fwddev->d_txavail(fwddev);
    }

  dev->d_iob = iob;
  dev->d_buf = buf;
  dev->d_len = len;

  return OK;
}
