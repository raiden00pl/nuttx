/****************************************************************************
 * net/ipforward/ipfwd_poll.c
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
#include <nuttx/debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>

#include "devif/devif.h"
#include "ipforward/ipforward.h"

#ifdef CONFIG_NET_IPFORWARD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfwd_poll
 *
 * Description:
 *   Poll all pending transfer for ARP requests to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll().
 *
 ****************************************************************************/

void ipfwd_poll(FAR struct net_driver_s *dev)
{
  /* Setup for the callback (most of these do not apply) */

  dev->d_appdata = NULL;
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Perform the forwarding callbacks */

  devif_conn_event(dev, IPFWD_POLL, dev->d_conncb);
}

#endif /* CONFIG_NET_ARP_SEND */
