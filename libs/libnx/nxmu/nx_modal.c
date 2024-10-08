/****************************************************************************
 * libs/libnx/nxmu/nx_modal.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_modal
 *
 * Description:
 *   May be used to either (1) raise a window to the top of the display and
 *   select modal behavior, or (2) disable modal behavior.
 *
 * Input Parameters:
 *   hwnd  - The window to be modified
 *   modal - True: enter modal state; False: leave modal state
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_modal(NXWINDOW hwnd, bool modal)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_modal_s outmsg;

  /* Send the MODAL message */

  outmsg.msgid = NX_SVRMSG_MODAL;
  outmsg.wnd   = wnd;
  outmsg.modal = modal;

  return nxmu_sendwindow(wnd, &outmsg, sizeof(struct nxsvrmsg_modal_s));
}
