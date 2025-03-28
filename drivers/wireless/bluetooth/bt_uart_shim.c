/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_shim.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/serial/tioctl.h>
#include <nuttx/wireless/bluetooth/bt_uart_shim.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is the variable state of the binding to the UART */

struct hciuart_state_s
{
  /* Registered Rx callback */

  btuart_rxcallback_t callback; /* Rx callback function */
  FAR void *arg;                /* Rx callback argument */

  struct file f;                /* File structure */
  struct pollfd p;              /* Poll structure */
  spinlock_t lock;              /* Spinlock */
};

struct hciuart_config_s
{
  /* Setup the interface from the upper to the lower */

  struct btuart_lowerhalf_s lower;    /* Generic UART lower half */
  struct hciuart_state_s state;       /* Variable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* UART Lower-Half Methods */

static void hciuart_rxattach(FAR const struct btuart_lowerhalf_s *lower,
                             btuart_rxcallback_t callback, FAR void *arg);
static void hciuart_rxpollcb(FAR struct pollfd *fds);
static void hciuart_rxenable(FAR const struct btuart_lowerhalf_s *lower,
                             bool enable);
static int hciuart_setbaud(FAR const struct btuart_lowerhalf_s *lower,
                           uint32_t baud);
static ssize_t hciuart_read(FAR const struct btuart_lowerhalf_s *lower,
                            FAR void *buffer, size_t buflen);
static ssize_t hciuart_write(FAR const struct btuart_lowerhalf_s *lower,
                             FAR const void *buffer, size_t buflen);
static ssize_t hciuart_rxdrain(FAR const struct btuart_lowerhalf_s *lower);
static int hciuart_ioctl(FAR const struct btuart_lowerhalf_s *lower,
                         int cmd, unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_rxattach
 *
 * Description:
 *   Attach/detach the upper half Rx callback.
 *
 *   rxattach() allows the upper half logic to attach a callback function
 *   that will be used to inform the upper half that an Rx frame is
 *   available.  This callback will, most likely, be invoked in the
 *   context of an interrupt callback.  The receive() method should then
 *   be invoked in order to receive the obtain the Rx frame data.
 *
 ****************************************************************************/

static void
hciuart_rxattach(FAR const struct btuart_lowerhalf_s *lower,
                 btuart_rxcallback_t callback, FAR void *arg)
{
  struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  struct hciuart_state_s *state;
  irqstate_t flags;

  state = &config->state;

  /* If the callback is NULL, then we are detaching */

  flags = spin_lock_irqsave(&state->lock);
  if (callback == NULL)
    {
      /* Disable Rx callbacks and detach the Rx callback */

      state->callback = NULL;
      state->arg = NULL;
    }

  /* Otherwise, we are attaching */

  else
    {
      state->arg = arg;
      state->callback = callback;
    }

  spin_unlock_irqrestore(&state->lock, flags);
}

/****************************************************************************
 * Name: hciuart_rxpollcb
 *
 * Description:
 *   Callback to receive the UART driver POLLIN notification.
 *
 ****************************************************************************/

static void hciuart_rxpollcb(FAR struct pollfd *fds)
{
  FAR struct hciuart_config_s *n = (FAR struct hciuart_config_s *)fds->arg;
  FAR struct hciuart_state_s *s = &n->state;

  if (fds->revents & POLLIN)
    {
      fds->revents = 0;
      if (s->callback != NULL)
        {
          wlinfo("Activating callback\n");
          s->callback(&n->lower, s->arg);
        }
      else
        {
          wlwarn("Dropping data (no CB)\n");
          hciuart_rxdrain(&n->lower);
        }
    }
}

/****************************************************************************
 * Name: hciuart_rxenable
 *
 * Description:
 *   Enable/disable RX callbacks from the HCI UART.
 *
 *   hciuart_rxenable() may be used to enable or disable callback events.
 *   This probably translates to enabling and disabled Rx interrupts at
 *   the UART.  NOTE:  Rx event notification should be done sparingly:
 *   Rx data overrun may occur when Rx events are disabled!
 *
 ****************************************************************************/

static void hciuart_rxenable(FAR const struct btuart_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *s = &config->state;

  if (enable != !!s->p.priv)
    {
      file_poll(&s->f, &s->p, enable);
    }
}

/****************************************************************************
 * Name: hciuart_setbaud
 *
 * Description:
 *   The HCI UART comes up with some initial BAUD rate.  Some support
 *   auto-BAUD detection, some support writing a configuration file to
 *   select the initial BAUD.  The simplest strategy, however, is simply
 *   to use the HCI UART's default initial BAUD to perform the basic
 *   bring up, then send a vendor-specific command to increase the HCI
 *   UARTs BAUD.  This method then may be used to adjust the lower half
 *   driver to the new HCI UART BAUD.
 *
 ****************************************************************************/

static int
hciuart_setbaud(FAR const struct btuart_lowerhalf_s *lower, uint32_t baud)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct termios tio;
  int ret;

  ret = hciuart_ioctl(lower, TCGETS, (unsigned long)&tio);
  if (ret)
    {
      wlerr("ERROR during TCGETS\n");
      return ret;
    }

  if (baud != 0)
    {
      cfsetspeed(&tio, baud);
    }

  /* To be a H4 interface, CTS/RTS are needed */

  tio.c_cflag |= CRTS_IFLOW | CCTS_OFLOW;

  ret = hciuart_ioctl(lower, TCSETS, (unsigned long)&tio);
  if (ret)
    {
      wlerr("ERROR during TCSETS, does UART support CTS/RTS?\n");
      return ret;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: hciuart_read
 *
 * Description:
 *   Read UART data.
 *
 *   hciuart_read() after receipt of a callback notifying the upper half of
 *   the availability of Rx frame, the upper half may call the receive()
 *   method in order to obtain the buffered Rx frame data.
 *
 ****************************************************************************/

static ssize_t
hciuart_read(FAR const struct btuart_lowerhalf_s *lower,
             FAR void *buffer, size_t buflen)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *state = &config->state;

  wlinfo("config %p buffer %p buflen %zu\n", config, buffer, buflen);

  /* NOTE: This assumes that the caller has exclusive access to the Rx
   * buffer, i.e., one lower half instance can server only one upper half!
   */

  return file_read(&state->f, buffer, buflen);
}

/****************************************************************************
 * Name: hciuart_write
 *
 * Description:
 *   Write UART data.
 *
 *   hciuart_write() will add the outgoing frame to the Tx buffer and will
 *   return immediately.  This function may block only in the event that
 *   there is insufficient buffer space to hold the Tx frame data.  In that
 *   case the lower half will block until there is sufficient to buffer
 *   the entire outgoing packet.
 *
 ****************************************************************************/

static ssize_t
hciuart_write(FAR const struct btuart_lowerhalf_s *lower,
              FAR const void *buffer, size_t buflen)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *state = &config->state;

  wlinfo("config %p buffer %p buflen %zu\n", config, buffer, buflen);

  return file_write(&state->f, buffer, buflen);
}

/****************************************************************************
 * Name: hciuart_rxdrain
 *
 * Description:
 *   Flush/drain all buffered RX data
 *
 ****************************************************************************/

static ssize_t hciuart_rxdrain(FAR const struct btuart_lowerhalf_s *lower)
{
  return hciuart_ioctl(lower, TCDRN, 0);
}

/****************************************************************************
 * Name: hciuart_ioctl
 ****************************************************************************/

static int hciuart_ioctl(FAR const struct btuart_lowerhalf_s *lower,
                         int cmd, unsigned long arg)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *s = &config->state;

  return file_ioctl(&s->f, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btuart_shim_getdevice
 *
 * Description:
 *   Get a pointer to the device that will be used to communicate with the
 *   regular serial port on the HCI.
 *
 * Input Parameters:
 *   Entry in filesystem hierarchy for device
 *
 * Returned Value:
 *   Pointer to device interface
 *
 ****************************************************************************/

FAR struct btuart_lowerhalf_s *btuart_shim_getdevice(FAR const char *path)
{
  FAR struct hciuart_config_s *n;
  FAR struct hciuart_state_s *s;
  int ret;

  /* Get the memory for this shim instance */

  n = (FAR struct hciuart_config_s *)
    kmm_zalloc(sizeof(struct hciuart_config_s));

  if (!n)
    {
      return NULL;
    }

  s = &n->state;

  ret = file_open(&s->f, path, O_RDWR | O_CLOEXEC | O_NONBLOCK);
  if (ret < 0)
    {
      kmm_free(n);
      return NULL;
    }

  /* Setup poll structure */

  s->p.events = POLLIN;
  s->p.arg    = n;
  s->p.cb     = hciuart_rxpollcb;
  spin_lock_init(&s->lock);

  /* Hook the routines in */

  n->lower.rxattach = hciuart_rxattach;
  n->lower.rxenable = hciuart_rxenable;
  n->lower.setbaud  = hciuart_setbaud;
  n->lower.read     = hciuart_read;
  n->lower.write    = hciuart_write;
  n->lower.rxdrain  = hciuart_rxdrain;
  n->lower.ioctl    = hciuart_ioctl;

  return (FAR struct btuart_lowerhalf_s *)n;
}
