/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_tickless_grtc.c
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
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "arm_internal.h"
#include "hardware/nrf54l_grtc.h"
#include "nrf54l_grtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_TIMER_ARCH
#  error CONFIG_TIMER_ARCH must not be set
#endif

/* Allow the compare write to reach the counter before its deadline. A second
 * counter read below also catches preemption and bus delays.
 */

#define NRF54L_GRTC_ALARM_MARGIN 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_tickless_dev_s
{
  struct nrf54l_grtc_dev_s *grtc;
  bool alarm_set;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf54l_tickless_dev_s g_tickless_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: grtc_counter_to_ts
 ****************************************************************************/

static void grtc_counter_to_ts(uint64_t counter, struct timespec *now)
{
  now->tv_sec = counter / USEC_PER_SEC;
  now->tv_nsec = (counter % USEC_PER_SEC) * NSEC_PER_USEC;
}

/****************************************************************************
 * Name: grtc_cancel_ack
 ****************************************************************************/

static void grtc_cancel_ack(void)
{
  NRF54L_GRTC_DISABLEINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  NRF54L_GRTC_DISABLECC(g_tickless_dev.grtc, NRF54L_GRTC_CC0);
  NRF54L_GRTC_ACKINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  g_tickless_dev.alarm_set = false;
}

/****************************************************************************
 * Name: grtc_prepare_alarm
 ****************************************************************************/

static void grtc_prepare_alarm(uint64_t target)
{
  uint64_t now;

  /* A deadline already in the past must still produce an interrupt. Check
   * both time and the event after writing CC, so an event which arrived
   * during programming is never discarded.
   */

  for (; ; )
    {
      NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &now);
      if (target <= now + NRF54L_GRTC_ALARM_MARGIN)
        {
          target = now + NRF54L_GRTC_ALARM_MARGIN;
        }

      NRF54L_GRTC_SETCC(g_tickless_dev.grtc, NRF54L_GRTC_CC0, target);
      NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &now);
      if (target > now ||
          NRF54L_GRTC_CHECKINT(g_tickless_dev.grtc,
                              NRF54L_GRTC_EVT_COMPARE0) > 0)
        {
          break;
        }
    }

  NRF54L_GRTC_ENABLEINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
}

/****************************************************************************
 * Name: grtc_handler
 ****************************************************************************/

static int grtc_handler(int irq, void *context, void *arg)
{
  irqstate_t flags;
  bool alarm_set;

  flags = enter_critical_section();
  if (NRF54L_GRTC_CHECKINT(g_tickless_dev.grtc,
                          NRF54L_GRTC_EVT_COMPARE0) > 0)
    {
      alarm_set = g_tickless_dev.alarm_set;
      grtc_cancel_ack();
      if (alarm_set)
        {
          nxsched_process_timer();
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_alarm_cancel
 ****************************************************************************/

int up_alarm_cancel(struct timespec *ts)
{
  uint64_t counter;
  irqstate_t flags;

  flags = enter_critical_section();
  grtc_cancel_ack();
  NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &counter);
  leave_critical_section(flags);

  if (ts != NULL)
    {
      grtc_counter_to_ts(counter, ts);
    }

  return OK;
}

/****************************************************************************
 * Name: up_alarm_start
 ****************************************************************************/

int up_alarm_start(const struct timespec *ts)
{
  uint64_t target;
  irqstate_t flags;

  if (ts == NULL || ts->tv_sec < 0 || ts->tv_nsec < 0 ||
      ts->tv_nsec >= NSEC_PER_SEC ||
      (uint64_t)ts->tv_sec > GRTC_COUNTER_MAX / USEC_PER_SEC)
    {
      return -EINVAL;
    }

  target = (uint64_t)ts->tv_sec * USEC_PER_SEC +
           (ts->tv_nsec + NSEC_PER_USEC - 1) / NSEC_PER_USEC;
  if (target > GRTC_COUNTER_MAX - NRF54L_GRTC_ALARM_MARGIN)
    {
      return -ERANGE;
    }

  flags = enter_critical_section();
  grtc_cancel_ack();
  g_tickless_dev.alarm_set = true;
  grtc_prepare_alarm(target);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: up_timer_gettime
 ****************************************************************************/

int up_timer_gettime(struct timespec *ts)
{
  uint64_t counter;

  NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &counter);
  grtc_counter_to_ts(counter, ts);
  return OK;
}

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
  int ret;

  g_tickless_dev.grtc = nrf54l_grtc_init(0);
  ASSERT(g_tickless_dev.grtc != NULL);

  /* Start the counter before enabling the IRQ. A pending interrupt can
   * read the system time as soon as the handler is attached.
   */

  ret = NRF54L_GRTC_START(g_tickless_dev.grtc);
  ASSERT(ret == OK);

  ret = NRF54L_GRTC_SETISR(g_tickless_dev.grtc, grtc_handler, NULL);
  ASSERT(ret == OK);

  nxsched_process_timer();
}
