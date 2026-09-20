/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_grtc.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/nrf54l_grtc.h"
#include "nrf54l_grtc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_grtc_priv_s
{
  struct nrf54l_grtc_ops_s *ops;
  uint32_t                base;
  uint32_t                irq;
  uint8_t                 chan;
  bool                    inuse;
  bool                    started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf54l_grtc_start(struct nrf54l_grtc_dev_s *dev);
static int nrf54l_grtc_stop(struct nrf54l_grtc_dev_s *dev);
static int nrf54l_grtc_clear(struct nrf54l_grtc_dev_s *dev);
static int nrf54l_grtc_getcounter(struct nrf54l_grtc_dev_s *dev,
                                uint64_t *cc);
static int nrf54l_grtc_setcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                           uint64_t cc);
static int nrf54l_grtc_getcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                           uint64_t *cc);
static int nrf54l_grtc_disablecc(struct nrf54l_grtc_dev_s *dev, uint8_t i);
static int nrf54l_grtc_setisr(struct nrf54l_grtc_dev_s *dev, xcpt_t handler,
                            void *arg);
static int nrf54l_grtc_enableint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static int nrf54l_grtc_disableint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static int nrf54l_grtc_checkint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static int nrf54l_grtc_ackint(struct nrf54l_grtc_dev_s *dev, uint8_t s);
static uint32_t nrf54l_grtc_getbase(struct nrf54l_grtc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf54l_grtc_ops_s nrf54l_grtc_ops =
{
  .start      = nrf54l_grtc_start,
  .stop       = nrf54l_grtc_stop,
  .clear      = nrf54l_grtc_clear,
  .getcounter = nrf54l_grtc_getcounter,
  .setcc      = nrf54l_grtc_setcc,
  .getcc      = nrf54l_grtc_getcc,
  .disablecc  = nrf54l_grtc_disablecc,
  .setisr     = nrf54l_grtc_setisr,
  .enableint  = nrf54l_grtc_enableint,
  .disableint = nrf54l_grtc_disableint,
  .checkint   = nrf54l_grtc_checkint,
  .ackint     = nrf54l_grtc_ackint,
  .getbase    = nrf54l_grtc_getbase,
};

static struct nrf54l_grtc_priv_s g_nrf54l_grtc_priv =
{
  .ops   = &nrf54l_grtc_ops,
  .base  = NRF54L_GRTC_BASE,
  .irq   = NRF54L_IRQ_GRTC_0,
  .chan  = 12,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_grtc_getreg
 ****************************************************************************/

static uint32_t nrf54l_grtc_getreg(struct nrf54l_grtc_dev_s *dev,
                                 uint32_t offset)
{
  return getreg32(((struct nrf54l_grtc_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf54l_grtc_putreg
 ****************************************************************************/

static void nrf54l_grtc_putreg(struct nrf54l_grtc_dev_s *dev,
                             uint32_t offset, uint32_t value)
{
  putreg32(value, ((struct nrf54l_grtc_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf54l_grtc_waitready
 ****************************************************************************/

static void nrf54l_grtc_waitready(struct nrf54l_grtc_dev_s *dev)
{
  /* START, STOP and CLEAR must wait for the previous LF task to finish. */

  while ((nrf54l_grtc_getreg(dev, NRF54L_GRTC_STATUS_LFTIMER_OFFSET) &
          GRTC_STATUS_LFTIMER_READY) == 0)
    {
    }
}

/****************************************************************************
 * Name: nrf54l_grtc_start
 ****************************************************************************/

static int nrf54l_grtc_start(struct nrf54l_grtc_dev_s *dev)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  uint64_t counter;

  if (grtc->started)
    {
      return -EBUSY;
    }

  nrf54l_grtc_waitready(dev);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_TASKS_START_OFFSET, GRTC_TASKS_START);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_MODE_OFFSET, GRTC_MODE_SYSCOUNTEREN);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_SYSCOUNTER_ACTIVE_OFFSET(0),
                   GRTC_SYSCOUNTER_ACTIVE);

  /* Reading the low word initiates synchronization of the snapshot. */

  nrf54l_grtc_getcounter(dev, &counter);

  grtc->started = true;
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_stop
 ****************************************************************************/

static int nrf54l_grtc_stop(struct nrf54l_grtc_dev_s *dev)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_SYSCOUNTER_ACTIVE_OFFSET(0), 0);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_MODE_OFFSET, 0);
  nrf54l_grtc_waitready(dev);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_TASKS_STOP_OFFSET, GRTC_TASKS_STOP);
  grtc->started = false;
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_clear
 ****************************************************************************/

static int nrf54l_grtc_clear(struct nrf54l_grtc_dev_s *dev)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  if (grtc->started)
    {
      return -EBUSY;
    }

  nrf54l_grtc_waitready(dev);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_TASKS_CLEAR_OFFSET, GRTC_TASKS_CLEAR);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_getcounter
 ****************************************************************************/

static int nrf54l_grtc_getcounter(struct nrf54l_grtc_dev_s *dev,
                                uint64_t *ctr)
{
  irqstate_t flags;
  uint32_t low;
  uint32_t high;

  DEBUGASSERT(dev && ctr);

  /* Keep readers of the application domain's snapshot together. Retry when
   * synchronization is busy or the low word rolled over between reads.
   */

  flags = enter_critical_section();
  do
    {
      low = nrf54l_grtc_getreg(dev, NRF54L_GRTC_SYSCOUNTERL_OFFSET(0));
      high = nrf54l_grtc_getreg(dev, NRF54L_GRTC_SYSCOUNTERH_OFFSET(0));
    }
  while ((high & (GRTC_SYSCOUNTERH_BUSY | GRTC_SYSCOUNTERH_OVERFLOW)) != 0);

  *ctr = ((uint64_t)(high & GRTC_SYSCOUNTERH_MASK) << 32) | low;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_setcc
 ****************************************************************************/

static int nrf54l_grtc_setcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                           uint64_t cc)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  irqstate_t flags;

  if (i >= grtc->chan || cc > GRTC_COUNTER_MAX)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCEN_OFFSET(i), 0);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCL_OFFSET(i), (uint32_t)cc);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCH_OFFSET(i), (uint32_t)(cc >> 32));
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCEN_OFFSET(i), GRTC_CCEN_ACTIVE);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_getcc
 ****************************************************************************/

static int nrf54l_grtc_getcc(struct nrf54l_grtc_dev_s *dev, uint8_t i,
                           uint64_t *cc)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  irqstate_t flags;
  uint32_t high;

  DEBUGASSERT(cc);
  if (i >= grtc->chan)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  high = nrf54l_grtc_getreg(dev, NRF54L_GRTC_CCH_OFFSET(i));
  *cc = ((uint64_t)(high & GRTC_CCH_MASK) << 32) |
        nrf54l_grtc_getreg(dev, NRF54L_GRTC_CCL_OFFSET(i));
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_disablecc
 ****************************************************************************/

static int nrf54l_grtc_disablecc(struct nrf54l_grtc_dev_s *dev, uint8_t i)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  if (i >= grtc->chan)
    {
      return -EINVAL;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CCEN_OFFSET(i), 0);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_setisr
 ****************************************************************************/

static int nrf54l_grtc_setisr(struct nrf54l_grtc_dev_s *dev, xcpt_t handler,
                            void *arg)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  int ret;

  if (handler == NULL)
    {
      up_disable_irq(grtc->irq);
      irq_detach(grtc->irq);
      return OK;
    }

  ret = irq_attach(grtc->irq, handler, arg);
  if (ret == OK)
    {
      up_enable_irq(grtc->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_grtc_enableint
 ****************************************************************************/

static int nrf54l_grtc_enableint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  if (s >= grtc->chan)
    {
      return -EINVAL;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTENSET_OFFSET(0),
                   GRTC_INT_COMPARE(s));
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_disableint
 ****************************************************************************/

static int nrf54l_grtc_disableint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  if (s >= grtc->chan)
    {
      return -EINVAL;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTENCLR_OFFSET(0),
                   GRTC_INT_COMPARE(s));
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_checkint
 ****************************************************************************/

static int nrf54l_grtc_checkint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  if (s >= grtc->chan)
    {
      return -EINVAL;
    }

  return nrf54l_grtc_getreg(dev, NRF54L_GRTC_EVENTS_COMPARE_OFFSET(s));
}

/****************************************************************************
 * Name: nrf54l_grtc_ackint
 ****************************************************************************/

static int nrf54l_grtc_ackint(struct nrf54l_grtc_dev_s *dev, uint8_t s)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;

  if (s >= grtc->chan)
    {
      return -EINVAL;
    }

  nrf54l_grtc_putreg(dev, NRF54L_GRTC_EVENTS_COMPARE_OFFSET(s), 0);
  nrf54l_grtc_getreg(dev, NRF54L_GRTC_EVENTS_COMPARE_OFFSET(s));
  return OK;
}

/****************************************************************************
 * Name: nrf54l_grtc_getbase
 ****************************************************************************/

static uint32_t nrf54l_grtc_getbase(struct nrf54l_grtc_dev_s *dev)
{
  return ((struct nrf54l_grtc_priv_s *)dev)->base;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_grtc_init
 *
 * Description:
 *   Initialize the application GRTC instance using interrupt group zero.
 *
 ****************************************************************************/

struct nrf54l_grtc_dev_s *nrf54l_grtc_init(int grtc)
{
  struct nrf54l_grtc_priv_s *priv = &g_nrf54l_grtc_priv;
  struct nrf54l_grtc_dev_s *dev = (struct nrf54l_grtc_dev_s *)priv;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();
  if (grtc != 0 || priv->inuse)
    {
      leave_critical_section(flags);
      return NULL;
    }

  priv->inuse = true;
  nrf54l_grtc_stop(dev);
  nrf54l_grtc_setisr(dev, NULL, NULL);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTENCLR_OFFSET(0), 0xffffffff);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_SHORTS_OFFSET, 0);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTERVAL_OFFSET, 0);
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_CLKCFG_OFFSET,
                   GRTC_CLKCFG_CLKSEL_LFLPRC | GRTC_CLKCFG_CLKFASTDIV(1));

  for (i = 0; i < priv->chan; i++)
    {
      nrf54l_grtc_disablecc(dev, i);
      nrf54l_grtc_ackint(dev, i);
      nrf54l_grtc_putreg(dev, NRF54L_GRTC_SUBSCRIBE_CAPTURE_OFFSET(i), 0);
      nrf54l_grtc_putreg(dev, NRF54L_GRTC_PUBLISH_COMPARE_OFFSET(i), 0);
    }

  nrf54l_grtc_clear(dev);
  leave_critical_section(flags);
  return dev;
}

/****************************************************************************
 * Name: nrf54l_grtc_deinit
 *
 * Description:
 *   Stop and release the GRTC instance.
 *
 ****************************************************************************/

int nrf54l_grtc_deinit(struct nrf54l_grtc_dev_s *dev)
{
  struct nrf54l_grtc_priv_s *grtc = (struct nrf54l_grtc_priv_s *)dev;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();
  nrf54l_grtc_putreg(dev, NRF54L_GRTC_INTENCLR_OFFSET(0), 0xffffffff);
  nrf54l_grtc_setisr(dev, NULL, NULL);
  for (i = 0; i < grtc->chan; i++)
    {
      nrf54l_grtc_disablecc(dev, i);
      nrf54l_grtc_ackint(dev, i);
    }

  nrf54l_grtc_stop(dev);
  grtc->inuse = false;
  leave_critical_section(flags);
  return OK;
}
