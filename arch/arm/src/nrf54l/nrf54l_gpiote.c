/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_gpiote.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <string.h>

#include <arch/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "nrf54l_gpio.h"
#include "nrf54l_gpiote.h"

#include "hardware/nrf54l_gpiote.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOTE_PER_CHANNEL (8)
#define GPIOTE_CHANNELS    (12)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_gpiote_callback_s
{
  xcpt_t    callback;
  void     *arg;
  uint32_t  pinset;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callbacks attached to each GPIOTE channel */

static struct nrf54l_gpiote_callback_s
  g_gpiote_ch_callbacks[GPIOTE_CHANNELS];

static const uint8_t g_gpiote_pins[NRF54L_GPIO_NPORTS] =
  NRF54L_GPIO_PIN_COUNTS;

#ifdef CONFIG_NRF54L_PER_PIN_INTERRUPTS
/* Callbacks attached to each GPIO pin */

static struct nrf54l_gpiote_callback_s
    g_gpiote_pin_callbacks[NRF54L_GPIO_NPORTS][32];
#else
/* Callback for the PORT event */

static struct nrf54l_gpiote_callback_s
    g_gpiote_port_callback[NRF54L_GPIO_NPORTS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_gpiote_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset for a given GPIOTE instance
 *
 ****************************************************************************/

static inline void nrf54l_gpiote_putreg(int inst, uint32_t offset,
                                       uint32_t value)
{
  if (inst == 1)
    {
      putreg32(value, NRF54L_GPIOTE30_BASE + offset);
    }
  else
    {
      putreg32(value, NRF54L_GPIOTE20_BASE + offset);
    }
}

/****************************************************************************
 * Name: nrf54l_gpiote_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset for a given GPIOTE instance
 *
 ****************************************************************************/

static inline uint32_t nrf54l_gpiote_getreg(int inst, uint32_t offset)
{
  if (inst == 1)
    {
      return getreg32(NRF54L_GPIOTE30_BASE + offset);
    }
  else
    {
      return getreg32(NRF54L_GPIOTE20_BASE + offset);
    }
}

/****************************************************************************
 * Name: nrf54l_gpiote_pininstance
 *
 * Description:
 *   Return the GPIOTE instance connected to the pin, or a negated errno.
 *
 ****************************************************************************/

static int nrf54l_gpiote_pininstance(uint32_t pinset)
{
  unsigned int port = GPIO_PORT_DECODE(pinset);
  unsigned int pin = GPIO_PIN_DECODE(pinset);

  if (port >= NRF54L_GPIO_NPORTS || pin >= g_gpiote_pins[port] || port == 2)
    {
      return -EINVAL;
    }

  return port == 0 ? 1 : 0;
}

/****************************************************************************
 * Name: nrf54l_gpiote_isr
 *
 * Description:
 *   Common GPIOTE interrupt handler (GPIOTE20 and GPIOTE30)
 *
 ****************************************************************************/

static int nrf54l_gpiote_isr(int irq, void *context, void *arg)
{
  uint32_t regval = 0;
  int      ret    = OK;
  int      i      = 0;
  int      inst   = 0;
  int      off    = 0;
#ifdef CONFIG_NRF54L_PER_PIN_INTERRUPTS
  int      j      = 0;
#endif

  /* Get GPIOTE instance */

  inst = (irq == NRF54L_IRQ_GPIOTE20_0) ? 0 : 1;

  /* Scan all GPIOTE channels */

  for (i = 0; i < (inst == 0 ? 8 : 4); i += 1)
    {
      off = i + GPIOTE_PER_CHANNEL * inst;

      /* Only if callback is registered */

      if (g_gpiote_ch_callbacks[off].callback != NULL)
        {
          /* Get input event register */

          regval = nrf54l_gpiote_getreg(inst,
                                       NRF54L_GPIOTE_EVENTS_IN_OFFSET(i));
          if (regval == GPIOTE_EVENT_IN_EVENT)
            {
              /* Execute callback */

              xcpt_t callback = g_gpiote_ch_callbacks[off].callback;
              void *cbarg = g_gpiote_ch_callbacks[off].arg;

              /* Clear event */

              nrf54l_gpiote_putreg(inst,
                                  NRF54L_GPIOTE_EVENTS_IN_OFFSET(i), 0);
              nrf54l_gpiote_getreg(inst,
                                  NRF54L_GPIOTE_EVENTS_IN_OFFSET(i));

              ret = callback(irq, context, cbarg);
            }
        }
    }

  /* Check for PORT event */

  regval = nrf54l_gpiote_getreg(inst,
                              NRF54L_GPIOTE_EVENTS_PORT_SECURE_OFFSET);
  if (regval)
    {
      uint32_t addr = 0;

      /* Ack PORT event */

      nrf54l_gpiote_putreg(inst,
                          NRF54L_GPIOTE_EVENTS_PORT_SECURE_OFFSET, 0);
      nrf54l_gpiote_getreg(inst, NRF54L_GPIOTE_EVENTS_PORT_SECURE_OFFSET);

      /* For each GPIO port, get LATCH register */

      for (i = 0; i < NRF54L_GPIO_NPORTS; i++)
        {
          if (i == 2 || (inst == 1) != (i == 0))
            {
              continue;
            }

          switch (i)
            {
              case 0:
                addr = NRF54L_GPIO_P0_BASE + NRF54L_GPIO_LATCH_OFFSET;
                break;

              case 1:
                addr = NRF54L_GPIO_P1_BASE + NRF54L_GPIO_LATCH_OFFSET;
                break;

              case 3:
                addr = NRF54L_GPIO_P3_BASE + NRF54L_GPIO_LATCH_OFFSET;
                break;
            }

          /* Retrieve LATCH register */

          regval = getreg32(addr);

          /* Clear LATCH register (this may set PORT again) */

          putreg32(regval, addr);

#ifdef CONFIG_NRF54L_PER_PIN_INTERRUPTS
          /* Check for pins with DETECT bit high in LATCH register
           * and dispatch callback if set
           */

          for (j = 0; j < g_gpiote_pins[i] && regval; j++)
            {
              if ((regval & (1 << j)) != 0 &&
                  g_gpiote_pin_callbacks[i][j].callback)
                {
                  /* Run callback */

                  xcpt_t callback = g_gpiote_pin_callbacks[i][j].callback;
                  void *cbarg = g_gpiote_pin_callbacks[i][j].arg;

                  ret = callback(irq, context, cbarg);

                  /* Mark bit is as "visited", we can stop looping sooner
                   * this way
                   */

                  regval &= ~(1 << j);
                }
            }
#else
          if (regval && g_gpiote_port_callback[i].callback)
            {
              xcpt_t callback = g_gpiote_port_callback[i].callback;
              void *cbarg = g_gpiote_port_callback[i].arg;

              ret = callback(irq, context, cbarg);
            }
#endif
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_NRF54L_PER_PIN_INTERRUPTS
/****************************************************************************
 * Name: nrf54l_gpiote_set_pin_event
 *
 * Description:
 *   Sets/clears a handler for a given pin for the GPIO PORT event. This
 *   will mean edge-sensitive or level-sensitive according to GPIO detect
 *   mode configuration for the port (see nrf54l_gpio_detectmode()). Pin
 *   will be sensitive to high/low according to GPIO_SENSE_LOW/HIGH
 *   (set via nrf54l_gpio_config()).
 *
 *   The passed handler will be invoked from the main ISR for the PORT
 *   event and will take care of clearing the LATCH register.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf54l_gpiote_set_pin_event(uint32_t pinset, xcpt_t func, void *arg)
{
  int        pin    = 0;
  int        port   = 0;
  irqstate_t flags;

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  if (nrf54l_gpiote_pininstance(pinset) < 0)
    {
      return;
    }

  flags = enter_critical_section();

  g_gpiote_pin_callbacks[port][pin].callback = func;
  g_gpiote_pin_callbacks[port][pin].arg = arg;

  leave_critical_section(flags);
}
#else
/****************************************************************************
 * Name: nrf54l_gpiote_set_port_event
 *
 * Description:
 *   Sets/clears the handler for the GPIO PORT event.
 *
 *   The passed handler will be invoked from the main ISR for the PORT
 *   event and will take care of clearing the LATCH register.
 *
 * Input Parameters:
 *  - pinset:      GPIO port will be extracted from this parameter
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf54l_gpiote_set_port_event(uint32_t pinset, xcpt_t func, void *arg)
{
  int        port   = 0;
  int        inst   = 0;
  irqstate_t flags;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  inst = nrf54l_gpiote_pininstance(pinset);
  if (inst < 0)
    {
      return;
    }

  flags = enter_critical_section();

  g_gpiote_port_callback[port].callback = func;
  g_gpiote_port_callback[port].arg      = arg;

  if (func)
    {
      /* Enable the ISR */

      nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENSET0_OFFSET,
                          GPIOTE_INT_PORT_SECURE);
    }
  else
    {
      /* Check if we can disable the ISR */

      int i;

      for (i = 0; i < NRF54L_GPIO_NPORTS; i++)
        {
          if ((inst == 1) == (i == 0) &&
              g_gpiote_port_callback[i].callback)
            {
              break;
            }
        }

      if (i == NRF54L_GPIO_NPORTS)
        {
          nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENCLR0_OFFSET,
                              GPIOTE_INT_PORT_SECURE);
        }
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: nrf54l_gpiote_set_ch_event
 *
 * Description:
 *   Configures a GPIOTE channel in EVENT mode, assigns it to a given pin
 *   and sets a handler for the corresponding channel events.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - channel:     GPIOTE channel used to capture events
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf54l_gpiote_set_ch_event(uint32_t pinset, int channel,
                               bool risingedge, bool fallingedge,
                               xcpt_t func, void *arg)
{
  int        pin    = 0;
  int        port   = 0;
  int        inst   = 0;
  uint32_t   regval = 0;
  uint32_t   rchan  = 0;
  irqstate_t flags;

  if (channel < 0 || channel >= GPIOTE_CHANNELS)
    {
      return;
    }

  /* Get GPIOTE instance */

  inst = (channel < GPIOTE_PER_CHANNEL) ? 0 : 1;
  if (nrf54l_gpiote_pininstance(pinset) != inst)
    {
      return;
    }

  rchan = (inst == 1) ? (channel - GPIOTE_PER_CHANNEL) : channel;

  /* NOTE: GPIOTE module has priority over GPIO module
   *       so GPIO configuration will be ignored
   */

  flags = enter_critical_section();

  nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENCLR0_OFFSET,
                      GPIOTE_INT_IN(rchan));
  nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_CONFIG_OFFSET(rchan), 0);
  nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_EVENTS_IN_OFFSET(rchan), 0);
  nrf54l_gpiote_getreg(inst, NRF54L_GPIOTE_EVENTS_IN_OFFSET(rchan));

  if (func)
    {
      /* Select EVENT mode */

      regval |= GPIOTE_CONFIG_MODE_EV;

      /* Select GPIOTE pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      regval |= (pin << GPIOTE_CONFIG_PSEL_SHIFT);

      port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
      regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);

      /* Select polarity */

      if (risingedge == true && fallingedge == true)
        {
          regval |= GPIOTE_CONFIG_POL_TG;
        }
      else if (risingedge == true)
        {
          regval |= GPIOTE_CONFIG_POL_LTH;
        }
      else if (fallingedge == true)
        {
          regval |= GPIOTE_CONFIG_POL_HTL;
        }

      /* Enable callback for channel */

      g_gpiote_ch_callbacks[channel].callback = func;
      g_gpiote_ch_callbacks[channel].arg      = arg;
      g_gpiote_ch_callbacks[channel].pinset   = pinset;
    }
  else
    {
      /* Leave register as zero (disabled mode) */

      /* Disable interrupt for given event */

      nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENCLR0_OFFSET,
                          GPIOTE_INT_IN(rchan));

      /* Remove callback configuration */

      g_gpiote_ch_callbacks[channel].callback = NULL;
      g_gpiote_ch_callbacks[channel].arg      = NULL;
    }

  /* Write CONFIG register */

  nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_CONFIG_OFFSET(rchan), regval);

  if (func)
    {
      nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENSET0_OFFSET,
                          GPIOTE_INT_IN(rchan));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_gpiote_set_event
 *
 * Description:
 *   Configures a GPIOTE channel in EVENT mode, assigns it to a given pin
 *   and sets a handler for the first available GPIOTE channel.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Channel index on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf54l_gpiote_set_event(uint32_t pinset,
                           bool risingedge, bool fallingedge,
                           xcpt_t func, void *arg)
{
  irqstate_t flags;
  int ret = -ENOMEM;
  int i = 0;
  int first;
  int end;
  int freechan = -1;

  ret = nrf54l_gpiote_pininstance(pinset);
  if (ret < 0)
    {
      return ret;
    }

  first = ret == 0 ? 0 : GPIOTE_PER_CHANNEL;
  end = ret == 0 ? GPIOTE_PER_CHANNEL : GPIOTE_CHANNELS;
  ret = -ENOMEM;

  flags = enter_critical_section();

  /* Get free channel or channel already used by pinset */

  for (i = first; i < end; i++)
    {
      if (g_gpiote_ch_callbacks[i].callback != NULL &&
          GPIO_PIN_DECODE(g_gpiote_ch_callbacks[i].pinset) ==
          GPIO_PIN_DECODE(pinset) &&
          GPIO_PORT_DECODE(g_gpiote_ch_callbacks[i].pinset) ==
          GPIO_PORT_DECODE(pinset))
        {
          freechan = i;
          break;
        }

      if (freechan < 0 &&
          nrf54l_gpiote_getreg(i < GPIOTE_PER_CHANNEL ? 0 : 1,
            NRF54L_GPIOTE_CONFIG_OFFSET(i % GPIOTE_PER_CHANNEL)) == 0)
        {
          freechan = i;
        }
    }

  if (func == NULL && i == end)
    {
      ret = -ENOENT;
    }
  else if (freechan >= 0)
    {
      nrf54l_gpiote_set_ch_event(pinset, freechan,
                                risingedge, fallingedge, func, arg);
      ret = freechan;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: nrf54l_gpiote_set_task
 *
 * Description:
 *   Configure GPIO in TASK mode (to be controlled via tasks).
 *   Note that a pin can only be either in TASK or EVENT mode (set by
 *   nrf54l_gpiote_set_event). Also, once set to TASK mode,
 *   pin control is only possible via tasks. TASK mode automatically
 *   selects the output direction.
 *   Finally, a given pin should only be assigned to a given channel.
 *
 * Input Parameters:
 *  - pinset:      gpio pin configuration (only port + pin is important here)
 *  - channel:     the GPIOTE channel used to control the given pin
 *  - output_high: set pin initially to output HIGH or LOW.
 *  - outcfg:      configure pin behavior one OUT task is triggered
 *
 ****************************************************************************/

void nrf54l_gpiote_set_task(uint32_t pinset, int channel,
                           bool output_high,
                           enum nrf54l_gpiote_outcfg_e outcfg)
{
  uint32_t regval;
  uint32_t rchan;
  int pin;
  int port;
  int inst;
  irqstate_t flags;

  /* Get GPIOTE instance */

  if (channel < 0 || channel >= GPIOTE_CHANNELS)
    {
      return;
    }

  inst = (channel < GPIOTE_PER_CHANNEL) ? 0 : 1;
  if (nrf54l_gpiote_pininstance(pinset) != inst)
    {
      return;
    }

  rchan = (inst == 1) ? (channel - GPIOTE_PER_CHANNEL) : channel;

  /* Select GPIOTE pin */

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  regval = (pin << GPIOTE_CONFIG_PSEL_SHIFT);

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);

  /* Select TASK mode */

  regval |= GPIOTE_CONFIG_MODE_TS;

  /* Select pin number */

  regval |= (pin << GPIOTE_CONFIG_PSEL_SHIFT);

  /* Select initial output */

  if (output_high)
    {
      regval |= (1 << GPIOTE_CONFIG_OUTINIT_SHIFT);
    }

  /* Set polarity mode */

  switch (outcfg)
    {
      case NRF54L_GPIOTE_SET:
        regval |= GPIOTE_CONFIG_POL_LTH;
        break;
      case NRF54L_GPIOTE_CLEAR:
        regval |= GPIOTE_CONFIG_POL_HTL;
        break;
      case NRF54L_GPIOTE_TOGGLE:
        regval |= GPIOTE_CONFIG_POL_TG;
        break;
    }

  /* Write register */

  flags = enter_critical_section();
  nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENCLR0_OFFSET,
                      GPIOTE_INT_IN(rchan));
  g_gpiote_ch_callbacks[channel].callback = NULL;
  g_gpiote_ch_callbacks[channel].arg = NULL;
  nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_CONFIG_OFFSET(rchan), regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_gpiote_init
 *
 * Description:
 *   Initialize GPIOTE
 *
 ****************************************************************************/

int nrf54l_gpiote_init(void)
{
  int inst;
  int i;
  int ret;

  /* Clear LATCH register(s) */

  putreg32(0xffffffff, NRF54L_GPIO_P0_BASE + NRF54L_GPIO_LATCH_OFFSET);
  putreg32(0xffffffff, NRF54L_GPIO_P1_BASE + NRF54L_GPIO_LATCH_OFFSET);
  if (NRF54L_GPIO_NPORTS > 3)
    {
      putreg32(0xffffffff, NRF54L_GPIO_P3_BASE + NRF54L_GPIO_LATCH_OFFSET);
    }

  for (inst = 0; inst < 2; inst++)
    {
      nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENCLR0_OFFSET, 0xffffffff);
      nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_INTENCLR1_OFFSET, 0xffffffff);
      nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_EVENTS_PORT_SECURE_OFFSET, 0);
      nrf54l_gpiote_getreg(inst, NRF54L_GPIOTE_EVENTS_PORT_SECURE_OFFSET);
      for (i = 0; i < (inst == 0 ? 8 : 4); i++)
        {
          nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_CONFIG_OFFSET(i), 0);
          nrf54l_gpiote_putreg(inst, NRF54L_GPIOTE_EVENTS_IN_OFFSET(i), 0);
          nrf54l_gpiote_getreg(inst, NRF54L_GPIOTE_EVENTS_IN_OFFSET(i));
        }
    }

  /* Reset GPIOTE data */

  memset(&g_gpiote_ch_callbacks, 0, sizeof(g_gpiote_ch_callbacks));

#ifdef CONFIG_NRF54L_PER_PIN_INTERRUPTS
  memset(&g_gpiote_pin_callbacks, 0, sizeof(g_gpiote_pin_callbacks));

  /* Enable PORT event interrupt */

  nrf54l_gpiote_putreg(0, NRF54L_GPIOTE_INTENSET0_OFFSET,
                      GPIOTE_INT_PORT_SECURE);
  nrf54l_gpiote_putreg(1, NRF54L_GPIOTE_INTENSET0_OFFSET,
                      GPIOTE_INT_PORT_SECURE);
#else
  memset(&g_gpiote_port_callback, 0, sizeof(g_gpiote_port_callback));
#endif

  /* Attach GPIOTE interrupt handler */

  ret = irq_attach(NRF54L_IRQ_GPIOTE20_0, nrf54l_gpiote_isr, NULL);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(NRF54L_IRQ_GPIOTE30_0, nrf54l_gpiote_isr, NULL);
  if (ret < 0)
    {
      irq_detach(NRF54L_IRQ_GPIOTE20_0);
      return ret;
    }

  up_enable_irq(NRF54L_IRQ_GPIOTE20_0);
  up_enable_irq(NRF54L_IRQ_GPIOTE30_0);

  return OK;
}
