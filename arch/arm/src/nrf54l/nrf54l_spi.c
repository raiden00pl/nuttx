/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_spi.c
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
#include <inttypes.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/barriers.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "nrf54l_gpio.h"
#include "nrf54l_spi.h"

#include "hardware/nrf54l_spi.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         base;       /* Base address of SPI register */
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  uint32_t         irq;        /* SPI IRQ number */
#endif
  nrf54l_pinset_t sck_pin;     /* SCK pin configuration */
  uint32_t         frequency;  /* Requested clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */
  int              status;     /* DMA transfer status */

  mutex_t          lock;       /* Held while chip is selected for mutual
                                * exclusion
                                */
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  sem_t            sem_isr;    /* Interrupt wait semaphore */
#endif
  bool             initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf54l_spi_putreg(struct nrf54l_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf54l_spi_getreg(struct nrf54l_spidev_s *priv,
                                        uint32_t offset);

/* SPI methods */

static int nrf54l_spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t nrf54l_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void nrf54l_spi_setmode(struct spi_dev_s *priv,
                              enum spi_mode_e mode);
static void nrf54l_spi_setbits(struct spi_dev_s *priv, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int nrf54l_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t nrf54l_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void nrf54l_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void nrf54l_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords);
static void nrf54l_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords);
#endif

#ifdef CONFIG_SPI_TRIGGER
static int nrf54l_spi_trigger(struct spi_dev_s *dev);
#endif

#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
static int nrf54l_spi_isr(int irq, void *context, void *arg);
#endif

/* Initialization */

static int nrf54l_spi_init(struct nrf54l_spidev_s *priv);
static int nrf54l_spi_pselinit(struct nrf54l_spidev_s *priv,
                               uint32_t offset, nrf54l_pinset_t pinset);
static int nrf54l_spi_gpioinit(struct nrf54l_spidev_s *priv);
static void nrf54l_spi_gpiodeinit(struct nrf54l_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI0 */

#ifdef CONFIG_NRF54L_SPI0_MASTER
static const struct spi_ops_s g_spi0ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi0select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi0status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi0cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi0register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf54l_spidev_s g_spi0dev =
{
  .spidev    =
  {
    .ops     = &g_spi0ops,
  },

  .base      = NRF54L_SPIM20_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL20,
#endif
  .sck_pin   = BOARD_SPI0_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI1 */

#ifdef CONFIG_NRF54L_SPI1_MASTER
static const struct spi_ops_s g_spi1ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi1select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi1status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi1cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi1register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf54l_spidev_s g_spi1dev =
{
  .spidev    =
  {
    .ops     = &g_spi1ops,
  },

  .base      = NRF54L_SPIM21_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL21,
#endif
  .sck_pin   = BOARD_SPI1_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI2 */

#ifdef CONFIG_NRF54L_SPI2_MASTER
static const struct spi_ops_s g_spi2ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi2select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi2status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi2cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi2register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf54l_spidev_s g_spi2dev =
{
  .spidev    =
  {
    .ops     = &g_spi2ops,
  },

  .base      = NRF54L_SPIM22_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL22,
#endif
  .sck_pin   = BOARD_SPI2_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI3 */

#ifdef CONFIG_NRF54L_SPI3_MASTER
static const struct spi_ops_s g_spi3ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi3select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi3status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi3cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi3register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf54l_spidev_s g_spi3dev =
{
  .spidev    =
  {
    .ops     = &g_spi3ops,
  },

  .base      = NRF54L_SPIM30_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL30,
#endif
  .sck_pin   = BOARD_SPI3_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI4 */

#ifdef CONFIG_NRF54L_SPI4_MASTER
static const struct spi_ops_s g_spi4ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi4select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi4status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi4cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi4register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf54l_spidev_s g_spi4dev =
{
  .spidev    =
  {
    .ops     = &g_spi4ops,
  },

  .base      = NRF54L_SPIM00_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL00,
#endif
  .sck_pin   = BOARD_SPI4_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI5 */

#ifdef CONFIG_NRF54L_SPI5_MASTER
static const struct spi_ops_s g_spi5ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi5select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi5status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi5cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#  ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#  endif
#  ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi5register,
#  else
  .registercallback  = NULL,
#  endif
};

static struct nrf54l_spidev_s g_spi5dev =
{
  .spidev    =
  {
    .ops     = &g_spi5ops,
  },

  .base      = NRF54L_SPIM23_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#  ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL23,
#  endif
  .sck_pin   = BOARD_SPI5_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI6 */

#ifdef CONFIG_NRF54L_SPI6_MASTER
static const struct spi_ops_s g_spi6ops =
{
  .lock              = nrf54l_spi_lock,
  .select            = nrf54l_spi6select,
  .setfrequency      = nrf54l_spi_setfrequency,
  .setmode           = nrf54l_spi_setmode,
  .setbits           = nrf54l_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf54l_spi_hwfeatures,
#  endif
  .status            = nrf54l_spi6status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf54l_spi6cmddata,
#  endif
  .send              = nrf54l_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf54l_spi_exchange,
#  else
  .sndblock          = nrf54l_spi_sndblock,
  .recvblock         = nrf54l_spi_recvblock,
#  endif
#  ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf54l_spi_trigger,
#  endif
#  ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf54l_spi6register,
#  else
  .registercallback  = NULL,
#  endif
};

static struct nrf54l_spidev_s g_spi6dev =
{
  .spidev    =
  {
    .ops     = &g_spi6ops,
  },

  .base      = NRF54L_SPIM24_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#  ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF54L_IRQ_SERIAL24,
#  endif
  .sck_pin   = BOARD_SPI6_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_spi_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf54l_spi_putreg(struct nrf54l_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf54l_spi_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf54l_spi_getreg(struct nrf54l_spidev_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf54l_spi_isr
 *
 * Description:
 *   Common SPI interrupt service routine
 *
 ****************************************************************************/

#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
static int nrf54l_spi_isr(int irq, void *context, void *arg)
{
  struct nrf54l_spidev_s *priv = (struct nrf54l_spidev_s *)arg;
  uint32_t enabled;

  /* Ignore a pending interrupt after the completion source was disabled. */

  enabled = nrf54l_spi_getreg(priv, NRF54L_SPIM_INTENSET_OFFSET);
  if (((enabled & SPIM_INT_END) &&
       nrf54l_spi_getreg(priv, NRF54L_SPIM_EVENTS_END_OFFSET)) ||
      ((enabled & SPIM_INT_DMA_RX_BUSERROR) &&
       nrf54l_spi_getreg(priv, NRF54L_SPIM_EVENTS_DMA_RX_BUSERROR_OFFSET)) ||
      ((enabled & SPIM_INT_DMA_TX_BUSERROR) &&
       nrf54l_spi_getreg(priv, NRF54L_SPIM_EVENTS_DMA_TX_BUSERROR_OFFSET)))
    {
      /* Transfer is complete */

      nrf54l_spi_putreg(priv, NRF54L_SPIM_INTENCLR_OFFSET,
                       SPIM_INT_END | SPIM_INT_DMA_RX_BUSERROR |
                       SPIM_INT_DMA_TX_BUSERROR);
      nrf54l_spi_getreg(priv, NRF54L_SPIM_INTENCLR_OFFSET);
      nxsem_post(&priv->sem_isr);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nrf54l_spi_init
 *
 * Description:
 *   Configure SPI
 *
 ****************************************************************************/

static int nrf54l_spi_init(struct nrf54l_spidev_s *priv)
{
  int ret;

  /* Disable SPI */

  nrf54l_spi_putreg(priv, NRF54L_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);

  /* Configure SPI pins */

  ret = nrf54l_spi_gpioinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* NOTE: Chip select pin must be configured by board-specific logic */

  nrf54l_spi_putreg(priv, NRF54L_SPIM_INTENCLR_OFFSET, 0xffffffff);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_END_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_STOPPED_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_DMA_RX_BUSERROR_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_DMA_TX_BUSERROR_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_SHORTS_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_CONFIG_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_ORC_OFFSET, 0xff);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_LIST_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_TX_LIST_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_MATCH_CONFIG_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_TERMINATEONBUSERROR_OFFSET,
                   SPIM_DMA_TERMINATEONBUSERROR);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_TX_TERMINATEONBUSERROR_OFFSET,
                   SPIM_DMA_TERMINATEONBUSERROR);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_RXDELAY_OFFSET,
                   priv->base == NRF54L_SPIM00_BASE ? 2 : 1);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELCSN_OFFSET, SPIM_PSEL_RESET);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELDCX_OFFSET, SPIM_PSEL_RESET);
  priv->frequency = 0;
  priv->mode = SPIDEV_MODE0;
  nrf54l_gpio_write(priv->sck_pin, false);
  nrf54l_spi_setfrequency(&priv->spidev, 2000000);

#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  /* Enable interrupts for RX and TX done */

  nrf54l_spi_putreg(priv, NRF54L_SPIM_INTENSET_OFFSET, SPIM_INT_END);
#endif

  /* Enable SPI */

  nrf54l_spi_putreg(priv, NRF54L_SPIM_ENABLE_OFFSET, SPIM_ENABLE_EN);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_spi_pselinit
 *
 * Description:
 *   Configure PSEL for SPI devices
 *
 ****************************************************************************/

static int nrf54l_spi_pselinit(struct nrf54l_spidev_s *priv,
                               uint32_t offset, nrf54l_pinset_t pinset)
{
  uint32_t regval;
  int pin  = GPIO_PIN_DECODE(pinset);
  int port = GPIO_PORT_DECODE(pinset);
  int ret;

  ret = nrf54l_gpio_config(pinset);
  if (ret < 0)
    {
      return ret;
    }

  regval = (pin << SPIM_PSEL_PIN_SHIFT);
  regval |= (port << SPIM_PSEL_PORT_SHIFT);
  nrf54l_spi_putreg(priv, offset, regval);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_spi_gpioinit
 *
 * Description:
 *   Configure GPIO for SPI pins
 *
 ****************************************************************************/

static int nrf54l_spi_gpioinit(struct nrf54l_spidev_s *priv)
{
  int ret;

  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELSCK_OFFSET, SPIM_PSEL_RESET);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELMISO_OFFSET, SPIM_PSEL_RESET);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELMOSI_OFFSET, SPIM_PSEL_RESET);
  ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELSCK_OFFSET, priv->sck_pin);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_NRF54L_SPI0_MASTER
  if (priv == &g_spi0dev)
    {
#ifdef BOARD_SPI0_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI0_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI0_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI0_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI0_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI1_MASTER
  if (priv == &g_spi1dev)
    {
#ifdef BOARD_SPI1_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI1_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI1_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI1_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI1_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI2_MASTER
  if (priv == &g_spi2dev)
    {
#ifdef BOARD_SPI2_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI2_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI2_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI2_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI2_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI3_MASTER
  if (priv == &g_spi3dev)
    {
#ifdef BOARD_SPI3_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI3_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI3_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI3_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI3_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI4_MASTER
  if (priv == &g_spi4dev)
    {
#ifdef BOARD_SPI4_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI4_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI4_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI4_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI4_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI5_MASTER
  if (priv == &g_spi5dev)
    {
#ifdef BOARD_SPI5_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI5_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI5_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI5_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI5_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI6_MASTER
  if (priv == &g_spi6dev)
    {
#ifdef BOARD_SPI6_MISO_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMISO_OFFSET,
                               BOARD_SPI6_MISO_PIN);
      if (ret < 0)
        {
          goto errout;
        }

#endif
#ifdef BOARD_SPI6_MOSI_PIN
      ret = nrf54l_spi_pselinit(priv, NRF54L_SPIM_PSELMOSI_OFFSET,
                               BOARD_SPI6_MOSI_PIN);
      if (ret < 0)
        {
          goto errout;
        }

      nrf54l_gpio_write(BOARD_SPI6_MOSI_PIN, false);
#endif
    }
#endif

  return OK;

errout:
  nrf54l_spi_gpiodeinit(priv);
  return ret;
}

/****************************************************************************
 * Name: nrf54l_spi_gpiodeinit
 *
 * Description:
 *   Release GPIO and disconnect SPI pins
 *
 ****************************************************************************/

static void nrf54l_spi_gpiodeinit(struct nrf54l_spidev_s *priv)
{
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELSCK_OFFSET, SPIM_PSEL_RESET);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELMISO_OFFSET, SPIM_PSEL_RESET);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_PSELMOSI_OFFSET, SPIM_PSEL_RESET);
  nrf54l_gpio_unconfig(priv->sck_pin);

#ifdef CONFIG_NRF54L_SPI0_MASTER
  if (priv == &g_spi0dev)
    {
#ifdef BOARD_SPI0_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI0_MISO_PIN);
#endif
#ifdef BOARD_SPI0_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI0_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI1_MASTER
  if (priv == &g_spi1dev)
    {
#ifdef BOARD_SPI1_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI1_MISO_PIN);
#endif
#ifdef BOARD_SPI1_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI1_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI2_MASTER
  if (priv == &g_spi2dev)
    {
#ifdef BOARD_SPI2_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI2_MISO_PIN);
#endif
#ifdef BOARD_SPI2_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI2_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI3_MASTER
  if (priv == &g_spi3dev)
    {
#ifdef BOARD_SPI3_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI3_MISO_PIN);
#endif
#ifdef BOARD_SPI3_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI3_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI4_MASTER
  if (priv == &g_spi4dev)
    {
#ifdef BOARD_SPI4_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI4_MISO_PIN);
#endif
#ifdef BOARD_SPI4_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI4_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI5_MASTER
  if (priv == &g_spi5dev)
    {
#ifdef BOARD_SPI5_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI5_MISO_PIN);
#endif
#ifdef BOARD_SPI5_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI5_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF54L_SPI6_MASTER
  if (priv == &g_spi6dev)
    {
#ifdef BOARD_SPI6_MISO_PIN
      nrf54l_gpio_unconfig(BOARD_SPI6_MISO_PIN);
#endif
#ifdef BOARD_SPI6_MOSI_PIN
      nrf54l_gpio_unconfig(BOARD_SPI6_MOSI_PIN);
#endif
    }
#endif
}

/****************************************************************************
 * Name: nrf54l_spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf54l_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct nrf54l_spidev_s *priv = (struct nrf54l_spidev_s *)dev;
  int ret = OK;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t nrf54l_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct nrf54l_spidev_s *priv = (struct nrf54l_spidev_s *)dev;
  uint32_t regval;
  uint32_t clock = priv->base == NRF54L_SPIM00_BASE ? 128000000 : 16000000;
  uint32_t min = priv->base == NRF54L_SPIM00_BASE ? 4 : 2;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency */

      return priv->frequency;
    }

  if (frequency == 0)
    {
      return priv->frequency;
    }

  /* Round up to the next even divisor, so SCK does not exceed the request.
   * Requests below the minimum attainable frequency use the slowest clock.
   */

  regval = clock / frequency + (clock % frequency != 0);
  regval = (regval + 1) & ~1;
  if (regval < min)
    {
      regval = min;
    }
  else if (regval > SPIM_PRESCALER_DIVISOR_MAX)
    {
      regval = SPIM_PRESCALER_DIVISOR_MAX;
    }

  /* Write register */

  nrf54l_spi_putreg(priv, NRF54L_SPIM_PRESCALER_OFFSET, regval);

  /* Save the frequency setting */

  priv->frequency = clock / regval;

  spiinfo("Frequency %" PRId32 "\n", frequency);

  return priv->frequency;
}

/****************************************************************************
 * Name: nrf54l_spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf54l_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  struct nrf54l_spidev_s *priv = (struct nrf54l_spidev_s *)dev;
  uint32_t regval = 0;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      regval = nrf54l_spi_getreg(priv, NRF54L_SPIM_CONFIG_OFFSET);
      regval &= ~(SPIM_CONFIG_CPHA | SPIM_CONFIG_CPOL);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            {
              break;
            }

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            {
              regval |= SPIM_CONFIG_CPHA;
              break;
            }

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            {
              regval |= SPIM_CONFIG_CPOL;
              break;
            }

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            {
              regval |= SPIM_CONFIG_CPHA;
              regval |= SPIM_CONFIG_CPOL;
              break;
            }

          default:
            {
              DEBUGPANIC();
              return;
            }
        }

      nrf54l_spi_putreg(priv, NRF54L_SPIM_CONFIG_OFFSET, regval);

      /* According to manual we have to set SCK pin output
       * value the same as CPOL value
       */

      if (mode == SPIDEV_MODE2 || mode == SPIDEV_MODE3)
        {
          nrf54l_gpio_write(priv->sck_pin, true);
        }
      else
        {
          nrf54l_gpio_write(priv->sck_pin, false);
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: nrf54l_spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf54l_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spierr("ERROR: nbits not supported: %d\n", nbits);
    }
}

/****************************************************************************
 * Name: nrf54l_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int nrf54l_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct nrf54l_spidev_s *priv = (struct nrf54l_spidev_s *)dev;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;
  uint32_t regval;

  spiinfo("features=%08x\n", features);

  if ((features & ~HWFEAT_LSBFIRST) != 0)
    {
      return -ENOSYS;
    }

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = SPIM_CONFIG_ORDER;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = SPIM_CONFIG_ORDER;
    }

  regval = nrf54l_spi_getreg(priv, NRF54L_SPIM_CONFIG_OFFSET);
  regval &= ~clrbits;
  regval |= setbits;
  nrf54l_spi_putreg(priv, NRF54L_SPIM_CONFIG_OFFSET, regval);

  return OK;
#else
  return features == 0 ? OK : -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: nrf54l_spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t nrf54l_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t ret = 0;

  /* Exchange one word on SPI */

  nrf54l_spi_exchange(dev, &wd, &ret, 1);

  return ret;
}

/****************************************************************************
 * Name: nrf54l_spi_exchange
 *
 * Description:
 *   Exchange a block of data on SPI using EasyDMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf54l_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords)
{
  struct nrf54l_spidev_s *priv = (struct nrf54l_spidev_s *)dev;
  uint32_t regval = 0;
  size_t nwords_left = nwords;
  const uint8_t *txptr = txbuffer;
  uint8_t *rxptr = rxbuffer;
  uint8_t dummy = 0xff;

  priv->status = OK;

  while (nwords_left > 0)
    {
      size_t transfer_size = nwords_left > SPIM_DMA_MAXCNT_MASK ?
                             SPIM_DMA_MAXCNT_MASK : nwords_left;

      if (txbuffer == NULL && rxbuffer == NULL)
        {
          txptr = &dummy;
          transfer_size = 1;
        }

      nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_END_OFFSET, 0);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_STOPPED_OFFSET, 0);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_DMA_RX_BUSERROR_OFFSET, 0);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_DMA_TX_BUSERROR_OFFSET, 0);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_PTR_OFFSET,
                       (uintptr_t)rxptr);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_TX_PTR_OFFSET,
                       (uintptr_t)txptr);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_MAXCNT_OFFSET,
                       rxptr != NULL ? transfer_size : 0);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_TX_MAXCNT_OFFSET,
                       txptr != NULL ? transfer_size : 0);

#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
      nrf54l_spi_putreg(priv, NRF54L_SPIM_INTENSET_OFFSET,
                       SPIM_INT_END | SPIM_INT_DMA_RX_BUSERROR |
                       SPIM_INT_DMA_TX_BUSERROR);
#endif

      /* SPI start */

      UP_DMB();
      nrf54l_spi_putreg(priv, NRF54L_SPIM_TASK_START_OFFSET,
                       SPIM_TASKS_START);

#ifndef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
      /* Wait for RX done and TX done */

      while (!nrf54l_spi_getreg(priv, NRF54L_SPIM_EVENTS_END_OFFSET) &&
             !nrf54l_spi_getreg(priv,
                                NRF54L_SPIM_EVENTS_DMA_RX_BUSERROR_OFFSET) &&
             !nrf54l_spi_getreg(priv,
                                NRF54L_SPIM_EVENTS_DMA_TX_BUSERROR_OFFSET));
#else
      /* Wait for transfer complete */

      nxsem_wait_uninterruptible(&priv->sem_isr);
#endif

      if (nrf54l_spi_getreg(priv,
                            NRF54L_SPIM_EVENTS_DMA_RX_BUSERROR_OFFSET) ||
          nrf54l_spi_getreg(priv,
                            NRF54L_SPIM_EVENTS_DMA_TX_BUSERROR_OFFSET))
        {
          priv->status = -EIO;
          spierr("SPI DMA bus error\n");
        }

      /* SPI stop */

      nrf54l_spi_putreg(priv, NRF54L_SPIM_TASK_STOP_OFFSET, SPIM_TASKS_STOP);

      /* Wait for STOP event */

      while (!nrf54l_spi_getreg(priv, NRF54L_SPIM_EVENTS_STOPPED_OFFSET));

      /* Clear event */

      nrf54l_spi_putreg(priv, NRF54L_SPIM_EVENTS_STOPPED_OFFSET, 0);
      nrf54l_spi_getreg(priv, NRF54L_SPIM_EVENTS_STOPPED_OFFSET);
      UP_DMB();

      if (txptr != NULL)
        {
          regval = nrf54l_spi_getreg(priv, NRF54L_SPIM_DMA_TX_AMOUNT_OFFSET);
          if (regval != transfer_size)
            {
              priv->status = -EIO;
              spierr("Incomplete TX: %" PRIu32 " expected %zu\n",
                     regval, transfer_size);
            }
        }

      if (rxptr != NULL)
        {
          regval = nrf54l_spi_getreg(priv, NRF54L_SPIM_DMA_RX_AMOUNT_OFFSET);
          if (regval != transfer_size)
            {
              priv->status = -EIO;
              spierr("Incomplete RX: %" PRIu32 " expected %zu\n",
                     regval, transfer_size);
            }
        }

      if (priv->status < 0)
        {
          break;
        }

      if (txbuffer != NULL)
        {
          txptr += transfer_size;
        }

      if (rxbuffer != NULL)
        {
          rxptr += transfer_size;
        }

      nwords_left -= transfer_size;
    }

  /* Clear RX/TX DMA after transfer */

  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_PTR_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_RX_MAXCNT_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_TX_PTR_OFFSET, 0);
  nrf54l_spi_putreg(priv, NRF54L_SPIM_DMA_TX_MAXCNT_OFFSET, 0);
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: nrf54l_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf54l_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", txbuffer, nwords);
  return nrf54l_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: nrf54l_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf54l_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", rxbuffer, nwords);
  return nrf54l_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: nrf54l_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int nrf54l_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *nrf54l_spibus_initialize(int port)
{
  struct nrf54l_spidev_s *priv = NULL;
  int ret;

  /* Get SPI driver data */

  switch (port)
    {
#ifdef CONFIG_NRF54L_SPI0_MASTER
      case 0:
        {
          priv = &g_spi0dev;
          break;
        }
#endif

#ifdef CONFIG_NRF54L_SPI1_MASTER
      case 1:
        {
          priv = &g_spi1dev;
          break;
        }
#endif

#ifdef CONFIG_NRF54L_SPI2_MASTER
      case 2:
        {
          priv = &g_spi2dev;
          break;
        }
#endif

#ifdef CONFIG_NRF54L_SPI3_MASTER
      case 3:
        {
          priv = &g_spi3dev;
          break;
        }
#endif

#ifdef CONFIG_NRF54L_SPI4_MASTER
      case 4:
        {
          priv = &g_spi4dev;
          break;
        }
#endif

#ifdef CONFIG_NRF54L_SPI5_MASTER
      case 5:
        {
          priv = &g_spi5dev;
          break;
        }
#endif

#ifdef CONFIG_NRF54L_SPI6_MASTER
      case 6:
        {
          priv = &g_spi6dev;
          break;
        }
#endif

      default:
        {
          goto errout;
        }
    }

  /* Initialize the SPI */

  if (nxmutex_lock(&priv->lock) < 0)
    {
      return NULL;
    }

  if (priv->initialized)
    {
      nxmutex_unlock(&priv->lock);
      return &priv->spidev;
    }

  ret = nrf54l_spi_init(priv);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

#ifdef CONFIG_NRF54L_SPI_MASTER_INTERRUPTS
  /* Attach SPI interrupt */

  ret = irq_attach(priv->irq, nrf54l_spi_isr, priv);
  if (ret < 0)
    {
      nrf54l_spi_putreg(priv, NRF54L_SPIM_INTENCLR_OFFSET, 0xffffffff);
      nrf54l_spi_putreg(priv, NRF54L_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);
      nrf54l_spi_gpiodeinit(priv);
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  nxsem_reset(&priv->sem_isr, 0);
  up_enable_irq(priv->irq);
#endif

  priv->initialized = true;
  nxmutex_unlock(&priv->lock);

errout:
  return (struct spi_dev_s *)priv;
}
