/*****************************************************************************
 * drivers/serial/uart_pci_16550.c
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
 *****************************************************************************/

/* Serial driver for 16550 UART PCI */

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/dma/dma.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/uart_16550.h>

#include <nuttx/serial/uart_pci_16550.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define UART0_16550_PCI_DEVPATH "/dev/ttyS0"
#define UART1_16550_PCI_DEVPATH "/dev/ttyS1"
#define UART2_16550_PCI_DEVPATH "/dev/ttyS2"
#define UART3_16550_PCI_DEVPATH "/dev/ttyS3"

/* UART PCI console support */

#if defined(CONFIG_16550_PCI_UART0_SERIAL_CONSOLE) ||   \
    defined(CONFIG_16550_PCI_UART1_SERIAL_CONSOLE) ||   \
    defined(CONFIG_16550_PCI_UART2_SERIAL_CONSOLE) ||   \
    defined(CONFIG_16550_PCI_UART3_SERIAL_CONSOLE)
#  define HAVE_16550_PCI_CONSOLE
#endif

#if defined(CONFIG_16550_PCI_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0_pci_port
#elif defined(CONFIG_16550_PCI_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1_pci_port
#elif defined(CONFIG_16550_PCI_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2_pci_port
#elif defined(CONFIG_16550_PCI_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3_pci_port
#endif

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Extend default PCI devie type */

struct uart_pci_type_s
{
  FAR const struct pci_dev_type_s *type;     /* PCI device type */
  uint8_t                          ports;    /* Number of ports */
  uint8_t                          busy;     /* Port busy bits */
  uint8_t                          regincr;  /* Address increment */
  uint8_t                          portincr; /* Port address increment */
};

/* Extend default UART 16550 strucutre */

struct pci_u16550_s
{
  /* Common UART 16550 data must be first */

  struct u16550_s   common;
  FAR uart_dev_t   *dev;
  struct pci_dev_s  pcidev;
  uint16_t          vendor;
  uint16_t          device;
  uint8_t           port;
  char             *path;
  bool              is_msi;
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

static uart_datawidth_t pci_u16550_getreg_mem(FAR struct u16550_s *priv,
                                              unsigned int offset);
static void pci_u16550_putreg_mem(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value);
static uart_datawidth_t pci_u16550_getreg_io(FAR struct u16550_s *priv,
                                              unsigned int offset);
static void pci_u16550_putreg_io(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value);
static int pci_u16550_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

static FAR struct dma_chan_s *pci_u16550_dmachan(uart_addrwidth_t base,
                                                 unsigned int ident);
static int pci_u16550_interrupt(int irq, FAR void *context, FAR void *arg);
static int pci_u16550_register(FAR uart_dev_t *dev);
static int uart_16550_probe(FAR struct pci_bus_s *bus,
                            FAR const struct pci_dev_type_s *type,
                            uint16_t bdf);

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifdef CONFIG_16550_PCI_UART_QEMU
/* QEMU 16550 UART x1 */

const struct pci_dev_type_s g_pci_type_qemu_x1_16550 =
{
  .vendor    = 0x1b36,
  .device    = 0x0002,
  .class_rev = PCI_ID_ANY,
  .name      = "QEMU 16550 UART x1 PCI device",
  .probe     = uart_16550_probe
};

static struct uart_pci_type_s g_pci_qemu_x1_16550 =
{
  .type     = &g_pci_type_qemu_x1_16550,
  .ports    = 1,
  .busy     = 0,
  .regincr  = 1,
  .portincr = 0,
};

/* QEMU 16550 UART x2 */

const struct pci_dev_type_s g_pci_type_qemu_x2_16550 =
{
  .vendor    = 0x1b36,
  .device    = 0x0003,
  .class_rev = PCI_ID_ANY,
  .name      = "QEMU 16550 UART x2 PCI device",
  .probe     = uart_16550_probe
};

static struct uart_pci_type_s g_pci_qemu_x2_16550 =
{
  .type     = &g_pci_type_qemu_x2_16550,
  .ports    = 2,
  .busy     = 0,
  .regincr  = 1,
  .portincr = 8,
};

/* QEMU 16550 UART x4 */

const struct pci_dev_type_s g_pci_type_qemu_x4_16550 =
{
  .vendor    = 0x1b36,
  .device    = 0x0004,
  .class_rev = PCI_ID_ANY,
  .name      = "QEMU 16550 UART x4 PCI device",
  .probe     = uart_16550_probe
};

static struct uart_pci_type_s g_pci_qemu_x4_16550 =
{
  .type     = &g_pci_type_qemu_x4_16550,
  .ports    = 4,
  .busy     = 0,
  .regincr  = 1,
  .portincr = 8,
};
#endif  /* CONFIG_16550_PCI_UART_QEMU */

#ifdef CONFIG_16550_PCI_UART_AX99100
/*  AX99100 16550 UART x2 */

const struct pci_dev_type_s g_pci_type_ax99100_x2_16550 =
{
  .vendor    = 0x125b,
  .device    = 0x9100,
  .class_rev = PCI_ID_ANY,
  .name      = "AX99100 16550 UART x4 PCI device",
  .probe     = uart_16550_probe
};

static struct uart_pci_type_s g_pci_ax99100_x2_16550 =
{
  .type     = &g_pci_type_ax99100_x2_16550,
  .ports    = 2,
  .busy     = 0,
  .regincr  = 1,
  .portincr = 8,
};
#endif  /* CONFIG_16550_PCI_UART_AX99100 */

/* PCI devices */

static struct uart_pci_type_s *g_pci_16550_devs[] =
{
#ifdef CONFIG_16550_PCI_UART_QEMU
  &g_pci_qemu_x1_16550,
  &g_pci_qemu_x2_16550,
  &g_pci_qemu_x4_16550,
#endif
#ifdef CONFIG_16550_PCI_UART_AX99100
  &g_pci_ax99100_x2_16550
#endif
};

/* UART 16550 ops for MMIO operations */

static const struct u16550_ops_s g_pci_u16550_mem_ops =
{
  .isr        = pci_u16550_interrupt,
  .pci_getreg = pci_u16550_getreg_mem,
  .pci_putreg = pci_u16550_putreg_mem,
  .ioctl      = pci_u16550_ioctl,
  .dmachan    = pci_u16550_dmachan,
};

/* UART 16550 ops for IO operations */

static const struct u16550_ops_s g_pci_u16550_io_ops =
{
  .isr        = pci_u16550_interrupt,
  .pci_getreg = pci_u16550_getreg_io,
  .pci_putreg = pci_u16550_putreg_io,
  .ioctl      = pci_u16550_ioctl,
  .dmachan    = pci_u16550_dmachan,
};

/* I/O buffers */

#ifdef CONFIG_16550_PCI_UART0
static char g_uart0rxbuffer[CONFIG_16550_PCI_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_16550_PCI_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_16550_PCI_UART1
static char g_uart1rxbuffer[CONFIG_16550_PCI_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_16550_PCI_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_16550_PCI_UART2
static char g_uart2rxbuffer[CONFIG_16550_PCI_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_16550_PCI_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_16550_PCI_UART3
static char g_uart3rxbuffer[CONFIG_16550_PCI_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_16550_PCI_UART3_TXBUFSIZE];
#endif

/* This describes the state of the 16550 UART0 PCI port. */

#ifdef CONFIG_16550_PCI_UART0
static uart_dev_t g_uart0_pci_port;
static struct pci_u16550_s g_uart0_pci_priv =
{
  /* UART 16550 common data */

  .common =
  {
    .ops       = NULL,
    .uartbase  = 0,
    .pci       = true,
    .isrdev    = NULL,
    .regincr   = 0,
    .baud      = CONFIG_16550_PCI_UART0_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART0_CLOCK,
    .irq       = CONFIG_16550_PCI_UART0_IRQ,
    .parity    = CONFIG_16550_PCI_UART0_PARITY,
    .bits      = CONFIG_16550_PCI_UART0_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART0_2STOP,
#if defined(CONFIG_16550_PCI_UART0_IFLOWCONTROL) || \
  defined(CONFIG_16550_PCI_UART0_OFLOWCONTROL)
    .flow      = true,
#endif
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART0_VENDOR,
  .device      = CONFIG_16550_PCI_UART0_DEVICE,
  .port        = CONFIG_16550_PCI_UART0_PORT,
  .dev         = &g_uart0_pci_port,
  .pcidev      =
  {
    NULL,
    NULL,
    0
  },
  .path        = UART0_16550_PCI_DEVPATH
};

static uart_dev_t g_uart0_pci_port =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = NULL,
  .priv     = &g_uart0_pci_priv.common,
#ifdef CONFIG_16550_PCI_UART0_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

/* This describes the state of the 16550 UART1 PCI port. */

#ifdef CONFIG_16550_PCI_UART1
static uart_dev_t g_uart1_pci_port;
static struct pci_u16550_s g_uart1_pci_priv =
{
  /* UART 16550 common data */

  .common =
  {
    .ops       = NULL,
    .uartbase  = 0,
    .pci       = true,
    .isrdev    = NULL,
    .regincr   = 0,
    .baud      = CONFIG_16550_PCI_UART1_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART1_CLOCK,
    .irq       = CONFIG_16550_PCI_UART1_IRQ,
    .parity    = CONFIG_16550_PCI_UART1_PARITY,
    .bits      = CONFIG_16550_PCI_UART1_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART1_2STOP,
#if defined(CONFIG_16550_PCI_UART1_IFLOWCONTROL) || \
    defined(CONFIG_16550_PCI_UART1_OFLOWCONTROL)
    .flow      = true,
#endif
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART1_VENDOR,
  .device      = CONFIG_16550_PCI_UART1_DEVICE,
  .port        = CONFIG_16550_PCI_UART1_PORT,
  .dev         = &g_uart1_pci_port,
  .pcidev      =
  {
    NULL,
    NULL,
    0
  },
  .path        = UART1_16550_PCI_DEVPATH
};

static uart_dev_t g_uart1_pci_port =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = NULL,
  .priv     = &g_uart1_pci_priv.common,
#ifdef CONFIG_16550_PCI_UART1_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

/* This describes the state of the 16550 UART2 PCI port. */

#ifdef CONFIG_16550_PCI_UART2
static uart_dev_t g_uart2_pci_port;
static struct pci_u16550_s g_uart2_pci_priv =
{
  /* UART 16550 common data */

  .common =
  {
    .ops       = NULL,
    .uartbase  = 0,
    .pci       = true,
    .isrdev    = NULL,
    .regincr   = 0,
    .baud      = CONFIG_16550_PCI_UART2_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART2_CLOCK,
    .irq       = CONFIG_16550_PCI_UART2_IRQ,
    .parity    = CONFIG_16550_PCI_UART2_PARITY,
    .bits      = CONFIG_16550_PCI_UART2_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART2_2STOP,
#if defined(CONFIG_16550_PCI_UART2_IFLOWCONTROL) || \
    defined(CONFIG_16550_PCI_UART2_OFLOWCONTROL)
    .flow      = true,
#endif
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART2_VENDOR,
  .device      = CONFIG_16550_PCI_UART2_DEVICE,
  .port        = CONFIG_16550_PCI_UART2_PORT,
  .dev         = &g_uart2_pci_port,
  .pcidev      =
  {
    NULL,
    NULL,
    0
  },
  .path        = UART2_16550_PCI_DEVPATH
};

static uart_dev_t g_uart2_pci_port =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = NULL,
  .priv     = &g_uart2_pci_priv.common,
#ifdef CONFIG_16550_PCI_UART2_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

#ifdef CONFIG_16550_PCI_UART3
static uart_dev_t g_uart3_pci_port;
static struct pci_u16550_s g_uart3_pci_priv =
{
  /* UART 16550 common data */

  .common =
  {
    .ops       = NULL,
    .uartbase  = 0,
    .pci       = true,
    .isrdev    = NULL,
    .regincr   = 0,
    .baud      = CONFIG_16550_PCI_UART3_BAUD,
    .uartclk   = CONFIG_16550_PCI_UART3_CLOCK,
    .irq       = CONFIG_16550_PCI_UART3_IRQ,
    .parity    = CONFIG_16550_PCI_UART3_PARITY,
    .bits      = CONFIG_16550_PCI_UART3_BITS,
    .stopbits2 = CONFIG_16550_PCI_UART3_2STOP,
#if defined(CONFIG_16550_PCI_UART3_IFLOWCONTROL) || \
  defined(CONFIG_16550_PCI_UART3_OFLOWCONTROL)
    .flow      = true,
#endif
  },

  /* PCI specific data */

  .vendor      = CONFIG_16550_PCI_UART3_VENDOR,
  .device      = CONFIG_16550_PCI_UART3_DEVICE,
  .port        = CONFIG_16550_PCI_UART3_PORT,
  .dev         = &g_uart3_pci_port,
  .pcidev      =
  {
    NULL,
    NULL,
    0
  },
  .path        = UART3_16550_PCI_DEVPATH
};

static uart_dev_t g_uart3_pci_port =
{
  .recv     =
  {
    .size   = CONFIG_16550_PCI_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_PCI_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = NULL,
  .priv     = &g_uart3_pci_priv.common,
#ifdef CONFIG_16550_PCI_UART3_SERIAL_CONSOLE
  .isconsole = true,
#endif
};
#endif

/* PCI devices */

static struct pci_u16550_s *g_pci_u16550_inst[] =
{
#ifdef CONFIG_16550_PCI_UART0
  &g_uart0_pci_priv,
#endif
#ifdef CONFIG_16550_PCI_UART1
  &g_uart1_pci_priv,
#endif
#ifdef CONFIG_16550_PCI_UART2
  &g_uart2_pci_priv,
#endif
#ifdef CONFIG_16550_PCI_UART3
  &g_uart3_pci_priv,
#endif
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pci_u16550_getreg_mem
 *****************************************************************************/

static uart_datawidth_t pci_u16550_getreg_mem(FAR struct u16550_s *priv,
                                              unsigned int offset)
{
  uintptr_t addr = priv->uartbase + offset;

  return *((FAR volatile uart_datawidth_t *)(addr));
}

/*****************************************************************************
 * Name: pci_u16550_putreg_mem
 *****************************************************************************/

static void pci_u16550_putreg_mem(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value)
{
  uintptr_t addr = priv->uartbase + offset;

  *((FAR volatile uart_datawidth_t *)(addr)) = value;
}

/*****************************************************************************
 * Name: pci_u16550_getreg_io
 *****************************************************************************/

static uart_datawidth_t pci_u16550_getreg_io(FAR struct u16550_s *priv,
                                             unsigned int offset)
{
  FAR struct pci_u16550_s *p    = (FAR struct pci_u16550_s *)priv;
  uintptr_t                addr = priv->uartbase + offset;

  return p->pcidev.bus->ops->pci_io_read((void *)(addr), 1);
}

/*****************************************************************************
 * Name: pci_u16550_putreg_io
 *****************************************************************************/

static void pci_u16550_putreg_io(FAR struct u16550_s *priv,
                                  unsigned int offset,
                                  uart_datawidth_t value)
{
  FAR struct pci_u16550_s *p    = (FAR struct pci_u16550_s *)priv;
  uintptr_t                addr = priv->uartbase + offset;

  p->pcidev.bus->ops->pci_io_write((void *)(addr), value, 1);
}

/*****************************************************************************
 * Name: pci_u16550_ioctl
 *****************************************************************************/

static int pci_u16550_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}

/*****************************************************************************
 * Name: pci_u16550_dmachan
 *****************************************************************************/

static FAR struct dma_chan_s *pci_u16550_dmachan(uart_addrwidth_t base,
                                                 unsigned int ident)
{
  return NULL;
}

/*****************************************************************************
 * Name: pci_u16550_interrupt
 *
 * Description:
 *   Handle PCI interrupt.
 *
 *****************************************************************************/

static int pci_u16550_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s      *dev  = (struct uart_dev_s *)arg;
  FAR struct pci_u16550_s    *priv = NULL;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (FAR struct pci_u16550_s *)dev->priv;

  /* Not shared interupt if MSI enabled */

  if (priv->is_msi == true)
    {
      uart_16550_interrupt(dev);
    }

  /* Check PCI interrupt status for this device */

  else if (pci_int_stat(&priv->pcidev))
    {
      uart_16550_interrupt(dev);
    }

  return OK;
}

/*****************************************************************************
 * Name: pci_u16550_initialize
 *
 * Description:
 *   Initialize UART 16550 PCI device.
 *
 *****************************************************************************/

static int pci_u16550_initialize(FAR struct pci_u16550_s    *priv,
                                 FAR struct uart_pci_type_s *drv_priv,
                                 uintptr_t                   bar_addr,
                                 FAR struct pci_dev_s       *dev,
                                 bool                        mmio)
{
  int     ret = 0;
  uint8_t irq = 0;

  /* Configure UART PCI */

  priv->common.uartbase = bar_addr;
  pciinfo("uartbase %p", priv->common.uartbase);

  if (mmio)
    {
      priv->common.ops = &g_pci_u16550_mem_ops;
    }
  else
    {
      priv->common.ops = &g_pci_u16550_io_ops;
    }

  priv->common.isrdev  = priv->dev;
  priv->common.regincr = drv_priv->regincr;

  /* Store PCI dev */

  priv->pcidev.bus  = dev->bus;
  priv->pcidev.type = dev->type;
  priv->pcidev.bdf  = dev->bdf;

  /* Configure MSI if supported */

  ret = pci_msi_connect(dev, priv->common.irq, 1);
  if (ret == OK)
    {
      priv->is_msi = true;
      return OK;
    }

  /* If MSI not supported - verify if legacy IRQ match configuration */

  irq = dev->bus->ops->pci_cfg_read(dev, PCI_HEADER_NORM_INT_LINE, 1) + IRQ0;
  if (irq != priv->common.irq)
    {
      pcierr("UART PCI IRQ doesn't match configuration %d != %d",
             irq, priv->common.irq);
      return -EINVAL;
    }

  return OK;
}

/*****************************************************************************
 * Name: pci_u16550_register
 *
 * Description:
 *   Register UART 16550 PCI device.
 *
 *****************************************************************************/

static int pci_u16550_register(FAR uart_dev_t *dev)
{
  FAR struct pci_u16550_s *p   = (FAR struct pci_u16550_s *)dev->priv;
  int                      ret = OK;

  /* Bind with 16550 common driver */

  ret = uart_16550_bind(dev);
  if (ret < 0)
    {
      /* No associated device found */

      return -ENODEV;
    }

  DEBUGASSERT(dev->ops);

  /* Register driver */

  pciinfo("Register %s", p->path);
  return uart_register(p->path, dev);
}

/*****************************************************************************
 * Name: pci_u16550_probe
 *
 * Description:
 *   Initialize device.
 *
 *****************************************************************************/

static int uart_16550_probe(FAR struct pci_bus_s *bus,
                            FAR const struct pci_dev_type_s *type,
                            uint16_t bdf)
{
  FAR struct uart_pci_type_s     *drv_priv = NULL;
  FAR static struct pci_u16550_s *priv     = NULL;
  struct pci_dev_s                dev;
  uint32_t                        bar;
  uintptr_t                       bar_addr;
  uint8_t                         i;
  uint8_t                         port;
  uint8_t                         bar_id;
  bool                            mmio;
  int                             ret;
  irqstate_t                      flags;

  /* Get priv data associated with this PCI device type */

  for (i = 0; i < sizeof(g_pci_16550_devs) / sizeof(uintptr_t); i++)
    {
      if (type == g_pci_16550_devs[i]->type)
        {
          drv_priv = g_pci_16550_devs[i];
          break;
        }
    }

  /* Not found private data */

  if (drv_priv == NULL)
    {
      return -ENODEV;
    }

  /* Get PCI dev */

  dev.bus  = bus;
  dev.type = type;
  dev.bdf  = bdf;

  pci_enable_bus_master(&dev);
  pci_enable_io(&dev, PCI_SYS_RES_MEM);
  pci_enable_io(&dev, PCI_SYS_RES_IOPORT);

  /* Hardcode BAR 0 for now */

  bar_id = 0;
  mmio   = false;

  /* Need to query the BAR for IO vs MEM
   * Also handle if the bar is 64bit address
   */

  if (pci_bar_valid(&dev, bar_id) != OK)
    {
      return -ENODEV;
    }

  bar = bus->ops->pci_cfg_read(&dev,
                               PCI_HEADER_NORM_BAR0 + (bar_id * 4), 4);
  bar_addr = pci_bar_addr(&dev, bar_id);

  if ((bar & PCI_BAR_LAYOUT_MASK) == PCI_BAR_LAYOUT_MEM)
    {
      /* If the BAR is MMIO then it must be mapped */

      bus->ops->pci_map_bar(bar_addr, pci_bar_size(&dev, bar_id));
      mmio = true;
    }

  for (port = 0; port < drv_priv->ports; port++)
    {
      /* Get port address */

      bar_addr += drv_priv->portincr * port;

      /* Take the instance that matches the configuration */

      priv = NULL;
      for (i = 0; i < sizeof(g_pci_u16550_inst) / sizeof(uintptr_t); i++)
        {
          if (g_pci_u16550_inst[i]->vendor == type->vendor &&
              g_pci_u16550_inst[i]->device == type->device &&
              g_pci_u16550_inst[i]->port == port)
            {
              priv = g_pci_u16550_inst[i];
              break;
            }
        }

      /* Not found */

      if (priv == NULL)
        {
          return -ENODEV;
        }

      flags = enter_critical_section();

      /* Device already registered */

      if (drv_priv->busy & (1 << i))
        {
          leave_critical_section(flags);
          return -EBUSY;
        }

      /* Mark this port busy */

      drv_priv->busy |= (1 << i);
      leave_critical_section(flags);

      /* Initialize device */

      ret = pci_u16550_initialize(priv, drv_priv, bar_addr, &dev, mmio);
      if (ret < 0)
        {
          return ret;
        }

      /* Register UART device */

      ret = pci_u16550_register(priv->dev);
      if (ret < 0)
        {
          return ret;
        }

#ifdef CONSOLE_DEV
      /* Register console */

      if (priv->dev->isconsole)
        {
          uart_register("/dev/console", &CONSOLE_DEV);
        }
#endif
    }

  return OK;
}

/*****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 *****************************************************************************/

#ifdef HAVE_16550_PCI_CONSOLE
int up_putc(int ch)
{
  irqstate_t flags;

  /* Console not initialized yet */

  if (CONSOLE_DEV.ops == NULL)
    {
      return ch;
    }

  /* All interrupts must be disabled to prevent re-entrancy and to prevent
   * interrupts from firing in the serial driver code.
   */

  flags = enter_critical_section();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      uart_16550_putc(&CONSOLE_DEV, '\r');
    }

  uart_16550_putc(&CONSOLE_DEV, ch);
  leave_critical_section(flags);

  return ch;
}
#endif
