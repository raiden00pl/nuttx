/****************************************************************************
 * drivers/sensors/lps22h.c
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

#include <stdlib.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lps22h.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPS22H_I2C_ADDR     0x5d
#define LPS22H_I2C_ADDR     0x5c

#define LPS22H_I2C_FREQ     400000

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct lps22h_dev_s
{
#ifdef CONFIG_SENSORS_LPS22H_I2C
  FAR struct i2c_master_s  *i2c;   /* I2C interface */
  uint8_t                   addr;  /* I2C address */
  int                       freq;  /* Frequency */
#endif

#ifdef CONFIG_SENSORS_LPS22H_SPI
  FAR struct spi_dev_s     *spi;       /* SPI interface */
#endif

  sem_t devsem;
  sem_t waitsem;

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t lps22h_getreg8(FAR struct lps22h_dev_s *priv,
                              uint8_t regaddr);
static void lps22h_putreg8(FAR struct lps22h_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);

/* Character driver methods */

static int     lps22h_open(FAR struct file *filep);
static int     lps22h_close(FAR struct file *filep);
static ssize_t lps22h_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static int     lps22h_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

static int lps22h_checkid(FAR struct lps22h_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_lps22hfops =
{
  lps22h_open,     /* open */
  lps22h_close,    /* close */
  lps22h_read,     /* read */
  NULL,            /* write */
  NULL,            /* seek */
  lps22h_ioctl,    /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: lps22h_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LPS22H_SPI
static inline void lps22h_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the LPS22H */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_LPS22H_SPI_FREQUENCY);
}
#endif

static uint8_t lps22h_getreg8(FAR struct lps22h_dev_s *priv,
                              uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_LPS22H_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_LPS22H_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  lps22h_configspi(priv->spi);

  /* Select the LPS22H */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(0), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the LPS22H */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif

  return regval;
}

static void lps22h_putreg8(FAR struct lps22h_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
#ifdef CONFIG_SENSORS_LPS22H_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_LPS22H_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  lps22h_configspi(priv->spi);

  /* Select the LPS22H */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(0), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the LPS22H */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif
}

 int lps22h_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lps22h_dev_s *dev = inode->i_private;
  int32_t ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }


  /* tests!!!! */
  uint8_t regval = 0;

  printf("------------------------\n");
  printf("------------------------\n");

  /* one shot */
  lps22h_putreg8(dev, LPS22H_CMD_CTRL_REG2, 1);

  regval = lps22h_getreg8(dev, 0x28);
  printf("28=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x29);
  printf("29=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x2a);
  printf("2a=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x2b);
  printf("2b=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x2c);
  printf("2c=%d\n", regval);

  printf("------------------------\n");

  /* one shot */
  lps22h_putreg8(dev, LPS22H_CMD_CTRL_REG2, 1);

  regval = lps22h_getreg8(dev, 0x28);
  printf("28=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x29);
  printf("29=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x2a);
  printf("2a=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x2b);
  printf("2b=%d\n", regval);
  regval = lps22h_getreg8(dev, 0x2c);
  printf("2c=%d\n", regval);
  printf("------------------------\n");
  printf("------------------------\n");






  nxsem_post(&dev->devsem);
  return ret;
}

/****************************************************************************
 * Name: lps22h_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int lps22h_close(FAR struct file *filep)
{
}

/****************************************************************************
 * Name: lps22h_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t lps22h_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
}

/****************************************************************************
 * Name: lps22h_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int lps22h_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct lps22h_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: lps22h_checkid
 *
 * Description:
 *   Read and verify the LPS22H chip ID
 *
 ****************************************************************************/

static int lps22h_checkid(FAR struct lps22h_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = lps22h_getreg8(priv, LPS22H_CMD_WHO_AM_I);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) WHO_AM_I_DEFAULT)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lps22h_register
 *
 * Description:
 *   Register the LPS22H character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the I2C/SPI interface to use to communicate with
 *             LPS22H
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LPS22H_I2C
int lps22h_register(FAR const char *devpath, FAR struct i2c_master_s *dev)
#else /* CONFIG_SENSORS_LPS22H_SPI */
int lps22h_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
#endif
{
  FAR struct lps22h_dev_s *priv;
  int ret;

  priv = (FAR struct lps22h_dev_s *)kmm_malloc(sizeof(struct lps22h_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_LPS22H_I2C
  priv->i2c = dev;
  priv->addr = LPS22H_I2C_ADDR;
  priv->freq = LPS22H_I2C_FREQ;
#else /* CONFIG_SENSORS_LPS22H_SPI */
  priv->spi = dev;
#endif

#ifdef CONFIG_LPS22H_SPI_3WIRE
  /* Configure a SPI 3-wire interface */

  lps22h_putreg8(priv, LPS22H_CMD_CTRL_REG1, CTRL_REG1_3WIRE);

  /* Disable the automatic adress incremetation */

  lps22h_putreg8(priv, LPS22H_CMD_CTRL_REG2, 0);
#endif

  ret = lps22h_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  nxsem_init(&priv->devsem, 0, 1);
  nxsem_init(&priv->waitsem, 0, 0);

  /* Register driver */

  ret = register_driver(devpath, &g_lps22hfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("LPS22H driver loaded successfully!\n");
  return OK;
}
