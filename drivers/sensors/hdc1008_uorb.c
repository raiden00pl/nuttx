/****************************************************************************
 * drivers/sensors/hdc1008_uorb.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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
#include <nuttx/nuttx.h>

#include <debug.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/hdc1008.h>
#include <nuttx/signal.h>

#include "hdc1008_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

/* Sensor information for the lowerhalf sensors.
 * Since the HDC1008 has both a relative humidity and temperature sensor,
 * two lower halves are needed which will follow this structure.
 */

struct hdc1008_sensor_s
{
  FAR struct sensor_lowerhalf_s sensor_lower; /* Lower-half driver */
  FAR struct hdc1008_dev_s *dev;
  bool enabled;
};

/* Represents the main device, with two lower halves for both data types. */

struct hdc1008_dev_s
{
  struct hdc1008_sensor_s  hum;      /* Humidity lower-half */
  struct hdc1008_sensor_s  temp;     /* Temperature lower-half */
  FAR struct i2c_master_s *i2c;      /* I2C interface. */
  uint8_t                  addr;     /* I2C address. */
  sem_t                    run;      /* Lock for the polling measurement cycle */
  uint32_t                 interval; /* Measurement interval for polling in us */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hdc1008_getreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval);
static int hdc1008_putreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regval);
static int hdc1008_reset(FAR struct hdc1008_dev_s *priv);
static int hdc1008_measure_trh(FAR struct hdc1008_dev_s *priv,
                               FAR uint16_t *t, FAR uint16_t *h);
static int hdc1008_read(FAR struct hdc1008_dev_s *priv,
                        FAR struct sensor_humi *humi,
                        FAR struct sensor_temp *temp);

static int hdc1008_set_interval(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *period_us);
static int hdc1008_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable);
#ifndef CONFIG_SENSORS_HDC1008_POLL
static int hdc1008_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
#else
static int hdc1008_thread(int argc, char **argv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
    .activate = hdc1008_activate,
#ifdef CONFIG_SENSORS_HDC1008_POLL
    .fetch = NULL,
#else
    .fetch = hdc1008_fetch,
#endif
    .set_interval = hdc1008_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hdc1008_getreg
 *
 * Description:
 *   Get value of 16-bit register
 *
 ****************************************************************************/

static int hdc1008_getreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval)
{
  int ret;
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = i2c_read(priv->i2c, &config, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  *regval = (buf[0] << 8) | buf[1];

  return ret;
}

/****************************************************************************
 * Name: hdc1008_putreg
 *
 * Description:
 *   Set value of 16-bit register
 *
 ****************************************************************************/

static int hdc1008_putreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t buf[3];

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

  buf[0] = regaddr;
  buf[1] = (uint8_t)((regval >> 8) & 0xff);
  buf[2] = (uint8_t)(regval & 0xff);

  return i2c_write(priv->i2c, &config, buf, 3);
}

/****************************************************************************
 * Name: hdc1008_reset
 *
 * Description:
 *   Perform a software reset of the sensor
 *
 ****************************************************************************/

static int hdc1008_reset(FAR struct hdc1008_dev_s *priv)
{
  uint16_t reg;
  int count = 10;
  int ret;

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      return ret;
    }

  ret = hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION,
                       reg | HDC1008_CONFIGURATION_RST);
  if (ret < 0)
    {
      return ret;
    }

  /* Now we wait until the sensor has reset */

  do
    {
      ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
      nxsig_usleep(1000);
      --count;
    }
  while ((reg & HDC1008_CONFIGURATION_RST) && (ret == OK) && count);

  return ret;
}

/****************************************************************************
 * Name: hdc1008_measure_trh
 *
 * Description:
 *   Read both temperature and humidity from the sensor
 *
 ****************************************************************************/

static int hdc1008_measure_trh(FAR struct hdc1008_dev_s *priv,
                               FAR uint16_t *t, FAR uint16_t *h)
{
  struct i2c_config_s config;
  uint8_t             buf[4];
  uint8_t             reg = HDC1008_REG_TEMPERATURE;
  int                 ret;

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, &reg, 1);

  /* Wait for measurement to complete. Max should be about 20 ms if measuring
   * both temperature and humidity.
   */

  nxsig_usleep(20000);

  ret = i2c_read(priv->i2c, &config, buf, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Return raw data from sensor to temperature/humidity */

  *t = ((int)buf[0] << 8) | (int)buf[1];
  *h = ((int)buf[2] << 8) | (int)buf[3];

  return 0;
}

/****************************************************************************
 * Name: hdc1008_read
 *
 * Description: Read temperature and humidity into UORB sensor formats.
 *
 * Return:
 *   Negated error code, or 0 on success.
 *
 ****************************************************************************/

static int hdc1008_read(FAR struct hdc1008_dev_s *priv,
                        FAR struct sensor_humi *humi,
                        FAR struct sensor_temp *temp)
{
  uint16_t raw_temp;
  uint16_t raw_hum;
  int      ret;

  /* Get temp and humidity */

  ret = hdc1008_measure_trh(priv, &raw_temp, &raw_hum);
  if (ret < 0)
    {
      return ret;
    }

  /* Store results */

  humi->timestamp   = sensor_get_timestamp();
  humi->humidity    = RAW_TO_RH(raw_hum) / 10.0f;
  temp->timestamp   = humi->timestamp;
  temp->temperature = RAW_TO_TEMP(raw_temp) / 100.0f;

  return ret;
}

/****************************************************************************
 * Name: hdc1008_set_interval
 *
 * Description:
 *     Sets the measurement interval for the HDC1008 sensor in microseconds.
 *
 ****************************************************************************/

static int hdc1008_set_interval(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *period_us)
{
  FAR struct hdc1008_sensor_s *priv =
      container_of(lower, FAR struct hdc1008_sensor_s, sensor_lower);
  FAR struct hdc1008_dev_s *dev = priv->dev;
  dev->interval = *period_us;
  return 0;
}

/****************************************************************************
 * Name: hdc1008_activate
 ****************************************************************************/

static int hdc1008_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable)
{
  bool start_thread = false;
  FAR struct hdc1008_sensor_s *priv =
      container_of(lower, FAR struct hdc1008_sensor_s, sensor_lower);
  FAR struct hdc1008_dev_s *dev = priv->dev;

  if (enable)
    {
      if (!priv->enabled)
        {
          start_thread = true;
        }
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the polling thread */

      nxsem_post(&dev->run);
    }

  return 0;
}

/****************************************************************************
 * Name: hdc1008_fetch
 ****************************************************************************/

#ifndef CONFIG_SENSORS_HDC1008_POLL
static int hdc1008_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct hdc1008_sensor_s *priv =
      container_of(lower, FAR struct hdc1008_sensor_s, sensor_lower);
  FAR struct hdc1008_dev_s *dev = priv->dev;
  struct sensor_temp temp_data;
  struct sensor_humi humi_data;
  int ret;

  ret = hdc1008_read(dev, &humi_data, &temp_data);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->hum.enabled)
    {
      dev->hum.sensor_lower.push_event(dev->hum.sensor_lower.priv,
                                       &humi_data, sizeof(humi_data));
    }

  if (dev->temp.enabled)
    {
      dev->temp.sensor_lower.push_event(dev->temp.sensor_lower.priv,
                                        &temp_data, sizeof(temp_data));
    }

  return ret;
}
#else
/****************************************************************************
 * Name: hdc1008_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number of arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int hdc1008_thread(int argc, FAR char **argv)
{
  FAR struct hdc1008_dev_s *dev =
      (FAR struct hdc1008_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  struct sensor_temp temp_data;
  struct sensor_humi humi_data;
  int                ret;

  while (true)
    {
      if (!dev->hum.enabled && !dev->temp.enabled)
        {
          /* Wait for one of the lower halves to be enabled and wake us up */

          ret = nxsem_wait(&dev->run);
          if (ret < 0)
            {
              continue;
            }
        }

      ret = hdc1008_read(dev, &humi_data, &temp_data);
      if (ret < 0)
        {
          continue;
        }

      if (dev->hum.enabled)
        {
          dev->hum.sensor_lower.push_event(dev->hum.sensor_lower.priv,
                                           &humi_data, sizeof(humi_data));
        }

      if (dev->temp.enabled)
        {
          dev->temp.sensor_lower.push_event(dev->temp.sensor_lower.priv,
                                            &temp_data, sizeof(temp_data));
        }

      /* Sleep before next fetch */

      nxsig_usleep(dev->interval);
    }

  return OK;
}
#endif /* CONFIG_SENSORS_HDC1008_POLL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hdc1008_register_uorb
 *
 * Description:
 *   Register the HDC1008 as UORB device.
 *
 * Input Parameters:
 *   devno   - device number
 *   config  - device configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hdc1008_register_uorb(int devno, FAR struct hdc1008_config_s *config)
{
  FAR struct hdc1008_dev_s *priv;
  FAR char                 *argv[2];
  char                      arg1[32];
  int                       ret;

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(config->i2c != NULL);
  DEBUGASSERT(config->addr == 0x40 || config->addr == 0x41 ||
              config->addr == 0x42 || config->addr == 0x43);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct hdc1008_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance.\n");
      return -ENOMEM;
    }

  priv->i2c      = config->i2c;
  priv->addr     = config->addr;
  priv->interval = 1000000;

  ret = nxsem_init(&priv->run, 0, 0);
  if (ret < 0)
    {
      snerr("Failed to register HDC1008 driver: %d\n", ret);
      goto errout_no_hum;
    }

  ret = hdc1008_reset(priv);
  if (ret < 0)
    {
      snerr("Failed to reset HDC1008 driver: %d\n", ret);
    }

  /* Register lower half for humidity */

  priv->hum.sensor_lower.ops  = &g_sensor_ops;
  priv->hum.sensor_lower.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  priv->hum.enabled           = false;
  priv->hum.dev               = priv;

  /* Register UORB Sensor */

  ret = sensor_register(&priv->hum.sensor_lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register HDC1008 driver: %d\n", ret);
      goto errout_no_temp;
    }

  /* Register lower half for temperature */

  priv->temp.sensor_lower.ops  = &g_sensor_ops;
  priv->temp.sensor_lower.type = SENSOR_TYPE_TEMPERATURE;
  priv->temp.enabled           = false;
  priv->temp.dev               = priv;

  /* Register UORB Sensor */

  ret = sensor_register(&priv->temp.sensor_lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register HDC1008 driver: %d\n", ret);
      goto errout_no_poll;
    }

#ifdef CONFIG_SENSORS_HDC1008_POLL
  /* Polling thread */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("hdc1008_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_HDC1008_THREAD_STACKSIZE,
                       hdc1008_thread, argv);
  if (ret < 0)
    {
      snerr("Failed to create the HDC1008 notification kthread.\n");
      goto errout;
    }
#endif

  sninfo("Registered HDC1008 driver.\n");

  return OK;

errout:
  sensor_unregister(&priv->temp.sensor_lower, devno);
errout_no_poll:
  sensor_unregister(&priv->hum.sensor_lower, devno);
errout_no_temp:
  nxsem_destroy(&priv->run);
errout_no_hum:
  kmm_free(priv);

  return ret;
}
