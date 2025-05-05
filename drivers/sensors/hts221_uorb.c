/****************************************************************************
 * drivers/sensors/hts221_uorb.c
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
#include <stdio.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/hts221.h>

#include "hts221_base.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_HTS221)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CTRL_REG1: power on, block-data-update, one-shot output data rate */

#define HTS221_CTRL_REG1_VAL (HTS221_CTRL_REG1_PD | HTS221_CTRL_REG1_BDU)

/* AV_CONF: temperature averaging 16, humidity averaging 32 (default) */

#define HTS221_AV_CONF_VAL   (0x1b)

/* One-shot conversion timeout */

#define HTS221_ONE_SHOT_RETRIES 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Each measurement channel needs its own lower-half. */

struct hts221_sensor_s
{
  FAR struct sensor_lowerhalf_s lower;   /* Lower-half driver */
  FAR struct hts221_dev_s      *dev;     /* Parent device */
  bool                          enabled; /* Channel activated */
};

struct hts221_dev_s
{
  struct hts221_sensor_s   temp;     /* Temperature lower-half */
  struct hts221_sensor_s   hum;      /* Humidity lower-half */
  FAR struct i2c_master_s *i2c;      /* I2C interface */
  uint8_t                  addr;     /* I2C address */
  unsigned long            interval; /* Polling interval in us */
  struct
  {
    int16_t t0_out;
    int16_t t1_out;
    int16_t h0_t0_out;
    int16_t h1_t0_out;
    unsigned int t0_x8:10;
    unsigned int t1_x8:10;
    uint8_t h0_x2;
    uint8_t h1_x2;
  } calib;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hts221_read_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int hts221_write_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                            uint8_t regval);
static int hts221_load_calibration(FAR struct hts221_dev_s *priv);
static int hts221_configure(FAR struct hts221_dev_s *priv);
static int hts221_read_convert(FAR struct hts221_dev_s *priv,
                               FAR int *temperature, FAR int *humidity);

/* Sensor lower-half operations */

static int hts221_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int hts221_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);
static int hts221_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_hts221_ops =
{
  .activate     = hts221_activate,
  .set_interval = hts221_set_interval,
  .fetch        = hts221_fetch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hts221_read_reg
 ****************************************************************************/

static int hts221_read_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_config_s config;

  config.frequency = CONFIG_HTS221_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  return i2c_writeread(priv->i2c, &config, &regaddr, 1, regval, 1);
}

/****************************************************************************
 * Name: hts221_write_reg
 ****************************************************************************/

static int hts221_write_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                            uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = CONFIG_HTS221_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buf[0] = regaddr;
  buf[1] = regval;

  return i2c_write(priv->i2c, &config, buf, 2);
}

/****************************************************************************
 * Name: hts221_load_calibration
 *
 * Description:
 *   Read the factory calibration coefficients used to convert raw readings
 *   into physical units.
 *
 ****************************************************************************/

static int hts221_load_calibration(FAR struct hts221_dev_s *priv)
{
  uint8_t t0_degc_x8   = 0;
  uint8_t t1_degc_x8   = 0;
  uint8_t t1_t0_msb    = 0;
  uint8_t t0_out_lsb   = 0;
  uint8_t t0_out_msb   = 0;
  uint8_t t1_out_lsb   = 0;
  uint8_t t1_out_msb   = 0;
  uint8_t h0_rh_x2     = 0;
  uint8_t h1_rh_x2     = 0;
  uint8_t h0t0_out_lsb = 0;
  uint8_t h0t0_out_msb = 0;
  uint8_t h1t0_out_lsb = 0;
  uint8_t h1t0_out_msb = 0;
  int ret;

  ret = hts221_read_reg(priv, HTS221_CALIB_T0_DEGC_X8, &t0_degc_x8);
  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_DEGC_X8, &t1_degc_x8);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_T0_MSB, &t1_t0_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T0_OUT_L, &t0_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T0_OUT_H, &t0_out_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_OUT_L, &t1_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_OUT_H, &t1_out_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H0_RH_X2, &h0_rh_x2);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H1_RH_X2, &h1_rh_x2);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H0T0_OUT_L, &h0t0_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H0T0_OUT_H, &h0t0_out_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H1T0_OUT_L, &h1t0_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H1T0_OUT_H, &h1t0_out_msb);
    }

  if (ret < 0)
    {
      snerr("ERROR: Failed to read HTS221 calibration data: %d\n", ret);
      return ret;
    }

  priv->calib.t0_x8     = t0_degc_x8 | ((t1_t0_msb & 0x3) << 8);
  priv->calib.t1_x8     = t1_degc_x8 | ((t1_t0_msb & (0x3 << 2)) << (8 - 2));
  priv->calib.t0_out    = t0_out_lsb | (t0_out_msb << 8);
  priv->calib.t1_out    = t1_out_lsb | (t1_out_msb << 8);
  priv->calib.h0_x2     = h0_rh_x2;
  priv->calib.h1_x2     = h1_rh_x2;
  priv->calib.h0_t0_out = h0t0_out_lsb | (h0t0_out_msb << 8);
  priv->calib.h1_t0_out = h1t0_out_lsb | (h1t0_out_msb << 8);

  /* As calibration coefficients are unique to each sensor device,
   * they are a good candidate to be added to the entropy pool.
   */

  up_rngaddentropy(RND_SRC_HW, (FAR uint32_t *)&priv->calib,
                   sizeof(priv->calib) / sizeof(uint32_t));

  return OK;
}

/****************************************************************************
 * Name: hts221_configure
 *
 * Description:
 *   Reset the sensor, program averaging and enable one-shot mode.
 *
 ****************************************************************************/

static int hts221_configure(FAR struct hts221_dev_s *priv)
{
  uint8_t regval;
  int retries = HTS221_ONE_SHOT_RETRIES;
  int ret;

  /* Trigger a reboot to reload the trimming parameters */

  ret = hts221_write_reg(priv, HTS221_CTRL_REG2, HTS221_CTRL_REG2_BOOT);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      nxsig_usleep(10000);
      ret = hts221_read_reg(priv, HTS221_CTRL_REG2, &regval);
      if (ret < 0)
        {
          return ret;
        }
    }
  while ((regval & HTS221_CTRL_REG2_BOOT) && --retries);

  /* Program averaging configuration */

  ret = hts221_write_reg(priv, HTS221_AV_CONF, HTS221_AV_CONF_VAL);
  if (ret < 0)
    {
      return ret;
    }

  /* Power on the device in one-shot mode with block-data-update */

  return hts221_write_reg(priv, HTS221_CTRL_REG1, HTS221_CTRL_REG1_VAL);
}

/****************************************************************************
 * Name: hts221_read_convert
 *
 * Description:
 *   Trigger a one-shot conversion and convert the raw readings into
 *   temperature (in 0.01 degrees Celsius) and humidity (in 0.1 percent)
 *   using the factory calibration.
 *
 ****************************************************************************/

static int hts221_read_convert(FAR struct hts221_dev_s *priv,
                               FAR int *temperature, FAR int *humidity)
{
  uint8_t regval;
  uint8_t h_l;
  uint8_t h_h;
  uint8_t t_l;
  uint8_t t_h;
  int16_t t_out;
  int16_t h_out;
  int retries = HTS221_ONE_SHOT_RETRIES;
  int x1_x0_diff;
  int64_t y;
  int ret;

  /* Trigger a one-shot conversion */

  ret = hts221_write_reg(priv, HTS221_CTRL_REG2, HTS221_CTRL_REG2_ONE_SHOT);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for temperature and humidity data to be available */

  do
    {
      nxsig_usleep(5000);
      ret = hts221_read_reg(priv, HTS221_STATUS_REG, &regval);
      if (ret < 0)
        {
          return ret;
        }
    }
  while (((regval & (HTS221_STATUS_REG_H_DA | HTS221_STATUS_REG_T_DA)) !=
          (HTS221_STATUS_REG_H_DA | HTS221_STATUS_REG_T_DA)) && --retries);

  if (retries == 0)
    {
      snerr("ERROR: HTS221 conversion timeout\n");
      return -ETIMEDOUT;
    }

  ret = hts221_read_reg(priv, HTS221_HUM_OUT_L, &h_l);
  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_HUM_OUT_H, &h_h);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_TEMP_OUT_L, &t_l);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_TEMP_OUT_H, &t_h);
    }

  if (ret < 0)
    {
      return ret;
    }

  t_out = t_h << 8 | t_l;
  h_out = h_h << 8 | h_l;

  add_sensor_randomness(h_out ^ t_out);

  /* Linear interpolation between the two factory calibration points.
   * The result is scaled by HTS221_TEMPERATURE_PRECISION (0.01 degC) and
   * HTS221_HUMIDITY_PRECISION (0.1 %rH) respectively.
   */

  x1_x0_diff = priv->calib.t1_out - priv->calib.t0_out;
  y  = priv->calib.t0_x8 * x1_x0_diff +
       (priv->calib.t1_x8 - priv->calib.t0_x8) *
       (t_out - priv->calib.t0_out);
  y *= HTS221_TEMPERATURE_PRECISION;
  y /= x1_x0_diff * 8;
  *temperature = (int)y;

  x1_x0_diff = priv->calib.h1_t0_out - priv->calib.h0_t0_out;
  y  = priv->calib.h0_x2 * x1_x0_diff +
       (priv->calib.h1_x2 - priv->calib.h0_x2) *
       (h_out - priv->calib.h0_t0_out);
  y *= HTS221_HUMIDITY_PRECISION;
  y /= x1_x0_diff * 2;
  *humidity = (int)y;

  return OK;
}

/****************************************************************************
 * Name: hts221_activate
 ****************************************************************************/

static int hts221_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct hts221_sensor_s *priv =
      container_of(lower, FAR struct hts221_sensor_s, lower);

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: hts221_set_interval
 ****************************************************************************/

static int hts221_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct hts221_sensor_s *priv =
      container_of(lower, FAR struct hts221_sensor_s, lower);

  priv->dev->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: hts221_fetch
 ****************************************************************************/

static int hts221_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct hts221_sensor_s *priv =
      container_of(lower, FAR struct hts221_sensor_s, lower);
  FAR struct hts221_dev_s *dev = priv->dev;
  uint64_t timestamp;
  int temperature;
  int humidity;
  int ret;

  ret = hts221_read_convert(dev, &temperature, &humidity);
  if (ret < 0)
    {
      return ret;
    }

  timestamp = sensor_get_timestamp();

  if (lower->type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      struct sensor_temp temp;

      if (buflen != sizeof(temp))
        {
          return -EINVAL;
        }

      temp.timestamp   = timestamp;
      temp.temperature = (float)temperature / HTS221_TEMPERATURE_PRECISION;
      memcpy(buffer, &temp, sizeof(temp));
      return sizeof(temp);
    }
  else
    {
      struct sensor_humi humi;

      if (buflen != sizeof(humi))
        {
          return -EINVAL;
        }

      humi.timestamp = timestamp;
      humi.humidity  = (float)humidity / HTS221_HUMIDITY_PRECISION;
      memcpy(buffer, &humi, sizeof(humi));
      return sizeof(humi);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hts221_register_uorb
 *
 * Description:
 *   Register the HTS221 humidity and temperature sensor as UORB devices.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path.
 *   i2c   - The I2C bus driver instance.
 *   addr  - The I2C address of the HTS221.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hts221_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr)
{
  FAR struct hts221_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct hts221_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate HTS221 instance\n");
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->addr     = addr;
  priv->interval = 1000000;

  ret = hts221_configure(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure HTS221: %d\n", ret);
      goto errout;
    }

  ret = hts221_load_calibration(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to calibrate HTS221: %d\n", ret);
      goto errout;
    }

  /* Register the temperature lower-half */

  priv->temp.dev        = priv;
  priv->temp.lower.ops  = &g_hts221_ops;
  priv->temp.lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;

  ret = sensor_register(&priv->temp.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register HTS221 temperature: %d\n", ret);
      goto errout;
    }

  /* Register the humidity lower-half */

  priv->hum.dev        = priv;
  priv->hum.lower.ops  = &g_hts221_ops;
  priv->hum.lower.type = SENSOR_TYPE_RELATIVE_HUMIDITY;

  ret = sensor_register(&priv->hum.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register HTS221 humidity: %d\n", ret);
      sensor_unregister(&priv->temp.lower, devno);
      goto errout;
    }

  sninfo("HTS221 registered\n");
  return OK;

errout:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_HTS221 */
