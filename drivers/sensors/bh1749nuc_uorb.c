/****************************************************************************
 * drivers/sensors/bh1749nuc_uorb.c
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

#include <nuttx/sensors/sensor.h>

#include "bh1749nuc_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

enum bh1749nuc_idx_e
{
  BH1749NUC_RGB_IDX = 0,
  BH1749NUC_IR_IDX,
  BH1749NUC_MAX_IDX
};

struct bh1749nuc_sensor_dev_s;
struct bh1749nuc_sensor_s
{
  struct sensor_lowerhalf_s          lower;
  float                              scale;
  FAR struct bh1749nuc_sensor_dev_s *dev;
};

/* Used by the driver to manage the device */

struct bh1749nuc_sensor_dev_s
{
  struct bh1749nuc_sensor_s priv[BH1749NUC_MAX_IDX];
  struct bh1749nuc_dev_s    dev;
  bool                      activated;
  mutex_t                   lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bh1749nuc_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int bh1749nuc_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bh1749nuc_sensor_ops =
{
  NULL,                /* open */
  NULL,                /* close */
  bh1749nuc_activate,  /* activate */
  NULL,                /* set_interval */
  NULL,                /* batch */
  bh1749nuc_fetch,     /* fetch */
  NULL,                /* selftest */
  NULL,                /* set_calibvalue */
  NULL,                /* calibrate */
  NULL                 /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_activate
 ****************************************************************************/

static int bh1749nuc_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR struct bh1749nuc_sensor_s *priv = NULL;
  FAR struct bh1749nuc_dev_s    *dev = NULL;
  uint8_t                        val  = 0;

  /* Get dev */

  priv = (FAR struct bh1749nuc_sensor_s *)lower;
  dev = &priv->dev->dev;

  nxmutex_lock(&priv->dev->lock);

  if (enable)
    {
      /* MODE_CONTROL1 */

      val = (BH1749NUC_MODE_CONTROL1_MEAS_TIME160MS |
             BH1749NUC_MODE_CONTROL1_IR_GAIN_X1 |
             BH1749NUC_MODE_CONTROL1_RGB_GAIN_X1);
      bh1749nuc_putreg8(dev, BH1749NUC_MODE_CONTROL1, val);

      /* MODE_CONTROL2 */

      val = BH1749NUC_MODE_CONTROL2_RGBI_EN;
      bh1749nuc_putreg8(dev, BH1749NUC_MODE_CONTROL2, val);

      priv->dev->activated = true;
    }
  else
    {
      /* Stop sampling */

      val = (BH1749NUC_SYSTEM_CONTROL_SW_RESET |
             BH1749NUC_SYSTEM_CONTROL_INT_RESET);
      bh1749nuc_putreg8(dev, BH1749NUC_SYSTEM_CONTROL, val);

      priv->dev->activated = false;
    }

  nxmutex_unlock(&priv->dev->lock);

  return OK;
}

/****************************************************************************
 * Name: bh1749nuc_fetch
 ****************************************************************************/

static int bh1749nuc_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR char *buffer, size_t buflen)
{
  FAR struct bh1749nuc_sensor_s *priv = NULL;
  FAR struct bh1749nuc_dev_s    *dev  = NULL;
  struct sensor_rgb              rgb_data;
  struct sensor_ir               ir_data;
  uint64_t                       now  = sensor_get_timestamp();
  int                            ret  = 0;

  /* Get dev */

  priv = (FAR struct bh1749nuc_sensor_s *)lower;
  dev = &priv->dev->dev;

  nxmutex_lock(&priv->dev->lock);

  if (!priv->dev->activated)
    {
      ret = -EACCES;
      goto errout;
    }

  if (lower->type == SENSOR_TYPE_RGB)
    {
      if (buflen != sizeof(rgb_data))
        {
          ret = -EINVAL;
          goto errout;
        }

      rgb_data.timestamp = now;
      rgb_data.r         = bh1749nuc_read16(dev, BH1749NUC_RED_DATA_LSB);
      rgb_data.g         = bh1749nuc_read16(dev, BH1749NUC_GREEN_DATA_LSB);
      rgb_data.b         = bh1749nuc_read16(dev, BH1749NUC_BLUE_DATA_LSB);

      memcpy(buffer, &rgb_data, sizeof(rgb_data));

      ret = sizeof(ir_data);
    }
  else
    {
      if (buflen != sizeof(ir_data))
        {
          ret = -EINVAL;
          goto errout;
        }

      ir_data.timestamp = now;
      ir_data.ir        = bh1749nuc_read16(dev, BH1749NUC_IR_DATA_LSB);

      memcpy(buffer, &ir_data, sizeof(ir_data));
      ret = sizeof(ir_data);
    }

errout:
  nxmutex_unlock(&priv->dev->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_register_uorb
 *
 * Description:
 *   Register the BH1749NUC uorb device as 'devpath'
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   config  - device configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1749nuc_register_uorb(int devno, FAR struct bh1749nuc_config_s *config)
{
  FAR struct bh1749nuc_sensor_dev_s *dev = NULL;
  FAR struct bh1749nuc_sensor_s     *tmp = NULL;
  int                                ret = OK;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  dev = kmm_malloc(sizeof(struct bh1749nuc_sensor_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate bh1749nuc device instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(struct bh1749nuc_sensor_dev_s));
  nxmutex_init(&dev->lock);

  /* Configure dev */

  dev->dev.i2c  = config->i2c;
  dev->dev.addr = config->addr;
  dev->dev.freq = BH1749NUC_I2C_FREQ;

  /* Check Device ID */

  ret = bh1749nuc_checkid(&dev->dev);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /*  Register sensor */

  tmp             = &dev->priv[BH1749NUC_RGB_IDX];
  tmp->lower.type = SENSOR_TYPE_RGB;
  tmp->lower.ops  = &g_bh1749nuc_sensor_ops;
  tmp->dev        = dev;

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto rgb_err;
    }

  /*  Register sensor */

  tmp             = &dev->priv[BH1749NUC_IR_IDX];
  tmp->lower.type = SENSOR_TYPE_IR;
  tmp->lower.ops  = &g_bh1749nuc_sensor_ops;
  tmp->dev        = dev;

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto ir_err;
    }

ir_err:
  sensor_unregister(&dev->priv[BH1749NUC_RGB_IDX].lower, devno);
rgb_err:
  kmm_free(dev);
  return ret;
}
