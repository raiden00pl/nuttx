/****************************************************************************
 * drivers/sensors/fakesensor2_uorb.c
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

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/fakesensor2.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/gps.h>
#include <nuttx/signal.h>
#include <debug.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fakesensor2_s
{
  union
    {
      struct sensor_lowerhalf_s lower;
      struct gps_lowerhalf_s gps;
    };

  int type;
  struct file data;
  unsigned long interval;
  unsigned long batch;
  int raw_start;
  FAR const struct fakesensor2_data_s *samples;
  sem_t wakeup;
  volatile bool running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fakesensor2_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable);
static int fakesensor2_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR unsigned long *period_us);
static int fakesensor2_batch(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR unsigned long *latency_us);
static int fakegps_activate(FAR struct gps_lowerhalf_s *lower,
                            FAR struct file *filep, bool sw);
static int fakegps_set_interval(FAR struct gps_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us);
static void fakesensor2_push_event(FAR struct fakesensor2_s *sensor,
                                  uint64_t event_timestamp);
static int fakesensor2_thread(int argc, char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sensor_ops_s g_fakesensor2_ops =
{
  .activate = fakesensor2_activate,
  .set_interval = fakesensor2_set_interval,
  .batch = fakesensor2_batch,
};

static struct gps_ops_s g_fakegps_ops =
{
  .activate = fakegps_activate,
  .set_interval = fakegps_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void fakesensor2_read_accel(FAR struct fakesensor2_s *sensor,
                                          uint64_t event_timestamp)
{
  struct sensor_accel accel;

  accel.x = sensor->samples->data[sensor->raw_start++];
  accel.y = sensor->samples->data[sensor->raw_start++];
  accel.z = sensor->samples->data[sensor->raw_start++];

  accel.temperature = NAN;
  accel.timestamp = event_timestamp;
  sensor->lower.push_event(sensor->lower.priv, &accel,
                    sizeof(struct sensor_accel));
}

static inline void fakesensor2_read_mag(FAR struct fakesensor2_s *sensor,
                                        uint64_t event_timestamp)
{
  struct sensor_mag mag;

  mag.x = sensor->samples->data[sensor->raw_start++];
  mag.y = sensor->samples->data[sensor->raw_start++];
  mag.z = sensor->samples->data[sensor->raw_start++];

  mag.temperature = NAN;
  mag.timestamp = event_timestamp;
  sensor->lower.push_event(sensor->lower.priv, &mag,
                           sizeof(struct sensor_mag));
}

static inline void fakesensor2_read_gyro(FAR struct fakesensor2_s *sensor,
                                         uint64_t event_timestamp)
{
  struct sensor_gyro gyro;

  gyro.x = sensor->samples->data[sensor->raw_start++];
  gyro.y = sensor->samples->data[sensor->raw_start++];
  gyro.z = sensor->samples->data[sensor->raw_start++];

  gyro.temperature = NAN;
  gyro.timestamp = event_timestamp;
  sensor->lower.push_event(sensor->lower.priv, &gyro,
                    sizeof(struct sensor_gyro));
}

static inline void fakesensor2_read_gps(FAR struct fakesensor2_s *sensor)
{
  #warning TODO
  ASSERT(0);
}

static int fakesensor2_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable)
{
  FAR struct fakesensor2_s *sensor = container_of(lower,
                                                 struct fakesensor2_s, lower);
  if (enable)
    {
      sensor->running = true;

      /* Wake up the thread */

      nxsem_post(&sensor->wakeup);
    }
  else
    {
      sensor->running = false;
    }

  return OK;
}

static int fakegps_activate(FAR struct gps_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  return fakesensor2_activate((FAR void *)lower, filep, enable);
}

static int fakesensor2_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR unsigned long *period_us)
{
  FAR struct fakesensor2_s *sensor = container_of(lower,
                                                 struct fakesensor2_s, lower);
  sensor->interval = *period_us;
  return OK;
}

static int fakegps_set_interval(FAR struct gps_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us)
{
  return fakesensor2_set_interval((FAR void *)lower, filep, period_us);
}

static int fakesensor2_batch(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR unsigned long *latency_us)
{
  FAR struct fakesensor2_s *sensor = container_of(lower,
                                                 struct fakesensor2_s, lower);
  unsigned long max_latency = sensor->lower.nbuffer * sensor->interval;
  if (*latency_us > max_latency)
    {
      *latency_us = max_latency;
    }
  else if (*latency_us < sensor->interval && *latency_us > 0)
    {
      *latency_us = sensor->interval;
    }

  sensor->batch = *latency_us;
  return OK;
}

void fakesensor2_push_event(FAR struct fakesensor2_s *sensor,
                           uint64_t event_timestamp)
{
  switch (sensor->type)
  {
    case SENSOR_TYPE_ACCELEROMETER:
      fakesensor2_read_accel(sensor, event_timestamp);
      break;

    case SENSOR_TYPE_MAGNETIC_FIELD:
      fakesensor2_read_mag(sensor, event_timestamp);
      break;

    case SENSOR_TYPE_GYROSCOPE:
      fakesensor2_read_gyro(sensor, event_timestamp);
      break;

    case SENSOR_TYPE_GPS:
    case SENSOR_TYPE_GPS_SATELLITE:
      fakesensor2_read_gps(sensor);
      break;

    default:
      snerr("fakesensor: unsupported type sensor type\n");
      break;
  }
}

static int fakesensor2_thread(int argc, char** argv)
{
  FAR struct fakesensor2_s *sensor = (FAR struct fakesensor2_s *)
    ((uintptr_t)strtoul(argv[1], NULL, 16));

  /* Waiting to be woken up */

  nxsem_wait_uninterruptible(&sensor->wakeup);

  sensor->interval = sensor->samples->interval;

  while (sensor->running)
    {
      /* Sleeping thread for interval */

      nxsig_usleep(sensor->batch ? sensor->batch : sensor->interval);

      /* Notify upper */

      if (sensor->batch)
        {
          uint32_t batch_num = sensor->batch / sensor->interval;
          uint64_t event_timestamp =
            sensor_get_timestamp() - sensor->interval * batch_num;
          int i;

          for (i = 0; i < batch_num; i++)
            {
              fakesensor2_push_event(sensor, event_timestamp);
              event_timestamp += sensor->interval;
            }
        }
      else
        {
          fakesensor2_push_event(sensor, sensor_get_timestamp());
        }

      if (sensor->raw_start >= sensor->samples->dlen)
        {
          sensor->raw_start = 0;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fakesensor2_init
 *
 * Description:
 *   This function generates a sensor node under /dev/uorb/. And then
 *   report the data from provided data structure.
 *
 * Input Parameters:
 *   type        - The type of sensor and defined in <nuttx/sensors/sensor.h>
 *   data        - Sensor data provided as structure
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch
 *
 ****************************************************************************/

int fakesensor2_init(int type, FAR const struct fakesensor2_data_s *samples,
                     int devno, uint32_t batch_number)
{
  FAR struct fakesensor2_s *sensor;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  /* Alloc memory for sensor */

  sensor = kmm_zalloc(sizeof(struct fakesensor2_s));
  if (!sensor)
    {
      snerr("Memory cannot be allocated for fakesensor\n");
      return -ENOMEM;
    }

  sensor->samples = samples;
  sensor->type = type;

  nxsem_init(&sensor->wakeup, 0, 0);

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", sensor);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("fakesensor2_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       fakesensor2_thread, argv);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ERROR;
    }

  /*  Register sensor */

  if (type == SENSOR_TYPE_GPS || type == SENSOR_TYPE_GPS_SATELLITE)
    {
      sensor->gps.ops = &g_fakegps_ops;
      gps_register(&sensor->gps, devno, batch_number);
    }
  else
    {
      sensor->lower.type = type;
      sensor->lower.ops = &g_fakesensor2_ops;
      sensor->lower.nbuffer = batch_number;
      sensor_register(&sensor->lower, devno);
    }

  return OK;
}
