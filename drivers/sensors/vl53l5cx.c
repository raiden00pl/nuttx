/****************************************************************************
 * drivers/sensors/vl53l5cx.c
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

/* ST VL53L5CX 8x8 multizone time-of-flight sensor as a V4L2 depth camera.
 *
 * The driver provides both the image sensor and the image data interface
 * and registers its own capture device.  Each frame is V4L2_PIX_FMT_Z16,
 * 8x8 or 4x4 zones in the sensor's zone order, one little-endian distance
 * in millimetres per zone, 0 when the zone has no valid target.
 *
 * The sensor firmware, its default configuration, the default crosstalk
 * record and the NVM read request are loaded from files at the first
 * stream start.  The sensor ranges continuously with one target per zone
 * and the data-ready state is polled over I2C on the low priority work
 * queue.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/vl53l5cx.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Data bytes per I2C transfer.  Blocks are split into chunks addressed at
 * block start + offset; 128 stays below the 255 bytes that I2C controllers
 * move without reloading the byte count.
 */

#define VL53L5CX_CHUNK              128

/* Page register and pages */

#define VL53L5CX_PAGE_REG           0x7fff
#define VL53L5CX_PAGE_BOOT          0x00
#define VL53L5CX_PAGE_STATUS        0x01
#define VL53L5CX_PAGE_MAIN          0x02
#define VL53L5CX_PAGE_FW            0x09    /* 0x09, 0x0a and 0x0b */

/* Page 0 registers */

#define VL53L5CX_ID_REG             0x0000
#define VL53L5CX_REV_REG            0x0001
#define VL53L5CX_MCU_REG            0x0006
#define VL53L5CX_MCU_ERR_REG        0x0007
#define VL53L5CX_POWER_REG          0x0009
#define VL53L5CX_STOP_CMD_REG       0x0014
#define VL53L5CX_STOP_ARG_REG       0x0015

#define VL53L5CX_ID                 0xf0
#define VL53L5CX_REV                0x02

#define VL53L5CX_MCU_READY          0x01    /* Booted, awake */
#define VL53L5CX_MCU_ALL            0xff
#define VL53L5CX_MCU_STOPPED        0x80    /* Error code or stop status */
#define VL53L5CX_MCU_STOP_OK1       0x84
#define VL53L5CX_MCU_STOP_OK2       0x85

#define VL53L5CX_POWER_SLEEP        0x02
#define VL53L5CX_POWER_WAKE         0x04
#define VL53L5CX_POWER_RANGING      0x05    /* Firmware controls the sensor */

#define VL53L5CX_STOP_CMD           0x01
#define VL53L5CX_STOP_ARG           0x16

/* Page 1 registers */

#define VL53L5CX_FW_REG             0x0021
#define VL53L5CX_FW_READY           0x10

/* Page 2: result buffer and command window */

#define VL53L5CX_RESULT_REG         0x0000
#define VL53L5CX_UI_STATUS_REG      0x2c00
#define VL53L5CX_UI_DATA_REG        0x2c04
#define VL53L5CX_UI_END             0x3000
#define VL53L5CX_DCI_READ_REG       0x2ff4
#define VL53L5CX_CMD_REG            0x2ffc

/* UI status word bytes */

#define VL53L5CX_UI_S0_NVM_DONE     0x02
#define VL53L5CX_UI_S1_DONE         0x03
#define VL53L5CX_UI_S2_ERROR        0x7f

/* Run time files */

#define VL53L5CX_FW_SIZE            86016
#define VL53L5CX_FW_PAGE_SIZE       32768   /* Last page holds the rest */
#define VL53L5CX_CFG_SIZE           972
#define VL53L5CX_XTALK_SIZE         776
#define VL53L5CX_NVMREQ_SIZE        40

/* Byte 107 of the configuration is the internal target count: 2 for one
 * target per zone.
 */

#define VL53L5CX_CFG_NBTAR          107
#define VL53L5CX_FW_NBTAR           2

/* Offset (NVM) record */

#define VL53L5CX_NVM_SIZE           492
#define VL53L5CX_OFS_SIZE           488
#define VL53L5CX_OFS_DROP           8       /* Dropped at upload */
#define VL53L5CX_OFS_PATCH          0x10
#define VL53L5CX_OFS_SIGNAL         0x3c
#define VL53L5CX_OFS_RANGE          0x140

/* Crosstalk record */

#define VL53L5CX_XT_PATCH1          0x08
#define VL53L5CX_XT_PATCH2          0x20
#define VL53L5CX_XT_PATCH3          0x134
#define VL53L5CX_XT_PATCH4          0x78
#define VL53L5CX_XT_SIGNAL          0x34

/* Polling: 10 ms after each read, 200 or 500 polls */

#define VL53L5CX_POLL_US            10000
#define VL53L5CX_POLL_TRIES         200
#define VL53L5CX_STOP_TRIES         500
#define VL53L5CX_MCU_BOOT_US        1000
#define VL53L5CX_MCU_BOOT_TRIES     500

/* DCI items (index, length) */

#define VL53L5CX_DCI_MAX            48
#define VL53L5CX_DCI_HDR            4
#define VL53L5CX_DCI_FOOTER         8
#define VL53L5CX_DCI_FRAME          (VL53L5CX_DCI_HDR + VL53L5CX_DCI_FOOTER)

#define VL53L5CX_DCI_OUTPUT_CHECK   0x5440  /* 12 */
#define VL53L5CX_DCI_ZONE_CFG       0x5450  /* 8 */
#define VL53L5CX_DCI_FREQ           0x5458  /* 4 */
#define VL53L5CX_DCI_MODE           0xad30  /* 8 */
#define VL53L5CX_DCI_DSS            0xad38  /* 16 */
#define VL53L5CX_DCI_SINGLE_RANGE   0xd964  /* 4 */
#define VL53L5CX_DCI_OUTPUT_CFG     0xd968  /* 8 */
#define VL53L5CX_DCI_OUTPUT_MASK    0xd970  /* 16 */
#define VL53L5CX_DCI_OUTPUT_LIST    0xd980  /* 48 */
#define VL53L5CX_DCI_PIPE_CTRL      0xdb80  /* 4 */
#define VL53L5CX_DCI_GLARE          0xe108  /* 40 */

#define VL53L5CX_ZONE_CFG_LEN       8
#define VL53L5CX_FREQ_LEN           4
#define VL53L5CX_FREQ_HZ            1       /* N-offset */
#define VL53L5CX_MODE_LEN           8
#define VL53L5CX_DSS_LEN            16
#define VL53L5CX_GLARE_LEN          40
#define VL53L5CX_GLARE_EN1          0x26
#define VL53L5CX_GLARE_EN2          0x25
#define VL53L5CX_OUTPUT_CHECK_LEN   12
#define VL53L5CX_OUTPUT_CHECK_SIZE  8

/* Output list: 12 block headers, index << 16 | size << 4 | type */

#define VL53L5CX_NOUTPUTS           12
#define VL53L5CX_HDR_INDEX(h)       ((uint16_t)((h) >> 16))
#define VL53L5CX_HDR_SIZE(h)        (((h) >> 4) & 0xfff)
#define VL53L5CX_HDR_TYPE(h)        ((h) & 0xf)
#define VL53L5CX_HDR_SIZE_MASK      0x0000fff0
#define VL53L5CX_HDR_SIZE_SHIFT     4
#define VL53L5CX_TYPE_ELEM_MAX      12      /* Types 1-12: bytes/element */

#define VL53L5CX_OUT_TARGETS        5
#define VL53L5CX_OUT_DISTANCE       8
#define VL53L5CX_OUT_STATUS         10
#define VL53L5CX_OUT_MANDATORY      0x00000007
#define VL53L5CX_OUT_MASK3          0xc0000000
#define VL53L5CX_OUT_COUNT          13      /* List length + 1 */

#define VL53L5CX_BLK_TARGETS        0xdb84
#define VL53L5CX_BLK_DISTANCE       0xdf44
#define VL53L5CX_BLK_STATUS         0xe084

/* Result frame */

#define VL53L5CX_FRAME_OVERHEAD     24
#define VL53L5CX_FRAME_ID           8       /* Header ID, N-offset */
#define VL53L5CX_FRAME_FOOTER       4       /* Footer ID, from the end */
#define VL53L5CX_FRAME_BLOCKS       16      /* First block header */
#define VL53L5CX_STREAM_NONE        0xff

/* Data-ready word at 0x0000 */

#define VL53L5CX_READY_B1           0x05
#define VL53L5CX_READY_B2           0x05
#define VL53L5CX_READY_B3           0x10
#define VL53L5CX_READY_ERROR        0x80

/* Target status codes */

#define VL53L5CX_STATUS_VALID       5
#define VL53L5CX_STATUS_VALID_LARGE 9

#define VL53L5CX_DISTANCE_SHIFT     2       /* 1/4 mm */

/* Zones and rates */

#define VL53L5CX_WIDTH_8X8          8
#define VL53L5CX_WIDTH_4X4          4
#define VL53L5CX_ZONES_MAX          64
#define VL53L5CX_FPS_8X8            15
#define VL53L5CX_FPS_4X4            60
#define VL53L5CX_FPS_MIN            1
#define VL53L5CX_BYTES_PER_PIXEL    2

/* Data-ready polling */

#define VL53L5CX_READY_MS           5

/* Scratch buffer for the records and the result frames */

#define VL53L5CX_SCRATCH            VL53L5CX_XTALK_SIZE

/* Register sequence operations */

#define VL53L5CX_OP_WRITE           0
#define VL53L5CX_OP_READ            1       /* Dummy read */
#define VL53L5CX_OP_DELAY           2       /* val in ms */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vl53l5cx_op_s
{
  uint16_t reg;
  uint8_t  val;
  uint8_t  op;
};

struct vl53l5cx_dev_s
{
  struct imgsensor_s sensor;          /* Image sensor interface */
  struct imgdata_s data;              /* Image data interface */
  FAR struct i2c_master_s *i2c;       /* I2C bus */
  uint8_t addr;                       /* 7-bit I2C address */
  mutex_t lock;                       /* Serializes the sensor access */
  spinlock_t buflock;                 /* Buffer and callback hand-off */
  struct work_s work;                 /* Data-ready polling */

  /* Device state, under lock */

  bool booted;                        /* Firmware and configuration up */
  bool asleep;                        /* Low power mode */
  bool ranging;                       /* Ranging started */
  uint8_t devzones;                   /* Zones configured in the sensor */
  uint8_t zones;                      /* Zones of the running stream */
  uint8_t fps;                        /* Ranging frequency */
  uint16_t framesize;                 /* Result frame size */

  /* Worker state */

  uint8_t lastcount;                  /* Last stream counter */
  pid_t cbtid;                        /* Worker thread in the callback */

  /* Under buflock */

  bool polling;                       /* Worker requeues itself */
  FAR uint8_t *buf;                   /* Next frame buffer */
  uint32_t bufsize;
  imgdata_capture_t callback;
  FAR void *arg;

  /* Records and buffers */

  uint8_t ofs[VL53L5CX_OFS_SIZE];     /* Device offset record */
  uint8_t xtalk[VL53L5CX_XTALK_SIZE]; /* Crosstalk record */
  uint8_t scratch[VL53L5CX_SCRATCH];
  uint8_t dci[VL53L5CX_DCI_MAX + VL53L5CX_DCI_FRAME];
  uint8_t txbuf[2 + VL53L5CX_CHUNK];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Image sensor operations */

static bool vl53l5cx_is_available(FAR struct imgsensor_s *sensor);
static int vl53l5cx_init(FAR struct imgsensor_s *sensor);
static int vl53l5cx_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
vl53l5cx_get_driver_name(FAR struct imgsensor_s *sensor);
static int vl53l5cx_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                           imgsensor_stream_type_t type,
                                           uint8_t nr_datafmts,
                                           FAR imgsensor_format_t *datafmts,
                                           FAR imgsensor_interval_t *ival);
static int vl53l5cx_start_capture(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type,
                                  uint8_t nr_datafmts,
                                  FAR imgsensor_format_t *datafmts,
                                  FAR imgsensor_interval_t *interval);
static int vl53l5cx_stop_capture(FAR struct imgsensor_s *sensor,
                                 imgsensor_stream_type_t type);
static int vl53l5cx_get_frame_interval(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       FAR imgsensor_interval_t *interval);

/* Image data operations */

static int vl53l5cx_data_init(FAR struct imgdata_s *data);
static int vl53l5cx_data_uninit(FAR struct imgdata_s *data);
static int vl53l5cx_data_set_buf(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size);
static int vl53l5cx_data_validate(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmts,
                                  FAR imgdata_format_t *fmts,
                                  FAR imgdata_interval_t *ival);
static int vl53l5cx_data_start_capture(FAR struct imgdata_s *data,
                                       uint8_t nr_datafmts,
                                       FAR imgdata_format_t *datafmts,
                                       FAR imgdata_interval_t *interval,
                                       FAR imgdata_capture_t callback,
                                       FAR void *arg);
static int vl53l5cx_data_stop_capture(FAR struct imgdata_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgsensor_ops_s g_vl53l5cx_ops =
{
  .is_available           = vl53l5cx_is_available,
  .init                   = vl53l5cx_init,
  .uninit                 = vl53l5cx_uninit,
  .get_driver_name        = vl53l5cx_get_driver_name,
  .validate_frame_setting = vl53l5cx_validate_frame_setting,
  .start_capture          = vl53l5cx_start_capture,
  .stop_capture           = vl53l5cx_stop_capture,
  .get_frame_interval     = vl53l5cx_get_frame_interval,
};

static const struct imgdata_ops_s g_vl53l5cx_data_ops =
{
  .init                   = vl53l5cx_data_init,
  .uninit                 = vl53l5cx_data_uninit,
  .set_buf                = vl53l5cx_data_set_buf,
  .validate_frame_setting = vl53l5cx_data_validate,
  .start_capture          = vl53l5cx_data_start_capture,
  .stop_capture           = vl53l5cx_data_stop_capture,
};

static const struct v4l2_fmtdesc g_vl53l5cx_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_Z16,
    .description = "Depth 16-bit",
  },
};

/* 8x8 first: it is the default */

static const struct v4l2_frmsizeenum g_vl53l5cx_frmsizes[] =
{
  {
    .pixel_format = V4L2_PIX_FMT_Z16,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width    = VL53L5CX_WIDTH_8X8,
        .height   = VL53L5CX_WIDTH_8X8,
      },
  },
  {
    .pixel_format = V4L2_PIX_FMT_Z16,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width    = VL53L5CX_WIDTH_4X4,
        .height   = VL53L5CX_WIDTH_4X4,
      },
  },
};

/* Any whole number of frames per second in the range */

static const struct v4l2_frmivalenum g_vl53l5cx_frmintervals[] =
{
  {
    .pixel_format = V4L2_PIX_FMT_Z16,
    .width        = VL53L5CX_WIDTH_8X8,
    .height       = VL53L5CX_WIDTH_8X8,
    .type         = V4L2_FRMIVAL_TYPE_CONTINUOUS,
    .stepwise     =
      {
        .min      =
          {
            1, VL53L5CX_FPS_8X8
          },
        .max      =
          {
            1, VL53L5CX_FPS_MIN
          },
        .step     =
          {
            1, 1
          },
      },
  },
  {
    .pixel_format = V4L2_PIX_FMT_Z16,
    .width        = VL53L5CX_WIDTH_4X4,
    .height       = VL53L5CX_WIDTH_4X4,
    .type         = V4L2_FRMIVAL_TYPE_CONTINUOUS,
    .stepwise     =
      {
        .min      =
          {
            1, VL53L5CX_FPS_4X4
          },
        .max      =
          {
            1, VL53L5CX_FPS_MIN
          },
        .step     =
          {
            1, 1
          },
      },
  },
};

/* Software reboot (5.2 steps 1-10), up to the boot poll */

static const struct vl53l5cx_op_s g_vl53l5cx_reboot[] =
{
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
  { 0x0009, 0x04, VL53L5CX_OP_WRITE },
  { 0x000f, 0x40, VL53L5CX_OP_WRITE },
  { 0x000a, 0x03, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, 0, VL53L5CX_OP_READ },
  { 0x000c, 0x01, VL53L5CX_OP_WRITE },
  { 0x0101, 0x00, VL53L5CX_OP_WRITE },
  { 0x0102, 0x00, VL53L5CX_OP_WRITE },
  { 0x010a, 0x01, VL53L5CX_OP_WRITE },
  { 0x4002, 0x01, VL53L5CX_OP_WRITE },
  { 0x4002, 0x00, VL53L5CX_OP_WRITE },
  { 0x010a, 0x03, VL53L5CX_OP_WRITE },
  { 0x0103, 0x01, VL53L5CX_OP_WRITE },
  { 0x000c, 0x00, VL53L5CX_OP_WRITE },
  { 0x000f, 0x43, VL53L5CX_OP_WRITE },
  { 0, 1, VL53L5CX_OP_DELAY },
  { 0x000f, 0x40, VL53L5CX_OP_WRITE },
  { 0x000a, 0x01, VL53L5CX_OP_WRITE },
  { 0, 100, VL53L5CX_OP_DELAY },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
};

/* 5.2 steps 11-13, up to the firmware status poll */

static const struct vl53l5cx_op_s g_vl53l5cx_booted[] =
{
  { 0x000e, 0x01, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_MAIN, VL53L5CX_OP_WRITE },
  { 0x0003, 0x0d, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_STATUS, VL53L5CX_OP_WRITE },
};

/* 5.2 step 14 and 5.3 steps 1-6: power on and MCU wake */

static const struct vl53l5cx_op_s g_vl53l5cx_poweron[] =
{
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, 0, VL53L5CX_OP_READ },
  { 0x000c, 0x01, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
  { 0x0101, 0x00, VL53L5CX_OP_WRITE },
  { 0x0102, 0x00, VL53L5CX_OP_WRITE },
  { 0x010a, 0x01, VL53L5CX_OP_WRITE },
  { 0x4002, 0x01, VL53L5CX_OP_WRITE },
  { 0x4002, 0x00, VL53L5CX_OP_WRITE },
  { 0x010a, 0x03, VL53L5CX_OP_WRITE },
  { 0x0103, 0x01, VL53L5CX_OP_WRITE },
  { 0x400f, 0x00, VL53L5CX_OP_WRITE },
  { 0x021a, 0x43, VL53L5CX_OP_WRITE },
  { 0x021a, 0x03, VL53L5CX_OP_WRITE },
  { 0x021a, 0x01, VL53L5CX_OP_WRITE },
  { 0x021a, 0x00, VL53L5CX_OP_WRITE },
  { 0x0219, 0x00, VL53L5CX_OP_WRITE },
  { 0x021b, 0x00, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, 0, VL53L5CX_OP_READ },
  { 0x000c, 0x00, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_STATUS, VL53L5CX_OP_WRITE },
  { 0x0020, 0x07, VL53L5CX_OP_WRITE },
  { 0x0020, 0x06, VL53L5CX_OP_WRITE },
};

/* 5.3 steps 7 (end) and 8: download check, up to the status poll */

static const struct vl53l5cx_op_s g_vl53l5cx_fwcheck[] =
{
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_STATUS, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_MAIN, VL53L5CX_OP_WRITE },
  { 0x0003, 0x0d, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_STATUS, VL53L5CX_OP_WRITE },
};

/* 5.3 step 9 and 5.4 steps 1-3: MCU reset and boot */

static const struct vl53l5cx_op_s g_vl53l5cx_mcuboot[] =
{
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, 0, VL53L5CX_OP_READ },
  { 0x000c, 0x01, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, VL53L5CX_PAGE_BOOT, VL53L5CX_OP_WRITE },
  { 0x0114, 0x00, VL53L5CX_OP_WRITE },
  { 0x0115, 0x00, VL53L5CX_OP_WRITE },
  { 0x0116, 0x42, VL53L5CX_OP_WRITE },
  { 0x0117, 0x00, VL53L5CX_OP_WRITE },
  { 0x000b, 0x00, VL53L5CX_OP_WRITE },
  { VL53L5CX_PAGE_REG, 0, VL53L5CX_OP_READ },
  { 0x000c, 0x00, VL53L5CX_OP_WRITE },
  { 0x000b, 0x01, VL53L5CX_OP_WRITE },
};

/* Fixed bytes of the command frames (wire order) */

static const uint8_t g_vl53l5cx_dci_rd_footer[VL53L5CX_DCI_FOOTER] =
{
  0x00, 0x00, 0x00, 0x0f, 0x00, 0x02, 0x00, 0x08
};

static const uint8_t g_vl53l5cx_dci_wr_footer[VL53L5CX_DCI_FOOTER - 2] =
{
  0x00, 0x00, 0x00, 0x0f, 0x05, 0x01
};

static const uint8_t g_vl53l5cx_ofs_footer[VL53L5CX_OFS_DROP] =
{
  0x00, 0x00, 0x00, 0x0f, 0x03, 0x01, 0x01, 0xe4
};

static const uint8_t g_vl53l5cx_start_cmd[4] =
{
  0x00, 0x03, 0x00, 0x00
};

static const uint8_t g_vl53l5cx_stopped[4] =
{
  0xff, 0x04, 0x00, 0x00
};

/* 4x4 patches of the offset and the crosstalk records (5.8) */

static const uint8_t g_vl53l5cx_ofs_patch[8] =
{
  0x0f, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07
};

static const uint8_t g_vl53l5cx_xt_patch1[8] =
{
  0x0f, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07
};

static const uint8_t g_vl53l5cx_xt_patch2[8] =
{
  0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08
};

static const uint8_t g_vl53l5cx_xt_patch3[4] =
{
  0xa0, 0xfc, 0x01, 0x00
};

static const uint8_t g_vl53l5cx_xt_patch4[4] =
{
  0x00, 0x00, 0x00, 0x00
};

/* Output list for one target per zone (7.1) */

static const uint32_t g_vl53l5cx_outputs[VL53L5CX_NOUTPUTS] =
{
  0x0000000d,                         /* Frame start */
  0x54b400c0,                         /* Metadata */
  0x54c00040,                         /* Common data */
  0x54d00104,                         /* Ambient per SPAD */
  0x55d00404,                         /* SPADs enabled */
  0xdb840401,                         /* Targets detected */
  0xdbc40404,                         /* Signal per SPAD */
  0xdec40402,                         /* Range sigma */
  0xdf440402,                         /* Distance */
  0xe0440401,                         /* Reflectance */
  0xe0840401,                         /* Target status */
  0xd85808c0,                         /* Motion indicator */
};

static const uint32_t g_vl53l5cx_enabled = VL53L5CX_OUT_MANDATORY |
                                           (1 << VL53L5CX_OUT_TARGETS) |
                                           (1 << VL53L5CX_OUT_DISTANCE) |
                                           (1 << VL53L5CX_OUT_STATUS);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l5cx_getle16/32, vl53l5cx_putle32
 ****************************************************************************/

static uint16_t vl53l5cx_getle16(FAR const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t vl53l5cx_getle32(FAR const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void vl53l5cx_putle32(FAR uint8_t *p, uint32_t val)
{
  p[0] = val & 0xff;
  p[1] = (val >> 8) & 0xff;
  p[2] = (val >> 16) & 0xff;
  p[3] = (val >> 24) & 0xff;
}

/****************************************************************************
 * Name: vl53l5cx_swap
 *
 * Description:
 *   Reverse every 4-byte group: wire order <-> native (N-view) order.
 *
 ****************************************************************************/

static void vl53l5cx_swap(FAR uint8_t *buf, size_t len)
{
  uint8_t tmp;
  size_t i;

  for (i = 0; i + 4 <= len; i += 4)
    {
      tmp        = buf[i];
      buf[i]     = buf[i + 3];
      buf[i + 3] = tmp;
      tmp        = buf[i + 1];
      buf[i + 1] = buf[i + 2];
      buf[i + 2] = tmp;
    }
}

/****************************************************************************
 * Name: vl53l5cx_xfer
 *
 * Description:
 *   One write transaction of txbuf (address and len data bytes), or a read
 *   of len bytes at the address in txbuf.
 *
 ****************************************************************************/

static int vl53l5cx_xfer(FAR struct vl53l5cx_dev_s *priv, uint16_t reg,
                         FAR uint8_t *rxbuf, size_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  priv->txbuf[0] = reg >> 8;
  priv->txbuf[1] = reg & 0xff;

  msg[0].frequency = CONFIG_VL53L5CX_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = priv->txbuf;
  msg[0].length    = rxbuf != NULL ? 2 : 2 + len;

  msg[1].frequency = CONFIG_VL53L5CX_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = rxbuf;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, rxbuf != NULL ? 2 : 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C at 0x%04x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: vl53l5cx_write
 ****************************************************************************/

static int vl53l5cx_write(FAR struct vl53l5cx_dev_s *priv, uint16_t reg,
                          FAR const uint8_t *data, size_t len)
{
  size_t off;
  size_t n;
  int ret;

  for (off = 0; off < len; off += n)
    {
      n = MIN(len - off, VL53L5CX_CHUNK);
      memcpy(&priv->txbuf[2], &data[off], n);
      ret = vl53l5cx_xfer(priv, reg + off, NULL, n);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_read
 ****************************************************************************/

static int vl53l5cx_read(FAR struct vl53l5cx_dev_s *priv, uint16_t reg,
                         FAR uint8_t *data, size_t len)
{
  size_t off;
  size_t n;
  int ret;

  for (off = 0; off < len; off += n)
    {
      n = MIN(len - off, VL53L5CX_CHUNK);
      ret = vl53l5cx_xfer(priv, reg + off, &data[off], n);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_putreg/getreg/page
 ****************************************************************************/

static int vl53l5cx_putreg(FAR struct vl53l5cx_dev_s *priv, uint16_t reg,
                           uint8_t val)
{
  return vl53l5cx_write(priv, reg, &val, 1);
}

static int vl53l5cx_getreg(FAR struct vl53l5cx_dev_s *priv, uint16_t reg,
                           FAR uint8_t *val)
{
  return vl53l5cx_read(priv, reg, val, 1);
}

static int vl53l5cx_page(FAR struct vl53l5cx_dev_s *priv, uint8_t page)
{
  return vl53l5cx_putreg(priv, VL53L5CX_PAGE_REG, page);
}

/****************************************************************************
 * Name: vl53l5cx_runseq
 ****************************************************************************/

static int vl53l5cx_runseq(FAR struct vl53l5cx_dev_s *priv,
                           FAR const struct vl53l5cx_op_s *ops, size_t n)
{
  uint8_t val;
  size_t i;
  int ret = OK;

  for (i = 0; i < n && ret >= 0; i++)
    {
      switch (ops[i].op)
        {
          case VL53L5CX_OP_WRITE:
            ret = vl53l5cx_putreg(priv, ops[i].reg, ops[i].val);
            break;

          case VL53L5CX_OP_READ:
            ret = vl53l5cx_getreg(priv, ops[i].reg, &val);
            break;

          default:
            nxsig_usleep(ops[i].val * USEC_PER_MSEC);
            break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: vl53l5cx_pollreg
 *
 * Description:
 *   Poll one byte of the current page until (byte & mask) == val.
 *
 ****************************************************************************/

static int vl53l5cx_pollreg(FAR struct vl53l5cx_dev_s *priv, uint16_t reg,
                            uint8_t mask, uint8_t val, int tries)
{
  uint8_t b;
  int ret;

  while (tries-- > 0)
    {
      ret = vl53l5cx_getreg(priv, reg, &b);
      if (ret < 0)
        {
          return ret;
        }

      nxsig_usleep(VL53L5CX_POLL_US);
      if ((b & mask) == val)
        {
          return OK;
        }
    }

  snerr("ERROR: 0x%04x is 0x%02x\n", reg, b);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: vl53l5cx_pollui
 *
 * Description:
 *   Poll the UI status word until a command has completed (section 3).
 *
 ****************************************************************************/

static int vl53l5cx_pollui(FAR struct vl53l5cx_dev_s *priv, bool nvm)
{
  uint8_t s[4];
  int tries;
  int ret;

  for (tries = 0; tries < VL53L5CX_POLL_TRIES; tries++)
    {
      ret = vl53l5cx_read(priv, VL53L5CX_UI_STATUS_REG, s, sizeof(s));
      if (ret < 0)
        {
          return ret;
        }

      nxsig_usleep(VL53L5CX_POLL_US);
      if (s[2] >= VL53L5CX_UI_S2_ERROR)
        {
          snerr("ERROR: firmware status 0x%02x\n", s[2]);
          return -EIO;
        }

      if (nvm ? s[0] == VL53L5CX_UI_S0_NVM_DONE :
                s[1] == VL53L5CX_UI_S1_DONE)
        {
          return OK;
        }
    }

  snerr("ERROR: command timeout\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: vl53l5cx_dci_header
 ****************************************************************************/

static void vl53l5cx_dci_header(FAR uint8_t *hdr, uint16_t index,
                                size_t len)
{
  hdr[0] = index >> 8;
  hdr[1] = index & 0xff;
  hdr[2] = (len >> 4) & 0xff;
  hdr[3] = (len & 0x0f) << 4;
}

/****************************************************************************
 * Name: vl53l5cx_dci_write
 *
 * Description:
 *   Write a DCI item, data in the N-view (4.2).
 *
 ****************************************************************************/

static int vl53l5cx_dci_write(FAR struct vl53l5cx_dev_s *priv,
                              uint16_t index, FAR const uint8_t *data,
                              size_t len)
{
  FAR uint8_t *frame = priv->dci;
  size_t flen = len + VL53L5CX_DCI_FRAME;
  int ret;

  DEBUGASSERT(len <= VL53L5CX_DCI_MAX && (len & 3) == 0);

  vl53l5cx_dci_header(frame, index, len);
  memcpy(&frame[VL53L5CX_DCI_HDR], data, len);
  vl53l5cx_swap(&frame[VL53L5CX_DCI_HDR], len);
  memcpy(&frame[VL53L5CX_DCI_HDR + len], g_vl53l5cx_dci_wr_footer,
         sizeof(g_vl53l5cx_dci_wr_footer));
  frame[flen - 2] = ((len + VL53L5CX_DCI_FOOTER) >> 8) & 0xff;
  frame[flen - 1] = (len + VL53L5CX_DCI_FOOTER) & 0xff;

  ret = vl53l5cx_write(priv, VL53L5CX_UI_END - flen, frame, flen);
  if (ret < 0)
    {
      return ret;
    }

  return vl53l5cx_pollui(priv, false);
}

/****************************************************************************
 * Name: vl53l5cx_dci_read
 *
 * Description:
 *   Read a DCI item into data, in the N-view (4.3).
 *
 ****************************************************************************/

static int vl53l5cx_dci_read(FAR struct vl53l5cx_dev_s *priv,
                             uint16_t index, FAR uint8_t *data, size_t len)
{
  FAR uint8_t *frame = priv->dci;
  int ret;

  DEBUGASSERT(len <= VL53L5CX_DCI_MAX && (len & 3) == 0);

  vl53l5cx_dci_header(frame, index, len);
  memcpy(&frame[VL53L5CX_DCI_HDR], g_vl53l5cx_dci_rd_footer,
         sizeof(g_vl53l5cx_dci_rd_footer));

  ret = vl53l5cx_write(priv, VL53L5CX_DCI_READ_REG, frame,
                       VL53L5CX_DCI_FRAME);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_pollui(priv, false);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_read(priv, VL53L5CX_UI_DATA_REG, frame,
                      len + VL53L5CX_DCI_FRAME);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(data, &frame[VL53L5CX_DCI_HDR], len);
  vl53l5cx_swap(data, len);
  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_dci_patch
 *
 * Description:
 *   Read-modify-write the bytes of a DCI item at the N-offsets in off[].
 *
 ****************************************************************************/

static int vl53l5cx_dci_patch(FAR struct vl53l5cx_dev_s *priv,
                              uint16_t index, size_t len,
                              FAR const uint8_t *off,
                              FAR const uint8_t *val, size_t n)
{
  uint8_t item[VL53L5CX_DCI_MAX];
  size_t i;
  int ret;

  ret = vl53l5cx_dci_read(priv, index, item, len);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < n; i++)
    {
      DEBUGASSERT(off[i] < len);
      item[off[i]] = val[i];
    }

  return vl53l5cx_dci_write(priv, index, item, len);
}

/****************************************************************************
 * Name: vl53l5cx_dci_put32
 ****************************************************************************/

static int vl53l5cx_dci_put32(FAR struct vl53l5cx_dev_s *priv,
                              uint16_t index, uint32_t val)
{
  uint8_t item[4];

  vl53l5cx_putle32(item, val);
  return vl53l5cx_dci_write(priv, index, item, sizeof(item));
}

/****************************************************************************
 * Name: vl53l5cx_downsample
 *
 * Description:
 *   Average an 8x8 grid of 32-bit unsigned or 16-bit signed N-view values
 *   to 4x4 in elements 0-15, zero elements 16-63 (5.8.1).
 *
 ****************************************************************************/

static void vl53l5cx_downsample(FAR uint8_t *grid, bool range)
{
  int32_t out[VL53L5CX_ZONES_MAX / 4];
  size_t esize = range ? sizeof(int16_t) : sizeof(uint32_t);
  int64_t sum;
  int r;
  int c;
  int i;
  int k;

  for (r = 0; r < VL53L5CX_WIDTH_4X4; r++)
    {
      for (c = 0; c < VL53L5CX_WIDTH_4X4; c++)
        {
          sum = 0;
          for (i = 0; i < 4; i++)
            {
              k = (2 * r + i / 2) * VL53L5CX_WIDTH_8X8 + 2 * c + i % 2;
              if (range)
                {
                  sum += (int16_t)vl53l5cx_getle16(&grid[k * esize]);
                }
              else
                {
                  sum += vl53l5cx_getle32(&grid[k * esize]);
                }
            }

          out[r * VL53L5CX_WIDTH_4X4 + c] = (int32_t)(sum / 4);
        }
    }

  memset(grid, 0, VL53L5CX_ZONES_MAX * esize);
  for (i = 0; i < nitems(out); i++)
    {
      if (range)
        {
          grid[i * esize]     = out[i] & 0xff;
          grid[i * esize + 1] = (out[i] >> 8) & 0xff;
        }
      else
        {
          vl53l5cx_putle32(&grid[i * esize], (uint32_t)out[i]);
        }
    }
}

/****************************************************************************
 * Name: vl53l5cx_upload_offset
 ****************************************************************************/

static int vl53l5cx_upload_offset(FAR struct vl53l5cx_dev_s *priv,
                                  bool is4x4)
{
  FAR uint8_t *b = priv->scratch;
  size_t ulen = VL53L5CX_OFS_SIZE - VL53L5CX_OFS_DROP;
  int ret;

  memcpy(b, priv->ofs, VL53L5CX_OFS_SIZE);
  if (is4x4)
    {
      memcpy(&b[VL53L5CX_OFS_PATCH], g_vl53l5cx_ofs_patch,
             sizeof(g_vl53l5cx_ofs_patch));
      vl53l5cx_swap(b, VL53L5CX_OFS_SIZE);
      vl53l5cx_downsample(&b[VL53L5CX_OFS_SIGNAL], false);
      vl53l5cx_downsample(&b[VL53L5CX_OFS_RANGE], true);
      vl53l5cx_swap(b, VL53L5CX_OFS_SIZE);
    }

  memmove(b, &b[VL53L5CX_OFS_DROP], ulen);
  memcpy(&b[ulen], g_vl53l5cx_ofs_footer, sizeof(g_vl53l5cx_ofs_footer));

  ret = vl53l5cx_write(priv, VL53L5CX_UI_END - VL53L5CX_OFS_SIZE, b,
                       VL53L5CX_OFS_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  return vl53l5cx_pollui(priv, false);
}

/****************************************************************************
 * Name: vl53l5cx_upload_xtalk
 ****************************************************************************/

static int vl53l5cx_upload_xtalk(FAR struct vl53l5cx_dev_s *priv,
                                 bool is4x4)
{
  FAR uint8_t *x = priv->scratch;
  int ret;

  memcpy(x, priv->xtalk, VL53L5CX_XTALK_SIZE);
  if (is4x4)
    {
      memcpy(&x[VL53L5CX_XT_PATCH1], g_vl53l5cx_xt_patch1,
             sizeof(g_vl53l5cx_xt_patch1));
      memcpy(&x[VL53L5CX_XT_PATCH2], g_vl53l5cx_xt_patch2,
             sizeof(g_vl53l5cx_xt_patch2));
      vl53l5cx_swap(x, VL53L5CX_XTALK_SIZE);
      vl53l5cx_downsample(&x[VL53L5CX_XT_SIGNAL], false);
      vl53l5cx_swap(x, VL53L5CX_XTALK_SIZE);
      memcpy(&x[VL53L5CX_XT_PATCH3], g_vl53l5cx_xt_patch3,
             sizeof(g_vl53l5cx_xt_patch3));
      memcpy(&x[VL53L5CX_XT_PATCH4], g_vl53l5cx_xt_patch4,
             sizeof(g_vl53l5cx_xt_patch4));
    }

  ret = vl53l5cx_write(priv, VL53L5CX_UI_END - VL53L5CX_XTALK_SIZE, x,
                       VL53L5CX_XTALK_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  return vl53l5cx_pollui(priv, false);
}

/****************************************************************************
 * Name: vl53l5cx_openfile
 *
 * Description:
 *   Open a run time file and check its size.
 *
 ****************************************************************************/

static int vl53l5cx_openfile(FAR struct file *filep, FAR const char *path,
                             off_t size)
{
  struct stat st;
  int ret;

  ret = file_open(filep, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      snerr("ERROR: cannot open %s: %d\n", path, ret);
      return ret;
    }

  ret = file_fstat(filep, &st);
  if (ret >= 0 && st.st_size != size)
    {
      snerr("ERROR: %s is %jd bytes, not %jd\n", path,
            (intmax_t)st.st_size, (intmax_t)size);
      ret = -EINVAL;
    }

  if (ret < 0)
    {
      file_close(filep);
    }

  return ret;
}

/****************************************************************************
 * Name: vl53l5cx_readfile
 ****************************************************************************/

static int vl53l5cx_readfile(FAR struct file *filep, FAR uint8_t *buf,
                             size_t len)
{
  ssize_t n;
  size_t off;

  for (off = 0; off < len; off += n)
    {
      n = file_read(filep, &buf[off], len - off);
      if (n <= 0)
        {
          return n < 0 ? (int)n : -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_loadfile
 ****************************************************************************/

static int vl53l5cx_loadfile(FAR const char *path, FAR uint8_t *buf,
                             size_t len)
{
  struct file file;
  int ret;

  ret = vl53l5cx_openfile(&file, path, len);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_readfile(&file, buf, len);
  file_close(&file);
  return ret;
}

/****************************************************************************
 * Name: vl53l5cx_sendfile
 *
 * Description:
 *   Stream an open file to consecutive addresses.  If patch is not
 *   negative, the byte at that file offset is replaced by val.
 *
 ****************************************************************************/

static int vl53l5cx_sendfile(FAR struct vl53l5cx_dev_s *priv,
                             FAR struct file *filep, uint16_t reg,
                             size_t len, ssize_t patch, uint8_t val)
{
  FAR uint8_t *chunk = &priv->txbuf[2];
  size_t off;
  size_t n;
  int ret;

  for (off = 0; off < len; off += n)
    {
      n = MIN(len - off, VL53L5CX_CHUNK);
      ret = vl53l5cx_readfile(filep, chunk, n);
      if (ret < 0)
        {
          return ret;
        }

      if (patch >= (ssize_t)off && patch < (ssize_t)(off + n))
        {
          chunk[patch - off] = val;
        }

      ret = vl53l5cx_xfer(priv, reg + off, NULL, n);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_download
 *
 * Description:
 *   Download the firmware into the three firmware pages (5.3 step 7).
 *
 ****************************************************************************/

static int vl53l5cx_download(FAR struct vl53l5cx_dev_s *priv,
                             FAR struct file *fw)
{
  size_t left = VL53L5CX_FW_SIZE;
  size_t n;
  uint8_t page;
  int ret;

  for (page = VL53L5CX_PAGE_FW; left > 0; page++)
    {
      n = MIN(left, VL53L5CX_FW_PAGE_SIZE);
      ret = vl53l5cx_page(priv, page);
      if (ret < 0)
        {
          return ret;
        }

      ret = vl53l5cx_sendfile(priv, fw, 0, n, -1, 0);
      if (ret < 0)
        {
          return ret;
        }

      left -= n;
    }

  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_mcu_boot
 *
 * Description:
 *   Wait for the MCU to boot after its reset (5.4 step 4).
 *
 ****************************************************************************/

static int vl53l5cx_mcu_boot(FAR struct vl53l5cx_dev_s *priv)
{
  uint8_t g0;
  uint8_t err;
  int tries;
  int ret;

  for (tries = 0; tries < VL53L5CX_MCU_BOOT_TRIES; tries++)
    {
      ret = vl53l5cx_getreg(priv, VL53L5CX_MCU_REG, &g0);
      if (ret < 0)
        {
          return ret;
        }

      if (g0 & VL53L5CX_MCU_STOPPED)
        {
          ret = vl53l5cx_getreg(priv, VL53L5CX_MCU_ERR_REG, &err);
          if (ret >= 0 && err != 0)
            {
              snerr("ERROR: MCU boot error 0x%02x\n", err);
              ret = -EIO;
            }

          return ret;
        }

      nxsig_usleep(VL53L5CX_MCU_BOOT_US);
      if (g0 & VL53L5CX_MCU_READY)
        {
          return OK;
        }
    }

  snerr("ERROR: MCU boot timeout\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: vl53l5cx_nvm
 *
 * Description:
 *   Read the factory offset record (5.5).
 *
 ****************************************************************************/

static int vl53l5cx_nvm(FAR struct vl53l5cx_dev_s *priv)
{
  int ret;

  ret = vl53l5cx_loadfile(CONFIG_VL53L5CX_NVMREQ_PATH, priv->scratch,
                          VL53L5CX_NVMREQ_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_write(priv, VL53L5CX_UI_END - VL53L5CX_NVMREQ_SIZE,
                       priv->scratch, VL53L5CX_NVMREQ_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_pollui(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_read(priv, VL53L5CX_UI_DATA_REG, priv->scratch,
                      VL53L5CX_NVM_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(priv->ofs, priv->scratch, VL53L5CX_OFS_SIZE);
  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_boot
 *
 * Description:
 *   Reboot the sensor, download the firmware and the configuration
 *   (section 5).  The sensor is left stopped, 4x4, in continuous mode.
 *
 ****************************************************************************/

static int vl53l5cx_boot(FAR struct vl53l5cx_dev_s *priv)
{
  static const uint8_t pipe[4] =
    {
      1, 0, 1, 0                      /* One target, N-view */
    };

  static const uint8_t glare1 = VL53L5CX_GLARE_EN1;
  static const uint8_t glare2 = VL53L5CX_GLARE_EN2;
  static const uint8_t mode_off[2] =
    {
      1, 3
    };

  static const uint8_t mode_val[2] =
    {
      1, 3                            /* Continuous: N1 = 1, N3 = 3 */
    };

  static const uint8_t one = 1;
  struct file fw;
  struct file cfg;
  int ret;

  /* Check every file before touching the sensor */

  ret = vl53l5cx_loadfile(CONFIG_VL53L5CX_XTALK_PATH, priv->xtalk,
                          VL53L5CX_XTALK_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_openfile(&fw, CONFIG_VL53L5CX_NVMREQ_PATH,
                          VL53L5CX_NVMREQ_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  file_close(&fw);

  ret = vl53l5cx_openfile(&fw, CONFIG_VL53L5CX_FIRMWARE_PATH,
                          VL53L5CX_FW_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_openfile(&cfg, CONFIG_VL53L5CX_CONFIG_PATH,
                          VL53L5CX_CFG_SIZE);
  if (ret < 0)
    {
      file_close(&fw);
      return ret;
    }

  /* Software reboot and boot wait (5.2) */

  ret = vl53l5cx_runseq(priv, g_vl53l5cx_reboot,
                        nitems(g_vl53l5cx_reboot));
  if (ret >= 0)
    {
      ret = vl53l5cx_pollreg(priv, VL53L5CX_MCU_REG, VL53L5CX_MCU_ALL,
                             VL53L5CX_MCU_READY, VL53L5CX_POLL_TRIES);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_runseq(priv, g_vl53l5cx_booted,
                            nitems(g_vl53l5cx_booted));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_pollreg(priv, VL53L5CX_FW_REG, VL53L5CX_FW_READY,
                             VL53L5CX_FW_READY, VL53L5CX_POLL_TRIES);
    }

  /* Power on, MCU wake and firmware download (5.3) */

  if (ret >= 0)
    {
      ret = vl53l5cx_runseq(priv, g_vl53l5cx_poweron,
                            nitems(g_vl53l5cx_poweron));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_download(priv, &fw);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_runseq(priv, g_vl53l5cx_fwcheck,
                            nitems(g_vl53l5cx_fwcheck));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_pollreg(priv, VL53L5CX_FW_REG, VL53L5CX_FW_READY,
                             VL53L5CX_FW_READY, VL53L5CX_POLL_TRIES);
    }

  /* MCU reset and boot (5.4) */

  if (ret >= 0)
    {
      ret = vl53l5cx_runseq(priv, g_vl53l5cx_mcuboot,
                            nitems(g_vl53l5cx_mcuboot));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_mcu_boot(priv);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_page(priv, VL53L5CX_PAGE_MAIN);
    }

  /* Offset record, default crosstalk and configuration (5.5, 5.6) */

  if (ret >= 0)
    {
      ret = vl53l5cx_nvm(priv);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_upload_offset(priv, true);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_upload_xtalk(priv, true);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_sendfile(priv, &cfg,
                              VL53L5CX_UI_END - VL53L5CX_CFG_SIZE,
                              VL53L5CX_CFG_SIZE, VL53L5CX_CFG_NBTAR,
                              VL53L5CX_FW_NBTAR);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_pollui(priv, false);
    }

  file_close(&cfg);
  file_close(&fw);

  /* Post-config parameters (5.7), then continuous mode (6.2) */

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_write(priv, VL53L5CX_DCI_PIPE_CTRL, pipe,
                               sizeof(pipe));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_put32(priv, VL53L5CX_DCI_SINGLE_RANGE, 1);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_patch(priv, VL53L5CX_DCI_GLARE,
                               VL53L5CX_GLARE_LEN, &glare1, &one, 1);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_patch(priv, VL53L5CX_DCI_GLARE,
                               VL53L5CX_GLARE_LEN, &glare2, &one, 1);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_patch(priv, VL53L5CX_DCI_MODE, VL53L5CX_MODE_LEN,
                               mode_off, mode_val, nitems(mode_off));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_put32(priv, VL53L5CX_DCI_SINGLE_RANGE, 0);
    }

  if (ret < 0)
    {
      snerr("ERROR: boot failed: %d\n", ret);
      return ret;
    }

  priv->devzones = VL53L5CX_WIDTH_4X4 * VL53L5CX_WIDTH_4X4;
  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_power
 *
 * Description:
 *   Put the stopped sensor to sleep or wake it up (10.2).
 *
 ****************************************************************************/

static int vl53l5cx_power(FAR struct vl53l5cx_dev_s *priv, bool sleep)
{
  int ret;

  ret = vl53l5cx_page(priv, VL53L5CX_PAGE_BOOT);
  if (ret >= 0)
    {
      ret = vl53l5cx_putreg(priv, VL53L5CX_POWER_REG,
                            sleep ? VL53L5CX_POWER_SLEEP :
                                    VL53L5CX_POWER_WAKE);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_pollreg(priv, VL53L5CX_MCU_REG, VL53L5CX_MCU_READY,
                             sleep ? 0 : VL53L5CX_MCU_READY,
                             VL53L5CX_POLL_TRIES);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_page(priv, VL53L5CX_PAGE_MAIN);
    }

  if (ret >= 0)
    {
      priv->asleep = sleep;
    }

  return ret;
}

/****************************************************************************
 * Name: vl53l5cx_set_zones
 *
 * Description:
 *   Set the 4x4 or 8x8 resolution and upload the matching offset and
 *   crosstalk records (6.1).
 *
 ****************************************************************************/

static int vl53l5cx_set_zones(FAR struct vl53l5cx_dev_s *priv, bool is4x4)
{
  static const uint8_t dss_off[3] =
    {
      4, 6, 9
    };

  static const uint8_t dss_4x4[3] =
    {
      64, 64, 4
    };

  static const uint8_t dss_8x8[3] =
    {
      16, 16, 1
    };

  static const uint8_t zone_off[4] =
    {
      0, 1, 4, 5
    };

  static const uint8_t zone_4x4[4] =
    {
      4, 4, 8, 8
    };

  static const uint8_t zone_8x8[4] =
    {
      8, 8, 4, 4
    };

  int ret;

  ret = vl53l5cx_dci_patch(priv, VL53L5CX_DCI_DSS, VL53L5CX_DSS_LEN,
                           dss_off,
                           is4x4 ? dss_4x4 : dss_8x8, nitems(dss_off));
  if (ret >= 0)
    {
      ret = vl53l5cx_dci_patch(priv, VL53L5CX_DCI_ZONE_CFG,
                               VL53L5CX_ZONE_CFG_LEN, zone_off,
                               is4x4 ? zone_4x4 : zone_8x8,
                               nitems(zone_off));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_upload_offset(priv, is4x4);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_upload_xtalk(priv, is4x4);
    }

  return ret;
}

/****************************************************************************
 * Name: vl53l5cx_stop_ranging
 *
 * Description:
 *   Stop ranging (section 9).
 *
 ****************************************************************************/

static int vl53l5cx_stop_ranging(FAR struct vl53l5cx_dev_s *priv)
{
  uint8_t cmd[4];
  uint8_t b = 0;
  int ret;
  int err = OK;

  ret = vl53l5cx_read(priv, VL53L5CX_CMD_REG, cmd, sizeof(cmd));
  if (ret < 0)
    {
      return ret;
    }

  ret = vl53l5cx_page(priv, VL53L5CX_PAGE_BOOT);
  if (ret < 0)
    {
      return ret;
    }

  if (memcmp(cmd, g_vl53l5cx_stopped, sizeof(cmd)) != 0)
    {
      ret = vl53l5cx_putreg(priv, VL53L5CX_STOP_ARG_REG, VL53L5CX_STOP_ARG);
      if (ret >= 0)
        {
          ret = vl53l5cx_putreg(priv, VL53L5CX_STOP_CMD_REG,
                                VL53L5CX_STOP_CMD);
        }

      if (ret >= 0)
        {
          err = vl53l5cx_pollreg(priv, VL53L5CX_MCU_REG,
                                 VL53L5CX_MCU_STOPPED, VL53L5CX_MCU_STOPPED,
                                 VL53L5CX_STOP_TRIES);
        }
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_getreg(priv, VL53L5CX_MCU_REG, &b);
    }

  if (ret >= 0 && (b & VL53L5CX_MCU_STOPPED) != 0)
    {
      ret = vl53l5cx_getreg(priv, VL53L5CX_MCU_ERR_REG, &b);
      if (ret >= 0 && b != VL53L5CX_MCU_STOP_OK1 &&
          b != VL53L5CX_MCU_STOP_OK2)
        {
          snerr("ERROR: stop status 0x%02x\n", b);
          err = -EIO;
        }
    }

  /* Clean up even after an error */

  if (ret >= 0)
    {
      ret = vl53l5cx_putreg(priv, VL53L5CX_STOP_CMD_REG, 0);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_putreg(priv, VL53L5CX_STOP_ARG_REG, 0);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_putreg(priv, VL53L5CX_POWER_REG, VL53L5CX_POWER_WAKE);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_page(priv, VL53L5CX_PAGE_MAIN);
    }

  return ret < 0 ? ret : err;
}

/****************************************************************************
 * Name: vl53l5cx_start_ranging
 *
 * Description:
 *   Configure the outputs and start ranging (section 7).
 *
 ****************************************************************************/

static int vl53l5cx_start_ranging(FAR struct vl53l5cx_dev_s *priv)
{
  uint8_t list[VL53L5CX_NOUTPUTS * 4];
  uint8_t cfg[8];
  uint8_t mask[16];
  uint8_t check[VL53L5CX_OUTPUT_CHECK_LEN];
  uint32_t hdr;
  uint32_t size;
  uint32_t type;
  uint32_t fsize = VL53L5CX_FRAME_OVERHEAD;
  int i;
  int ret;

  /* Output list with element counts, and the frame size */

  for (i = 0; i < VL53L5CX_NOUTPUTS; i++)
    {
      hdr  = g_vl53l5cx_outputs[i];
      type = VL53L5CX_HDR_TYPE(hdr);
      if ((g_vl53l5cx_enabled & (1 << i)) != 0)
        {
          if (type >= 1 && type <= VL53L5CX_TYPE_ELEM_MAX)
            {
              /* Zone count, times the targets per zone (one) for the
               * per-target blocks.
               */

              size  = priv->zones;
              hdr   = (hdr & ~VL53L5CX_HDR_SIZE_MASK) |
                      (size << VL53L5CX_HDR_SIZE_SHIFT);
              fsize += 4 + type * size;
            }
          else
            {
              fsize += 4 + VL53L5CX_HDR_SIZE(hdr);
            }
        }

      vl53l5cx_putle32(&list[i * 4], hdr);
    }

  if (fsize > sizeof(priv->scratch))
    {
      return -EINVAL;
    }

  priv->framesize = fsize;

  vl53l5cx_putle32(&cfg[0], fsize);
  vl53l5cx_putle32(&cfg[4], VL53L5CX_OUT_COUNT);

  memset(mask, 0, sizeof(mask));
  vl53l5cx_putle32(&mask[0], g_vl53l5cx_enabled);
  vl53l5cx_putle32(&mask[12], VL53L5CX_OUT_MASK3);

  ret = vl53l5cx_dci_write(priv, VL53L5CX_DCI_OUTPUT_LIST, list,
                           sizeof(list));
  if (ret >= 0)
    {
      ret = vl53l5cx_dci_write(priv, VL53L5CX_DCI_OUTPUT_CFG, cfg,
                               sizeof(cfg));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_write(priv, VL53L5CX_DCI_OUTPUT_MASK, mask,
                               sizeof(mask));
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_page(priv, VL53L5CX_PAGE_BOOT);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_putreg(priv, VL53L5CX_POWER_REG,
                            VL53L5CX_POWER_RANGING);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_page(priv, VL53L5CX_PAGE_MAIN);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Start command, then check that the firmware took the outputs */

  ret = vl53l5cx_write(priv, VL53L5CX_CMD_REG, g_vl53l5cx_start_cmd,
                       sizeof(g_vl53l5cx_start_cmd));
  if (ret >= 0)
    {
      ret = vl53l5cx_pollui(priv, false);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_dci_read(priv, VL53L5CX_DCI_OUTPUT_CHECK, check,
                              sizeof(check));
    }

  if (ret >= 0 &&
      vl53l5cx_getle16(&check[VL53L5CX_OUTPUT_CHECK_SIZE]) != fsize)
    {
      snerr("ERROR: output configuration rejected\n");
      ret = -EIO;
    }

  if (ret < 0)
    {
      vl53l5cx_stop_ranging(priv);
      return ret;
    }

  priv->lastcount = VL53L5CX_STREAM_NONE;
  return OK;
}

/****************************************************************************
 * Name: vl53l5cx_parse
 *
 * Description:
 *   Check the result frame in scratch and write the Z16 image (8.2-8.4).
 *   Returns the image size or a negated errno value.
 *
 ****************************************************************************/

static int vl53l5cx_parse(FAR struct vl53l5cx_dev_s *priv,
                          FAR uint8_t *out)
{
  FAR uint8_t *f = priv->scratch;
  FAR const uint8_t *targets = NULL;
  FAR const uint8_t *distance = NULL;
  FAR const uint8_t *status = NULL;
  size_t fsize = priv->framesize;
  size_t p;
  size_t len;
  uint32_t hdr;
  uint32_t type;
  uint32_t size;
  int32_t mm;
  int z;

  vl53l5cx_swap(f, fsize);

  /* The header and the footer IDs differ in a torn frame */

  if (f[VL53L5CX_FRAME_ID] != f[fsize - VL53L5CX_FRAME_FOOTER] ||
      f[VL53L5CX_FRAME_ID + 1] != f[fsize - VL53L5CX_FRAME_FOOTER + 1])
    {
      return -EIO;
    }

  for (p = VL53L5CX_FRAME_BLOCKS; p + 4 <= fsize - VL53L5CX_FRAME_FOOTER;
       p += 4 + len)
    {
      hdr  = vl53l5cx_getle32(&f[p]);
      type = VL53L5CX_HDR_TYPE(hdr);
      size = VL53L5CX_HDR_SIZE(hdr);
      len  = type > 1 && type <= VL53L5CX_TYPE_ELEM_MAX ? type * size :
                                                          size;
      if (p + 4 + len > fsize - VL53L5CX_FRAME_FOOTER)
        {
          break;
        }

      switch (VL53L5CX_HDR_INDEX(hdr))
        {
          case VL53L5CX_BLK_TARGETS:
            targets = len == priv->zones ? &f[p + 4] : NULL;
            break;

          case VL53L5CX_BLK_DISTANCE:
            distance = len == priv->zones * sizeof(int16_t) ?
                       &f[p + 4] : NULL;
            break;

          case VL53L5CX_BLK_STATUS:
            status = len == priv->zones ? &f[p + 4] : NULL;
            break;

          default:
            break;
        }
    }

  if (targets == NULL || distance == NULL || status == NULL)
    {
      return -EIO;
    }

  for (z = 0; z < priv->zones; z++)
    {
      mm = (int16_t)vl53l5cx_getle16(&distance[z * 2]);
      mm /= 1 << VL53L5CX_DISTANCE_SHIFT;
      if (targets[z] == 0 || mm < 0 ||
          (status[z] != VL53L5CX_STATUS_VALID &&
           status[z] != VL53L5CX_STATUS_VALID_LARGE))
        {
          mm = 0;
        }

      out[z * 2]     = mm & 0xff;
      out[z * 2 + 1] = (mm >> 8) & 0xff;
    }

  return priv->zones * VL53L5CX_BYTES_PER_PIXEL;
}

/****************************************************************************
 * Name: vl53l5cx_worker
 *
 * Description:
 *   Poll the data-ready word, read and deliver a frame (section 8).
 *
 ****************************************************************************/

static void vl53l5cx_worker(FAR void *arg)
{
  FAR struct vl53l5cx_dev_s *priv = arg;
  imgdata_capture_t callback;
  FAR void *cbarg;
  FAR uint8_t *buf;
  uint32_t bufsize;
  struct timespec ts;
  struct timeval tv;
  irqstate_t flags;
  clock_t delay = MAX(MSEC2TICK(VL53L5CX_READY_MS), 1);
  clock_t period = USEC2TICK(USEC_PER_SEC / priv->fps);
  uint8_t st[4];
  bool frame = false;
  int ret;

  ret = vl53l5cx_read(priv, VL53L5CX_RESULT_REG, st, sizeof(st));
  if (ret >= 0 && st[0] != priv->lastcount &&
      st[0] != VL53L5CX_STREAM_NONE && st[1] == VL53L5CX_READY_B1 &&
      (st[2] & VL53L5CX_READY_B2) == VL53L5CX_READY_B2 &&
      (st[3] & VL53L5CX_READY_B3) == VL53L5CX_READY_B3)
    {
      frame = true;
    }
  else if (ret >= 0 && (st[3] & VL53L5CX_READY_ERROR) != 0)
    {
      snerr("ERROR: ranging error 0x%02x\n", st[2]);
      ret = -EIO;
    }

  if (frame || ret < 0)
    {
      clock_systime_timespec(&ts);
      TIMESPEC_TO_TIMEVAL(&tv, &ts);

      flags    = spin_lock_irqsave(&priv->buflock);
      buf      = priv->buf;
      bufsize  = priv->bufsize;
      callback = priv->callback;
      cbarg    = priv->arg;
      spin_unlock_irqrestore(&priv->buflock, flags);

      if (buf == NULL || callback == NULL)
        {
          /* No buffer queued: drop the frame */

          priv->lastcount = frame ? st[0] : priv->lastcount;
        }
      else
        {
          if (frame && bufsize < priv->zones * VL53L5CX_BYTES_PER_PIXEL)
            {
              priv->lastcount = st[0];
              ret = -ENOBUFS;
            }
          else if (frame)
            {
              ret = vl53l5cx_read(priv, VL53L5CX_RESULT_REG, priv->scratch,
                                  priv->framesize);
              if (ret >= 0)
                {
                  priv->lastcount = priv->scratch[0];
                  ret = vl53l5cx_parse(priv, buf);
                }
            }

          /* The callback queues the next buffer or stops the capture */

          priv->cbtid = nxsched_gettid();
          callback(ret < 0 ? EIO : 0, ret < 0 ? 0 : ret, &tv, cbarg);
          priv->cbtid = INVALID_PROCESS_ID;
        }

      /* The next frame is about one period away */

      if (period > 3 * delay)
        {
          delay = period - 2 * delay;
        }
    }

  flags = spin_lock_irqsave(&priv->buflock);
  if (priv->polling)
    {
      work_queue(LPWORK, &priv->work, vl53l5cx_worker, priv, delay);
    }

  spin_unlock_irqrestore(&priv->buflock, flags);
}

/****************************************************************************
 * Name: vl53l5cx_halt
 *
 * Description:
 *   Stop the data-ready polling and wait for a running poll to end.
 *
 ****************************************************************************/

static void vl53l5cx_halt(FAR struct vl53l5cx_dev_s *priv)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->buflock);
  priv->polling = false;
  spin_unlock_irqrestore(&priv->buflock, flags);

  work_cancel_sync(LPWORK, &priv->work);
}

/****************************************************************************
 * Name: vl53l5cx_check_format
 *
 * Description:
 *   Check a format and an interval, return the zones per row.
 *
 ****************************************************************************/

static int vl53l5cx_check_format(uint8_t nr_fmts, uint16_t width,
                                 uint16_t height, bool z16,
                                 uint32_t num, uint32_t den,
                                 FAR uint8_t *fps)
{
  uint32_t max;

  if (nr_fmts != 1 || !z16 || width != height)
    {
      return -EINVAL;
    }

  if (width == VL53L5CX_WIDTH_8X8)
    {
      max = VL53L5CX_FPS_8X8;
    }
  else if (width == VL53L5CX_WIDTH_4X4)
    {
      max = VL53L5CX_FPS_4X4;
    }
  else
    {
      return -EINVAL;
    }

  /* Whole frames per second only */

  if (num == 0 || den % num != 0 || den / num < VL53L5CX_FPS_MIN ||
      den / num > max)
    {
      return -EINVAL;
    }

  if (fps != NULL)
    {
      *fps = den / num;
    }

  return width;
}

/****************************************************************************
 * Image sensor operations
 ****************************************************************************/

static bool vl53l5cx_is_available(FAR struct imgsensor_s *sensor)
{
  return true;
}

static int vl53l5cx_init(FAR struct imgsensor_s *sensor)
{
  return OK;
}

/* Last close: stop ranging and put the sensor to sleep */

static int vl53l5cx_uninit(FAR struct imgsensor_s *sensor)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(sensor, struct vl53l5cx_dev_s, sensor);
  int ret = OK;

  nxmutex_lock(&priv->lock);
  vl53l5cx_halt(priv);
  if (priv->ranging)
    {
      priv->ranging = false;
      ret = vl53l5cx_stop_ranging(priv);
    }

  if (ret >= 0 && priv->booted && !priv->asleep)
    {
      ret = vl53l5cx_power(priv, true);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static FAR const char *
vl53l5cx_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "VL53L5CX";
}

static int vl53l5cx_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                           imgsensor_stream_type_t type,
                                           uint8_t nr_datafmts,
                                           FAR imgsensor_format_t *datafmts,
                                           FAR imgsensor_interval_t *ival)
{
  int ret;

  ret = vl53l5cx_check_format(nr_datafmts, datafmts->width,
                              datafmts->height,
                              datafmts->pixelformat ==
                              IMGSENSOR_PIX_FMT_Z16,
                              ival->numerator, ival->denominator, NULL);
  return ret < 0 ? ret : OK;
}

static int vl53l5cx_start_capture(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type,
                                  uint8_t nr_datafmts,
                                  FAR imgsensor_format_t *datafmts,
                                  FAR imgsensor_interval_t *interval)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(sensor, struct vl53l5cx_dev_s, sensor);
  static const uint8_t freq = VL53L5CX_FREQ_HZ;
  irqstate_t flags;
  uint8_t fps;
  uint8_t zones;
  int width;
  int ret = OK;

  width = vl53l5cx_check_format(nr_datafmts, datafmts->width,
                                datafmts->height,
                                datafmts->pixelformat ==
                                IMGSENSOR_PIX_FMT_Z16,
                                interval->numerator,
                                interval->denominator, &fps);
  if (width < 0 || type != IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return -EINVAL;
    }

  zones = width * width;

  nxmutex_lock(&priv->lock);

  /* Restarted after running out of buffers: still ranging */

  if (priv->ranging && priv->zones == zones && priv->fps == fps)
    {
      goto poll;
    }

  vl53l5cx_halt(priv);
  if (priv->ranging)
    {
      priv->ranging = false;
      vl53l5cx_stop_ranging(priv);
    }

  if (!priv->booted)
    {
      ret = vl53l5cx_boot(priv);
      if (ret < 0)
        {
          goto out;
        }

      priv->booted = true;
    }

  if (priv->asleep)
    {
      ret = vl53l5cx_power(priv, false);
      if (ret < 0)
        {
          goto out;
        }
    }

  if (priv->devzones != zones)
    {
      priv->devzones = 0;
      ret = vl53l5cx_set_zones(priv, width == VL53L5CX_WIDTH_4X4);
      if (ret < 0)
        {
          goto out;
        }

      priv->devzones = zones;
    }

  ret = vl53l5cx_dci_patch(priv, VL53L5CX_DCI_FREQ, VL53L5CX_FREQ_LEN,
                           &freq, &fps, 1);
  if (ret < 0)
    {
      goto out;
    }

  priv->zones = zones;
  priv->fps   = fps;

  ret = vl53l5cx_start_ranging(priv);
  if (ret < 0)
    {
      goto out;
    }

  priv->ranging = true;

poll:
  flags = spin_lock_irqsave(&priv->buflock);
  if (!priv->polling)
    {
      priv->polling = true;
      work_queue(LPWORK, &priv->work, vl53l5cx_worker, priv, 0);
    }

  spin_unlock_irqrestore(&priv->buflock, flags);

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int vl53l5cx_stop_capture(FAR struct imgsensor_s *sensor,
                                 imgsensor_stream_type_t type)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(sensor, struct vl53l5cx_dev_s, sensor);
  int ret = OK;

  /* v4l2_cap stops the capture from the frame callback when no buffer is
   * left.  Keep ranging then and drop the frames until a buffer is queued.
   */

  if (priv->cbtid == nxsched_gettid())
    {
      return OK;
    }

  nxmutex_lock(&priv->lock);
  vl53l5cx_halt(priv);
  if (priv->ranging)
    {
      priv->ranging = false;
      ret = vl53l5cx_stop_ranging(priv);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int vl53l5cx_get_frame_interval(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       FAR imgsensor_interval_t *interval)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(sensor, struct vl53l5cx_dev_s, sensor);
  int ret = -EINVAL;

  nxmutex_lock(&priv->lock);
  if (priv->ranging)
    {
      interval->numerator   = 1;
      interval->denominator = priv->fps;
      ret = OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Image data operations
 ****************************************************************************/

static int vl53l5cx_data_init(FAR struct imgdata_s *data)
{
  return OK;
}

static int vl53l5cx_data_uninit(FAR struct imgdata_s *data)
{
  return OK;
}

/* Called from the frame callback too: no blocking */

static int vl53l5cx_data_set_buf(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(data, struct vl53l5cx_dev_s, data);
  irqstate_t flags;

  if (addr == NULL || nr_datafmts != 1 ||
      size < (uint32_t)datafmts->width * datafmts->height *
             VL53L5CX_BYTES_PER_PIXEL)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->buflock);
  priv->buf     = addr;
  priv->bufsize = size;
  spin_unlock_irqrestore(&priv->buflock, flags);
  return OK;
}

static int vl53l5cx_data_validate(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmts,
                                  FAR imgdata_format_t *fmts,
                                  FAR imgdata_interval_t *ival)
{
  int ret;

  ret = vl53l5cx_check_format(nr_datafmts, fmts->width, fmts->height,
                              fmts->pixelformat == IMGDATA_PIX_FMT_Z16,
                              ival->numerator, ival->denominator, NULL);
  return ret < 0 ? ret : OK;
}

static int vl53l5cx_data_start_capture(FAR struct imgdata_s *data,
                                       uint8_t nr_datafmts,
                                       FAR imgdata_format_t *datafmts,
                                       FAR imgdata_interval_t *interval,
                                       FAR imgdata_capture_t callback,
                                       FAR void *arg)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(data, struct vl53l5cx_dev_s, data);
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->buflock);
  priv->callback = callback;
  priv->arg      = arg;
  spin_unlock_irqrestore(&priv->buflock, flags);
  return OK;
}

/* Called from the frame callback too: no blocking there */

static int vl53l5cx_data_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct vl53l5cx_dev_s *priv =
    container_of(data, struct vl53l5cx_dev_s, data);
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->buflock);
  priv->buf      = NULL;
  priv->callback = NULL;
  spin_unlock_irqrestore(&priv->buflock, flags);

  /* Outside of the callback, wait until no frame is being delivered */

  if (priv->cbtid != nxsched_gettid())
    {
      vl53l5cx_halt(priv);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l5cx_register
 *
 * Description:
 *   Register the VL53L5CX as a V4L2 capture device.
 *
 ****************************************************************************/

int vl53l5cx_register(FAR const char *devpath,
                      FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct vl53l5cx_dev_s *priv;
  FAR struct imgsensor_s *sensor;
  uint8_t id = 0;
  uint8_t rev = 0;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  priv = kmm_zalloc(sizeof(struct vl53l5cx_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c   = i2c;
  priv->addr  = addr;
  priv->cbtid = INVALID_PROCESS_ID;
  nxmutex_init(&priv->lock);
  spin_lock_init(&priv->buflock);

  /* Presence check (5.1) */

  ret = vl53l5cx_page(priv, VL53L5CX_PAGE_BOOT);
  if (ret >= 0)
    {
      ret = vl53l5cx_getreg(priv, VL53L5CX_ID_REG, &id);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_getreg(priv, VL53L5CX_REV_REG, &rev);
    }

  if (ret >= 0)
    {
      ret = vl53l5cx_page(priv, VL53L5CX_PAGE_MAIN);
    }

  if (ret >= 0 && (id != VL53L5CX_ID || rev != VL53L5CX_REV))
    {
      snerr("ERROR: device ID 0x%02x, revision 0x%02x\n", id, rev);
      ret = -ENODEV;
    }

  if (ret < 0)
    {
      goto errout;
    }

  priv->sensor.ops              = &g_vl53l5cx_ops;
  priv->sensor.fmtdescs         = g_vl53l5cx_fmtdescs;
  priv->sensor.fmtdescs_num     = nitems(g_vl53l5cx_fmtdescs);
  priv->sensor.frmsizes         = g_vl53l5cx_frmsizes;
  priv->sensor.frmsizes_num     = nitems(g_vl53l5cx_frmsizes);
  priv->sensor.frmintervals     = g_vl53l5cx_frmintervals;
  priv->sensor.frmintervals_num = nitems(g_vl53l5cx_frmintervals);
  priv->data.ops                = &g_vl53l5cx_data_ops;

  sensor = &priv->sensor;
  ret = capture_register(devpath, &priv->data, &sensor, 1);
  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
