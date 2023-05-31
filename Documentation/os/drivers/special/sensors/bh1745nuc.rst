=========
BH1745NUC
=========

The BH1745NUC is a Rohm I2C color sensor with red, green, blue and clear
channels, each read out as a 16-bit ADC count.

This driver uses the :doc:`uorb </os/drivers/special/sensors/sensors_uorb>`
interface and lives in ``drivers/sensors``. It is a new driver, not a
conversion of a legacy one: the only other BH1745NUC driver in NuttX is the
cxd56xx SCU character device driver under ``boards/arm/cxd56xx``, selected
with ``CONFIG_SENSORS_BH1745NUC_SCU``, which is unchanged. The shared header
exposes the legacy ``bh1745nuc_init()`` and ``bh1745nuc_register()``
prototypes only for that SCU driver.

Configuration
=============

- ``CONFIG_SENSORS_BH1745NUC`` enables the driver.
- ``CONFIG_BH1745NUC_I2C_FREQUENCY`` sets the I2C bus frequency, at most
  400 kHz.

Application Programming Interface
=================================

The header file for the BH1745NUC driver interface can be included using:

.. code-block:: c

   #include <nuttx/sensors/bh1745nuc.h>

Registering the driver creates one topic, ``sensor_rgb<n>``, where ``n`` is
the ``devno`` passed in:

.. code-block:: c

   int bh1745nuc_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                               uint8_t addr);

For example, a part at 0x38 on I2C bus 0, published as ``sensor_rgb0``:

.. code-block:: c

   struct i2c_master_s *i2c = board_i2cbus_initialize(0);

   bh1745nuc_register_uorb(0, i2c, 0x38);

Registration checks the manufacturer and part ID registers and resets the
part.

Reading
=======

Activating the topic starts RGBC measurement with a 160 ms measurement time
and x16 ADC gain. Deactivating it stops the measurement. A read uses the
``fetch`` method: the driver waits for the valid flag, burst reads the four
channels and reports the red, green and blue counts. The clear channel is
read but not reported because ``struct sensor_rgb`` has no field for it.

The values are raw 16-bit ADC counts, not the percentages the ``sensor_rgb``
topic nominally carries. The sampling interval set with
``SNIOC_SET_INTERVAL`` is stored but the part is only read when the
application reads the topic.
