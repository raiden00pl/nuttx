=======
LPS22HB
=======

The LPS22HB is a STMicroelectronics I2C absolute pressure sensor. It
measures from 260 to 1260 hPa with a 24-bit output and also reports the die
temperature.

The driver uses the :doc:`uorb </os/drivers/special/sensors/sensors_uorb>`
interface. It is a new driver: NuttX had no LPS22HB driver before, so there
is no legacy character device variant.

Configuration
=============

- ``CONFIG_SENSORS_LPS22HB`` enables the driver.
- ``CONFIG_LPS22HB_I2C_FREQUENCY`` sets the I2C bus frequency, at most
  400 kHz.

Application Programming Interface
=================================

The header file for the LPS22HB driver interface can be included using:

.. code-block:: c

   #include <nuttx/sensors/lps22hb.h>

Registering the driver creates one topic, ``sensor_baro<n>``, where ``n`` is
the ``devno`` passed in:

.. code-block:: c

   int lps22hb_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                             uint8_t addr);

For example, a part at 0x5c on I2C bus 0, published as ``sensor_baro0``:

.. code-block:: c

   struct i2c_master_s *i2c = board_i2cbus_initialize(0);

   lps22hb_register_uorb(0, i2c, 0x5c);

Registration checks the WHO_AM_I register.

Reading
=======

The topic uses the ``fetch`` method. A read enables block data update,
triggers a one-shot conversion, waits for both the pressure and the
temperature data-ready flags, and burst reads the five output bytes. Pressure
is reported in hPa (raw count divided by 4096) and temperature in degrees
Celsius (raw count divided by 100).

The driver does not run its own polling thread. The sampling interval set
with ``SNIOC_SET_INTERVAL`` is stored but the part is only read when the
application reads the topic.
