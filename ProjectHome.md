**MultiWii** is an open source software project aiming to provide the brain of a RC controlled multi rotor flying platform. It is compatible with several hardware boards and sensors.

The first and most famous setup is the association of a Wii Motion Plus and a Arduino pro mini board.

# Download #

  * [MultiWii\_2.4](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGZFRQX3ZhNVlXbU0)

  * [MultiWii\_2.3](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGWXVZcDRUN2FjMlE)
  * [MultiWii\_2.2](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGaWplU1Nxb1NXYkU)
  * [MultiWii\_2.1](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGZUt2LXBrZzdMaU0)
  * [MultiWii\_2.0](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGVWh1MUZ3SmV5WG8)
  * [MultiWii\_1.9](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGLWVvQWh2cVZEY1U)
  * [MultiWii\_1.8](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGbWdmaUs3eExyTzQ)
  * [MultiWii\_1.7](https://drive.google.com/uc?export=download&id=0B8_pEI-_SlWGODNSXzI5YXNQSnM)

# List of supported features #
  * multi Rotor type
    * BICOPTER
    * TRICOPTER
    * QUAD +
    * QUAD X
    * HEX Y6
    * HEX FLAT +
    * HEX FLAT X
    * OCTO X8
    * OCTO FLAT +
    * OCTO FLAT X
  * Gimbal
    * when associated with an accelerometer, MultiWii is able to drive 2 servos for PITCH and ROLL gimbal system adjustment
    * the gimbal can also be ajdusted via 2 RC channels
  * Camera trigger output
    * a servo output can be dedicated to trigger a camera button. A servo travel pattern is defined in this case
  * GUI:
    * Coded with processing, java core: Linux/MAC/PC compatible, USB connection
    * exhaustive parameter configuration
    * graphical visualization of sensors, motors and RC signal

  * Flight mode:
    * one of the following mode
      * angle velocity driven (accro mode)
      * absolute angle driven (level mode)
    * optional mode, compatible with the 2 previous one
      * altitude assisted mode (baro mode, compatible with the 2 previous mode)
      * head lock assisted mode (magneto mode)

  * Hardware compatibility
    * receiver
      * any standard receiver with a minimum of 4 RC channels
      * any PPM sum receiver
    * servo
      * up to 4 any standard 50Hz servos can be used
    * motor ESCs
      * up to 8 standard ESC can be used, boosted with a 488Hz refresh rate
    * sensors
      * 3 MEMS Gyro
        * 2x IDG-650, 1x ISZ-650 (genuine Wii Motion Plus)
        * ITG3200
      * 3 MEMS Acc (optional)
        * LIS3L02AL (genuine Nunchuk)
        * BMA020
        * BMA180
      * 3 MEMS magnetometer  (optional)
        * HMC5883
        * HMC5843
      * 1 MEMS barometer  (optional)
        * BMP085
  * LCD for configuration of every parameters on the field
    * any Sparkfun serial 9600 baud LCD 2x16 characters
    * TEXTSTAR LCD 2x16 characters, with its 4 buttons supported