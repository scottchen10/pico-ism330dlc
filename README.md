# Pico-ISM330DLC

2025-12-XX

### Overview:

A C libary for the Raspberry Pi Pico SDK to interface with the ST ISM330DLC iNEMO 6-axis inertial measurement unit (IMU). The project contains a platform independent driver core (`/ism330dlc_driver`) with the platform abstraction being implemented in (`/platform`). A few basic examples in (`/examples`) can be used to verify the functionality of the project and your sensor.

<blockquote>
<b>Notice:</b> <br>
I am a second year computer engineering student and this project is meant to teach myself how to interface with external devices and to showcase what I've learnt so far. <br>
This driver is not a complete implementation of all the sensor's features.
</blockquote>


### Features:

* 4-wire SPI and I²C communication for pico devices
* Reading gyroscope, accelerometer, and temperature sensor raw measurements with conversions available to rad/s, deg/s, m/s², g, and Celcius
* Configurable gyroscope and accelerometer full scale, output data rate (odr) and performance mode
* Configurable interrupt pins (high/low active, push-pull/open-drain driven)
* Configurable event-detection interrupts (tilt detection, free-fall, wake-up, 6D/4D orientation, click and double click)

### Missing Features:

* Sensor-hub implementation
* FIFO
* Miscellaneous configurable settings (Ex: low pass filter gains for gyroscope and accelerometer)
* Other minor features that weren't mentioned

### Getting Started

### License
