# Complementary Filter Demo

### Overview:

This is a demostration of using the ISM330DLC library to implement a quaternion based complementary filter to estimate the IMU's orientation for the Raspberry Pi Pico SDK. The complementary filter is based on the following paper:
<blockquote>
Valenti, R. G., Dryanovski, I., & Xiao, J. (2015). Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. Sensors, 15(8), 19302-19330. https://doi.org/10.3390/s150819302
</blockquote>

---

### Hardware Wiring (I2C):

| ISM330DLC Pin | Pico Pin | Function |
| --- | --- | --- |
| VCC / GND | 3.3V / GND | Power |
| SDA | GP0  | Data |
| SCL | GP1  | Clock |

---
