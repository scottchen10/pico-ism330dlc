# Pico-ISM330DLC

2025-12-XX

## Overview:

A C libary for the Raspberry Pi Pico SDK to interface with the ST ISM330DLC iNEMO 6-axis inertial measurement unit (IMU). The project contains a platform independent driver core (`/ism330dlc_driver`) with the platform abstraction being implemented in (`/platform`). A few basic examples in (`/examples`) can be used to verify the functionality of the project and your sensor.

<blockquote>
<b>Notice:</b> <br>
I am a second year computer engineering student and this project is meant to teach myself how to interface with external devices and to showcase what I've learnt so far. <br>
This driver is not a complete implementation of all the sensor's features.
</blockquote>


## Features:

* 4-wire SPI and I²C communication for pico devices
* Reading gyroscope, accelerometer, and temperature sensor raw measurements with conversions available to rad/s, deg/s, m/s², g, and Celcius
* Configurable gyroscope and accelerometer full scale, output data rates and performance modes
* Configurable interrupt pins (high/low active, push-pull/open-drain driven)
* Configurable event-detection interrupts (tilt detection, free-fall, wake-up, 6D/4D orientation, single and double tap)

## Missing Features:

* Sensor-hub implementation
* 4kb FIFO Buffer
* Miscellaneous configurable settings (Ex: low pass filter gains for gyroscope and accelerometer)

## Getting Started:

This project can be built standalone with the examples as excecutables or integrated as a library into an existing CMake project. The below will explain how to get started with both options.

### Prerequisites:

Before building, ensure you have the following environment set up:

* **Pico SDK:** Installed and configured.
* **Environment Variable:** `PICO_SDK_PATH` must point to your Pico SDK installation directory.
* **Toolchain:** `arm-none-eabi-gcc` installed and in your system path.
* **CMake:** Version 3.21 or higher.

---

### Standalone Build Instructions:
#### Step 1: Set-up the Repository

```bash
# Clone repo
git clone https://github.com/scottchen10/pico-ism330dlc.git
cd pico_ism330dlc

# Initialize submodule
git submodule update --init --recursive
```

#### Step 2. Configure your Targets

The standalone mode is configured by default for the **Raspberry Pi Pico 2 (RP2350)** using the ARM Cortex-M33 architecture. To edit the target microcontroller, and compiler, edit the top-level CMakeLists.txt and edit the following lines as desired

```cmake
set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350-arm-s)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
```

#### Step 3. Build and Compile

```bash
mkdir build && cd build

# Configure to build your desired examples
cmake .. -DBUILD_STANDALONE=ON \
         -DISM330DLC_BUILD_COMPLEMENTARY_FILTER_EXAMPLE=ON

# Compile
make -j$(nproc)

```
#### Step 4. Hardware Setup
<blockquote>
Prior to running each example, read the example's README.md to determine its PIN and hardware setup
</blockquote>

#### Step 5. Flashing the Firmware

Once the build is complete, you will find `.uf2` files in the `build/` directory (or specific example subdirectories). Follow these steps to flash your board:

1. **Enter Bootsel Mode:** Connect your Raspberry Pi Pico to your computer using a USB cable while holding down the **BOOTSEL** button.
2. **Mount Drive:** The Pico will appear as a removable drive named `RPI-RP2` (for RP2040) or `RP2350` (for Pico 2).
3. **Deploy:** Drag and drop the `.uf2` file (e.g., `complementary_filter_demo.uf2`) into that drive.
4. **Run:** The board will automatically reboot and start running the program.
---
### Integration as a Sub-directory Instructions:

If you want to use this driver within your own project, simply add this folder to your project's `CMakeLists.txt`:

```cmake
# In your project's CMakeLists.txt
add_subdirectory(pico-ism330dlc)

# Link the driver to your target
target_link_libraries(your_target_name ism330dlc_driver)
```
And include the header file as desired

```c
#include ism330dlc/ism330dlc_driver.h
```