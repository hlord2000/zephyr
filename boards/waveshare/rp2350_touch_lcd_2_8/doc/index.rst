.. zephyr:board:: rp2350_touch_lcd_2_8

Overview
********

The Waveshare RP2350 Touch LCD 2.8" is a development board based on the Raspberry Pi RP2350A
microcontroller with integrated 2.8" touchscreen LCD display, QMI8658 IMU, PCF85063 RTC, SD card
slot, and battery management features.

Hardware
********

- RP2350A dual Cortex-M33 processors at up to 150MHz
- 520KB of SRAM
- 16MB QSPI flash (W25Q128JVSIQ)
- 2.8" ST7789V LCD display (240x320 pixels) with backlight control
- CST328 capacitive touch controller (driver support pending)
- QMI8658 6-axis IMU with dual interrupt lines (driver support pending)
- PCF85063A RTC with interrupt support
- SD card slot (GPIO19-24)
- Battery voltage monitoring with voltage divider
- Battery enable/ship mode control
- RTC interrupt support
- I2S audio interface (GPIO2/3/4)
- USB 1.1 with device support
- User button

Supported Features
==================

.. zephyr:board-supported-hw::

Pin Mapping
===========

The board uses the following GPIO assignments:

- **GPIO0**: UART0 TX (Console)
- **GPIO1**: UART0 RX (Console)
- **GPIO2**: I2S BCK (requires PIO)
- **GPIO3**: I2S LRCLK (requires PIO)
- **GPIO4**: I2S DIN (requires PIO)
- **GPIO5**: RTC interrupt input
- **GPIO6**: I2C1 SDA (Touch + IMU)
- **GPIO7**: I2C1 SCL (Touch + IMU)
- **GPIO8**: IMU INT1
- **GPIO9**: IMU INT2
- **GPIO10**: LCD SPI1 SCK
- **GPIO11**: LCD SPI1 MOSI
- **GPIO12**: LCD SPI1 MISO
- **GPIO13**: LCD SPI1 CS
- **GPIO14**: LCD DC (Data/Command)
- **GPIO15**: LCD Reset
- **GPIO16**: LCD Backlight (PWM)
- **GPIO17**: Touch Reset
- **GPIO18**: Touch Interrupt
- **GPIO19**: SD Card SCK
- **GPIO20**: SD Card CMD
- **GPIO21**: SD Card D0
- **GPIO22**: SD Card D1
- **GPIO23**: SD Card D2
- **GPIO24**: SD Card D3
- **GPIO25**: User Button (sw0) - pulled up
- **GPIO26**: Battery Enable / Ship Mode
- **GPIO27**: Battery Voltage ADC (200k/100k divider)

Peripherals
===========

Display
-------

The board features a 2.8" ST7789V TFT LCD with 240x320 resolution. The display is connected
via SPI1 and supports MIPI-DBI 4-wire SPI mode. Backlight brightness can be controlled via
PWM on GPIO16.

Touch Controller
----------------

A Hynitron CST328 capacitive touch controller is connected via I2C1 at GPIO6/7. The touch
controller provides interrupt-driven touch event reporting via GPIO18, with reset on GPIO17.

**Note**: CST328 driver support is not yet available in Zephyr. The device tree includes
placeholders for future driver integration.

Real-Time Clock
---------------

The board includes an NXP PCF85063A I2C RTC connected to I2C1 at address 0x51. The RTC
provides interrupt output on GPIO5 for alarm and timer functions.

Storage
-------

- **QSPI Flash**: 16MB W25Q128JVSIQ for program storage
- **SD Card**: Accessible via GPIO19-24 (SDIO mode)

Battery Management
------------------

The board includes battery voltage monitoring through a 200k/100k voltage divider connected
to ADC channel 1 (GPIO27). Battery power can be controlled via GPIO26 for ship mode or
power-off functionality.

Audio
-----

I2S audio interface is available on GPIO2/3/4. Note that I2S support on RP2350 typically
requires PIO (Programmable I/O) blocks. This may require custom configuration or future
Zephyr PIO-I2S driver support.

Programming and Debugging
**************************

.. zephyr:board-supported-runners::

The board can be programmed using the UF2 bootloader (drag-and-drop), OpenOCD, probe-rs,
or J-Link debugger. The programming procedure is similar to other RP2350 boards.

UF2 Bootloader
==============

1. Press and hold the BOOTSEL button while connecting USB
2. The board will appear as a USB mass storage device
3. Copy the generated UF2 file to the drive
4. The board will automatically reboot and run the application

OpenOCD
=======

For debugging with OpenOCD, use Raspberry Pi's forked version which includes RP2350 support:

.. zephyr-app-commands::
    :zephyr-app: samples/basic/blinky
    :board: rp2350_touch_lcd_2_8/rp2350a/m33
    :goals: build flash
    :flash-args: --openocd /path/to/openocd

Building Applications
*********************

Build applications for the Cortex-M33 core:

.. zephyr-app-commands::
    :zephyr-app: samples/basic/blinky
    :board: rp2350_touch_lcd_2_8/rp2350a/m33
    :goals: build

Display Sample
==============

To test the display:

.. zephyr-app-commands::
    :zephyr-app: samples/drivers/display
    :board: rp2350_touch_lcd_2_8/rp2350a/m33
    :goals: build flash

Touch Input Sample
==================

To test touch input with LVGL:

.. zephyr-app-commands::
    :zephyr-app: samples/subsys/display/lvgl
    :board: rp2350_touch_lcd_2_8/rp2350a/m33
    :goals: build flash

References
**********

- `Waveshare RP2350 Touch LCD 2.8 Product Page <https://www.waveshare.com>`_
- `RP2350 Datasheet <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>`_
- `ST7789V Display Controller <https://www.sitronix.com.tw/en/product/Driver/mobile_display.html>`_

.. target-notes::
