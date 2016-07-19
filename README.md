# How to build a heart rate monitor using Zephyr\* on Arduino\* 101 boards

- [Introduction](#introduction)
- [Hardware Preparation](#hardware-preparation)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
    - [Environmental Setup](#environmental-setup)
    - [Building and Flashing the BLE Firmware](#building-and-flashing-the-ble-firmware)
    - [Getting application source code](#getting-application-source-code)
    - [Building and flashing the apps](#building-and-flashing-the-apps)
    - [Connecting from a smartphone](#connecting-from-a-smartphone)
- [Understanding the application code](#understanding-the-application-code)
    - [Data flow](#data-flow)
    - [Getting analog signal](#getting-analog-signal)
    - [Using Grove RGB LCD to show the heartbeat](#using-grove-rgb-lcd-to-show-the-heartbeat)
    - [Exchange data between ARC and x86 cores using Interprocess Mailboxes](#exchange-data-between-arc-and-x86-cores-using-interprocess-mailboxes)
    - [Sending data to connected phone over BLE](#sending-data-to-connected-phone-over-ble)
- [Summary](#summary)
- [Notices and disclaimers](#notices-and-disclaimers)

<br>

## Introduction
This is the sample app to measure heart rate using a pulse sensor
on an Arduino 101. The measured data will then be sent to a connected
smart phone over BLE.

A bluetooth firmware is neccessary to use the nRF51 Bluetooth LE controller.
Follow the instructions here to prepare the firmware:
[https://www.zephyrproject.org/doc/board/arduino_101_ble.html#arduino-101-ble](https://www.zephyrproject.org/doc/board/arduino_101_ble.html#arduino-101-ble)

Build and flash the ARC-side application with the following commands:

```
    $ make pristine && make BOARD=arduino_101_sss_factory ARCH=arc`
    $ sudo -E dfu-util -a sensor_core -D output/zephyr.bin`
```

Build and flash the x86-side application with these commands:

`$ make pristine && make BOARD=arduino_101_factory ARCH=x86`

`$ sudo -E dfu-util -a x86_app -D output/zephyr.bin`

The Zephyr Kernel is a small-footprint kernel designed for use on resource-constrained systems: from simple embedded environmental sensors and LED wearables to sophisticated smart watches and IoT wireless gateways. Among the many features that distinguish it from other RTOSes are:

* Rich I/O driver support.
* High configurability.
* Resource definition at compile time that makes it well-suited for quick prototyping. 
* Small footprint.
* Easy to optimize for specific use-cases.


The Arduino 101 board is an interesting Arduino product with an Intel® Curie™ module. Its Intel® Quark™ SE processor has two cores: an ARC core that controls the sensor subsystem, and an x86 core that controls the Bluetooth Low Energy (BLE) chip. Zephyr not only supports all sensor interfaces of the board, but also enables data exchange between the cores via the interprocessor mailboxes (IPM) mechanism and a BLE connection to other devices.
This article introduces Zephyr application development through a step-by-step explanation of how to build a heart rate sensor on an Arduino 101 board. We show details of how to read analog data from a pulse sensor and detect your heartbeat, how to visually display the heart rate and heartbeat on a Grove\* RGB LCD, and how to send heart rate data to a smartphone via a BLE connection using a standard heart rate profile. All source code is mainly based on the samples provided in the Zephyr repository. Readers can learn about the features that Zephyr provides for IoT applications and also can use this information to start building their own apps on Arduino 101 boards.

## Hardware Preparation
The following hardware parts are required to build the heart rate monitor:

* USB type B cable.
* [FTDI USB TTL serial cable.](http://www.ftdichip.com/Products/Cables/USBTTLSerial.htm)
* [Grove RGB LCD display](http://www.seeedstudio.com/wiki/Grove_-_LCD_RGB_Backlight) with a Grove universal 4-pin cable.
* [Pulse sensor.](http://pulsesensor.com/)
* [Arduino Proto Shield.](https://www.arduino.cc/en/Main/ArduinoProtoShield)
* Grove universal 4 pin connector 90°.
* L-type 6-pin male header connector.
* 7-pin female header connector.
* Two 10K ohm resistors.
* Jumper wires.
* A clear case for the Arduino 101 board. It has the same form factor as the Arduino Uno, so any case that fits the Arduino Uno can work.
* A breadboard for those who don't want to do soldering.

There are also some tools needed to build the hardware:

* Soldering tool and iron.
* Wire cutter.

## Hardware setup
Similar to other Arduino boards, the USB type B cable is necessary to flash applications to the Arduino 101 board. It can also be used to power the board at the same time. The FTDI USB TTL serial cable is used to grab data off the serial port for debugging. You can find details on how to connect them in the Zephyr Arduino 101 documentation.
The Grove RGB LCD display operates at 5V and communicates with the Arduino 101 board through the I2C bus using SCL and SDA lines. Since the Arduino 101 board operates at 3.3V and doesn't have an internal pull-up circuit, we need to create one.

1. Connect the 3.3V line to two 10K ohm resistors.
2. Connect the SCL and SDA lines to the pull-up circuit.
3. The SCL and SDA lines then go to the equivalent pins on the Grove RGB LCD display.
4. Connect the 5V and GND pins from the board to those of the display respectively.

The pulse sensor can operate at 3.3V or 5V. We use 3.3V for the Arduino 101. There are three cables connected to the sensor. Connect the red line to the 3.3V pin on the board, the black line to GND, and the purple line to Analog IN A2. You can specify any pin, from A0 to A5, by defining ADC_CHANNEL in the code. Use the following:

* 10 for A0
* 11 for A1
* 12 for A2
* 13 for A3
* 14 for A4
* 15 for A5

Follow the getting started video in PulseSensor.com to prepare the pulse sensor. Take note that sweat on your fingers could potentially cause short circuits that can damage the sensor. Take proper precautions to avoid causing short circuits. 
The Arduino 101 board uses digital pins 0 (RX) and 1 (TX) to send and receive serial data. Connect the following pins and lines:

1. Connect pin 0 (*RX*) on the board to the orange line, pin 5 (*TX*), of the USB FTDI cable.
2. Connect pin 1 (*TX*) of the board to the yellow line, pin 4 (*RX*) of the USB FTDI cable.
3. Finally, connect *GND* to the black line, pin 0 (*GND*) of the FTDI cable.

![Figure 1: Wiring with the breadboard](./docs/assets/image00.png)

*Figure 1: Wiring with the breadboard*

*Figure 1* shows an example of how to wire the device using a breadboard. The two 10K ohm resistors are used to make the pull-up circuit. They are connected to the 3.3V output at one end, and to the SCL and SDA at the other end, which transfer signals to the LCD. The pulse sensor shares the 3.3V line with the pull-up circuit (red cable), and uses A2 to send analog data to the board. The brown and orange jumper wires are connected to pin 0 and pin 1 at one end, and to TX and RX of the USB FTDI card respectively.

![Figure 2: Back of Proto Shield](./docs/assets/image04.png) 

*Figure 2: Front of Proto Shield*


![Figure 3: Front of Proto Shield](./docs/assets/image03.png)

*Figure 3: Front of Proto Shield*


*Figure 2* & *Figure 3* show an Arduino Proto Shield rev. 3 with circuits that are generally similar to the breadboard, but they are soldered down instead. The shield provides convenient 5V and GND connections for circuits. Pin 0 and pin 1 of the 7-pin female connector are bent to reach the 3.3V and GND ports, while the other five pins go into analog input ports.

![Figure 4: Hardware Setup](./docs/assets/image02.png) 

*Figure 4: Hardware Setup*

![Figure 5: Front of Proto Shield](./docs/assets/image06.png)

*Figure 5: Heart Rate Monitor in a case*


*Figure 4* and *Figure 5* show the hardware setup before and after getting the hardware inside a clear case. The Proto Shield has exactly the same form factor as the Arduino 101 board, so you can use some long M3 bolts to hold them together in the clear case.

##Software setup

### Environmental setup

The Zephyr programming environment needs to be set up to compile and flash applications. Details on how to setup the Zephyr programming environment and prepare the Arduino 101 board can be found here.

### Building and flashing the BLE firmware
The Arduino 101 board comes with a Nordic Semiconductor nRF51 Bluetooth LE controller. The Arduino 101 factory-installed firmware on this controller is not supported by Zephyr, so a new one needs to be flashed onto it. Follow the instructions on the [Zephyr website](https://www.zephyrproject.org/doc/board/arduino_101_ble.html#arduino-101-ble) to build and flash the firmware to the board. 

### Getting application source code
Check out the source code for the heart rate monitor application [here](https://www.zephyrproject.org/doc/board/arduino_101.html). 

`$ git clone https://gerrit.zephyrproject.org/r/heartrate-monitor`

### Building and flashing the apps
Build and flash the ARC-side application with the following commands:
 
 
 `$ cd heartrate-monitor`
 `$ make pristine && make BOARD=arduino_101_sss_factory ARCH=arc`
 `$ sudo -E dfu-util -a sensor_core -D output/zephyr.bin`
Build and flash the x86-side application with these commands:
    $ make pristine && make BOARD=arduino_101_factory ARCH=x86
    $ sudo -E dfu-util -a x86_app -D output/zephyr.bin

### Connecting from a smartphone
A portable device that supports BLE can be used to connect to the Arduino 101 board. This example has been tested with the default Health app on the iPhone* and the nRF Toolbox app on Android* devices.

![Figure 6: iOS\* Health App](./docs/assets/image01.png) 

*Figure 6: iOS\* Health App*

![Figure 7: Front of Proto Shield](./docs/assets/image05.png)

*Figure 7: nRF Toolbox App*
