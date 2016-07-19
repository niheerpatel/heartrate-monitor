# How to build a heart rate monitor using Zephyr\* on Arduino\* 101 boards

- [Hardware Preparation](#Hardware Preparation)
- [TEST](#test)
This is the sample app to measure heart rate using a pulse sensor
on an Arduino 101. The measured data will then be sent to a connected
smart phone over BLE.

A bluetooth firmware is neccessary to use the nRF51 Bluetooth LE controller.
Follow the instructions here to prepare the firmware:
[https://www.zephyrproject.org/doc/board/arduino_101_ble.html#arduino-101-ble](https://www.zephyrproject.org/doc/board/arduino_101_ble.html#arduino-101-ble)

Build and flash the ARC-side application with the following commands:
 $ make pristine && make BOARD=arduino_101_sss_factory ARCH=arc
 $ sudo -E dfu-util -a sensor_core -D output/zephyr.bin

Build and flash the x86-side application with these commands:
 $ make pristine && make BOARD=arduino_101_factory ARCH=x86
 $ sudo -E dfu-util -a x86_app -D output/zephyr.bin

The Zephyr Kernel is a small-footprint kernel designed for use on resource-constrained systems: from simple embedded environmental sensors and LED wearables to sophisticated smart watches and IoT wireless gateways. Among the many features that distinguish it from other RTOSes are:

* Rich I/O driver support.
* High configurability. 
* Resource definition at compile time that makes it well-suited for quick prototyping. 
* Small footprint.
* Easy to optimize for specific use-cases.


The Arduino 101 board is an interesting Arduino product with an Intel® Curie™ module. Its Intel® Quark™ SE processor has two cores: an ARC core that controls the sensor subsystem, and an x86 core that controls the Bluetooth Low Energy (BLE) chip. Zephyr not only supports all sensor interfaces of the board, but also enables data exchange between the cores via the interprocessor mailboxes (IPM) mechanism and a BLE connection to other devices.
This article introduces Zephyr application development through a step-by-step explanation of how to build a heart rate sensor on an Arduino 101 board. We show details of how to read analog data from a pulse sensor and detect your heartbeat, how to visually display the heart rate and heartbeat on a Grove\* RGB LCD, and how to send heart rate data to a smartphone via a BLE connection using a standard heart rate profile. All source code is mainly based on the samples provided in the Zephyr repository. Readers can learn about the features that Zephyr provides for IoT applications and also can use this information to start building their own apps on Arduino 101 boards.

##Hardware Preparation
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

#TEST
Some text