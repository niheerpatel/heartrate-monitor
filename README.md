This is the sample app to measure heart rate using a pulse sensor
on an Arduino 101. The measured data will then be sent to a connected
smart phone over BLE.

A bluetooth firmware is neccessary to use the nRF51 Bluetooth LE controller.
Follow the instructions here to prepare the firmware:
https://www.zephyrproject.org/doc/board/arduino_101_ble.html#arduino-101-ble

Build and flash the ARC-side application with the following commands:
 $ make pristine && make BOARD=arduino_101_sss_factory ARCH=arc
 $ sudo -E dfu-util -a sensor_core -D output/zephyr.bin

Build and flash the x86-side application with these commands:
 $ make pristine && make BOARD=arduino_101_factory ARCH=x86
 $ sudo -E dfu-util -a x86_app -D output/zephyr.bin

![Some Image](https://github.com/niheerpatel/heartrate-monitor/docs/assets/hr-monitor.jpeg)
