# Mindshark Dome Poi Control ESP32C3 BNO055

This code is loaded onto an esp32C3 on a custom board connected to the BNO055.

# To upload

- Download and install the Arduino IDE from the official website.
- In the Arduino IDE, go to File > Preferences, and in the "Additional Boards Manager URLs" field, add the following URL:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
- Go to Tools > Board > Boards Manager, search for "ESP32", and install the "esp32" package by Espressif Systems.
- Select your board as "ESP32C3 Dev Module" under Tools > Board.
- Choose the correct port under Tools > Port.
- Tool-> USB-CDC`-> Enable. MAKE SURE `USB-CDC On Boot` IS ENABLED. Otherwise serial output will not travel through USB. 
- Connect the board to the computer, note you will need to enable permissions. This often requires a machine reboot.
- While flipping the switch, hold down the button on top of the board to go into flash mode.
- Open this sketch and click on the Upload button.

# To debug

[CoolTerm](https://freeware.the-meiers.org/) is a nice serial reader which will not error out when the device is disconnected. And will auto connect when the device is reconnected.
