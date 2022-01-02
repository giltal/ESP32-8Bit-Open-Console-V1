# ESP32-8Bit-Open-Console-V1 (Look at the RTF file for a better  view)

 ESP32 based 8-bit gaming TV console

This project is based on rossumur great 8-bit emulators project (https://github.com/rossumur/esp_8_bit), which emulates Atari 800XL, NES and SMS (Sega Master System)

I have designed and built a special hardware to make this project much more practical and cooler.

The platform is based on ESP32 wroover device with 4MB SRAM.
It has support for native Atari Joystick, and various USB (Yes USB!) controllers including a NES native one.

It supports SD card (FAT32), so games can now be easily placed on it.

It Also enable all 3 emulators at once to be used without the need to flash the ESP32 module as in the original project.

The WII-mote support is now better, and we can connect it any time (not just 5 seconds from power on).

The added SRAM support solved issues with few big size games and makes games loading faster.

This updated platform also emulates Gameboy, Atari 2600 and ZX48 (ZX48 has issues with sound) as separated projects (will enable running all of them with the need to flash in the near future)

All supported projects can be complied on Arduino SDK or under Visual Studio with Visual Micro plug-in.

 
Preparing the SD card:
<img src="https://github.com/giltal/ESP32-8Bit-Open-Console-V1/blob/main/SDcard.jpg" alt="SD card Cont."/>
 

The ESP32_TV_EMU_AtariNES.bin and ESP32_TV_EMU_SMS.bin can be copied from the project directory or just copy them from this GIT.

To use the USB ports you will need to work through this GIT link (https://github.com/atc1441/CH559sdccUSBHost, and replace the main.c file from this GIT), you will need to prepare a USB cable to connect your PC to the platform in which D+\D- are crossed and the 5V is disconnected.

You will also need few Arduino libraries also located on my GIT.

Connection to the console is via USB type C.

This GIT has the schematics, production files and STLs for the package.

Building your own is not for beginners but V2 is on the way with off the shelf ESP32 module to make it a much easier DIY project.
STL files for the box is also on the GIT as well as schematic and production file for the board.
All parts can be purchased on AliExpress.

Feel free to address me for any support issues 😊

gillytal@gmail.com
