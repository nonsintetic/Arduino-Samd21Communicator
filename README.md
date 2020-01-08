# AT SAM D21 Communicator

Original idea by Joe (https://hackaday.io/joecrop) on hackaday.io and his Star Trek Communicator https://hackaday.io/project/19700-star-trek-communicator-badge .

The original project uses a Teensy. I'm not a fan of Teensy's closed source bootloader and it also requires an extra chip for the usb -> serial (more board space, money etc). 
This project uses an ATSAM D21 processor like is found in the Feather M0 from Adafruit. This project also uses a different library, RadioHead by Mike McCauley. The library is compatible with a wide range of RF modules (http://www.airspayce.com/mikem/arduino/RadioHead/ for a list), in theory this code should run on most of them without many changes.

# Changes from the original project
- different library - RadioHead (http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.74.zip), seems more reliable on the M0 than the RFM69 library from LowPowerLab
- found a way to read the ADC pin fast enough to achieve the required 12.5kHz sampling frequency (analogRead() achieves a max of around 400Hz sampling speed, way too low)
- found a way to set up a timed interrupt at the required sampling frequency on the SAMD21 (the original version uses a Teensy specific library for that)

# Unresolved Issues
- reading the audio packets from the RFM69 is interrupting playback on the ADC resulting in choppy audio ( I think ), a faster way of transfering the packets to and from the buffer would probably fix it (like DMA).

# Hardware
I use a Feather M0 proto (any samd21 board should work), an electet microphone with a MAX981-based auto-gain board (manual gain would be better imho) and a RFM69HW module (RFM69HCW should be identical asides from pinout). For the audio output I use a separate PAM8403 module and an 8ohm speaker, but you can use any means of audio amplification you prefer.

# Pin Configuration
You can change all the pins in the #define section of the sketch. VCC and GND are not mentioned below, but use 3.3v for everything. I run everything, except audio out amplification, off the Feather M0's regulator and it seems OK.

You need the following pins from the Feather M0 (or whatever SAMD21 board) to the RF module and microphone:

RF module:
- DIO0 -> any digital pin
- RST -> any digital pin 
- MOSI -> the hardware SPI MOSI pin 
- MISO -> the hardware SPI MISO pin 
- SCK -> the hardware SPI SCK pin 

Microphone:
- audio out -> any analog pin

Audio out:
- A0 on the SAMD21 board is the audio out, it needs to be amplified to use with speakers/headphones etc. Use anything you wish to amplify it, even an LM386 circuit will be ok.
