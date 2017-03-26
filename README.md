#AT SAM D21 Communicator

Original idea by Joe (https://hackaday.io/joecrop) on hackaday.io and his Star Trek Communicator https://hackaday.io/project/19700-star-trek-communicator-badge .

All I did was change his code to work with SAM D21 microcontrollers. The original project uses a Teensy. I'm not a fan of Teensy's closed source bootloader and it also requires an extra chip for the usb -> serial (more board space, money etc).

#Changes from Joe's project:
- found a way to read the ADC pin fast enough to achieve the required 12.5kHz sampling frequency (analogRead() achieves a max of around 400Hz sampling speed, way too low)
- found a way to set up a timed interrupt at the required sampling frequency on the SAMD21 (the original version uses a Teensy specific library for that)

#Hardware
I use a Feather M0 proto (any samd21 board should work), an electet microphone with a MAX981-based auto-gain board and a RFM69HW module (RFM69HCW should be identical asides from pinout). For the audio output I use a separate PAM8403 module and an 8ohm speaker, but you can use any means of audio amplification you prefer.

#Pin Configuration
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