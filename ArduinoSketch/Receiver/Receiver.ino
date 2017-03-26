#include <RFM69.h>
#include <SPI.h>
#include "timer.h"

//node info
#define NETWORKID     22   // Network ID - must be same for all modules (0 to 255)
#define MYNODEID      2    // CHANGE THIS FOR EACH NODE - The node's ID (0 to 255)
#define TONODEID      1    // Destination node ID (0 to 254, but you can use 255 to broadcast to all nodes on network)

//Pin setup
#define AUDIO_OUT A0    //AUDIO OUT out pin (has to be the DAC pin for best results, this is always pin A0 on an AT SAM D21 microcontroller)
#define AUDIO_IN A5     //MICROMPHONE in pin - for best results use an pre-amplified microphone, either with set gain or with auto gain control
#define RFM69_CS      5 // --> NSS on the RF module
#define RFM69_IRQ     6 // --> DIO0 on the RF module
#define RFM69_RST     9 // --> RST on the RF module

//RF module setup
#define IS_RFM69HCW true //set to 'false' if not using a RFM69HCW or RFM69HW (the high power ones)
//set the comments on the next part according to your module's frequency
#define FREQUENCY   RF69_433MHZ 
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ

uint32_t sampleRate = 12500; //the rate at which we will sample and send data from the ADC (in Hz)

//Library object
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQ);

void setup() {
  //set the DAC resolution, we're only sending 8bit audio
  analogWriteResolution(9);

  //hard reset RF module by doing a power cycle (it gets stuck sending garbage if you reset the main board)
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  //audio pin setup
  pinMode(AUDIO_OUT, OUTPUT);
  
  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  #if IS_RFM69HCW == true
  radio.setHighPower(true); // Always use this for RFM69HCW and RFM69HW
  #endif
  radio.setPowerLevel(1);   // 0-32 - power level, for close range just use 1 to make sure there's no power supply problems
  radio.streamingModeRX();  //puts the RF module into a very permissive receive mode, no packet length, encoding etc. This makes it fast enough for audio.
  
  //Start an interrupt timer that will call TC5_handler over and over at <sampleRate>Hz
  tcConfigure(sampleRate);
  tcStartCounter();
}

void loop() {
  //everythig is handled in the interrupt below
}


void TC5_Handler (void) {
  if(digitalRead(RFM69_IRQ)) {
    //read data from the RF module, write it to the DAC pin
    analogWrite(AUDIO_OUT, (uint8_t) radio.receiveStreamByte());
  }
}

