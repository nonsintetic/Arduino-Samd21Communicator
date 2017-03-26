//Full credit for this method to https://hackaday.io/joecrop and his https://hackaday.io/project/19700-star-trek-communicator-badge project
//the RFM69 library and method are 100% his creation
//all I did was change it to work with AT SAM D21 micros

#include <RFM69.h>
#include <SPI.h>
#include "adc.h"
#include "timer.h"

//node info
#define NETWORKID     22   // Network ID - must be same for all modules (0 to 255)
#define MYNODEID      1    // CHANGE THIS FOR EACH NODE - The node's ID (0 to 255)
#define TONODEID      2    // Destination node ID (0 to 254, but you can use 255 to broadcast to all nodes on network)

//Pin Setup
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

//packet and buffer setup
#define PACKET_SIZE 64 //size of the packet is closely related to the FIFO size on the module
static uint8_t sendbuffer[PACKET_SIZE];
uint8_t data; //this will store the value we get from the ADC
uint32_t sampleRate = 12500; //the rate at which we will sample and send data from the ADC (in Hz)

//RF module library object
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQ);


void setup() {
  //hard reset RF module by doing a power cycle (it gets stuck sending garbage if you reset the main board)
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  //setup the audio pins
  pinMode(AUDIO_IN, INPUT);
  pinMode(AUDIO_OUT, OUTPUT);

  //init ADC, it will be used to read the audio data
  initADC(AUDIO_IN);
  
  // Initialize the RFM69HCW/RFM69HW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  #if IS_RFM69HCW == true
  radio.setHighPower(true); // Always use this for RFM69HCW and RFM69HW
  #endif
  radio.setPowerLevel(10); //0-32 - power level, for close range just use 1 to make sure there's no power supply problems
  radio.streamingMode();

  //setup a buffer of 64 '1010 1010' bytes
  //we will use this to fill the FIFO on the RF module if it gets empty
  //being a very regular binary sequence we can check for signal integrity also
  for(int i = 0; i<PACKET_SIZE;i++) {
    sendbuffer[i] = 0xAA;
  }  
  
  //send the buffer to the RF module as to get it going
  radio.stream(TONODEID, sendbuffer, PACKET_SIZE);
 
  //Start an interrupt timer that will call TC5_handler over and over at <sampleRate>Hz
  tcConfigure(sampleRate);
  tcStartCounter();
}

void loop() {
  //check every 200ms if the FIFO is empty, if so just send the buffer
  delay(200);
  if(digitalRead(RFM69_IRQ)) radio.stream(TONODEID, sendbuffer, PACKET_SIZE);
}


//this is the interrupt handler, it will get called automatically at <sampleRate>Hz
void TC5_Handler (void) { 
 data = analogReadFast(AUDIO_IN); //get a reading from ADC, this will be an 8bit (1byte) value
 if(!digitalRead(RFM69_IRQ)) {    //if the buffer is not empty
    radio.sendStreamByte(data);   //send 1 byte via the RF module to the network
    analogWrite(AUDIO_OUT,data);  //OPTIONAL: write the value to the DAC pin so we can debug - disable if not used
 }  
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //keep the timer going
}
