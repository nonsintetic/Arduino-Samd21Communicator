#include <SPI.h>
#include <RH_RF69.h>
#include "timer.h"

//---------- SETTINGS - CHANGE THESE -----------
#define AUDIO_IN A5     //MICROMPHONE in pin - for best results use an pre-amplified microphone, either with set gain or with auto gain control
#define RFM69_CS      5 // --> NSS on the RF module
#define RFM69_IRQ     6 // --> DIO0 on the RF module
#define RFM69_RST     9 // --> RST on the RF module
#define FREQUENCY 433.0 //the RF frequency of the module (in MHz) - CHANGE TO SUIT YOUR MODULE
//#define FREQUENCY 868.0
//#define FREQUENCY 915.0
//------------ SETTINGS - OPTIONAL -------------
#define SAMPLE_RATE 12500 //Sample Rate in Hz, this is how fast the audio samples are played back (has to match Transmitter)
#define PACKET_SIZE 60    //The size of each audio packet, anything over 60 seems to cause stutter (has to match Transmitter)
uint8_t syncwords[] = { 0x2d, 0x64 }; //this set of two bytes identifies a network (channel), they can be any 2 byte values, but they have to be identical on sender and receiver, or they won't hear eachother
//-------------- END OF SETTINGS ---------------

RH_RF69 radio(RFM69_CS,RFM69_IRQ); //radiohead library singleton

//audio sample buffer and related variables
#define AUDIO_OUT A0    //AUDIO OUT out pin (has to be the DAC pin for best results, this is always pin A0 on an AT SAM D21 microcontroller)
volatile uint32_t count = 0;
uint8_t buffer_length = 0;
uint8_t buffer[PACKET_SIZE];
uint16_t buffer_play_index = 0;


void setup() {
  //Reset the RFM69 module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  Serial.begin(9600);
  pinMode(AUDIO_OUT, OUTPUT); //set the DAC pin to output
  analogWriteResolution(9);   //we're only using 8bit audio so we don't need more
  if (!radio.init()) Serial.println("init failed");                       //start the radio module
  if (!radio.setFrequency(433.0)) Serial.println("setFrequency failed");  //set the frequency
  radio.setSyncWords(syncwords, sizeof(syncwords));                       //set the channel/network (see 'syncwords' above for explanation)
  
  //Start an interrupt timer that will call TC5_handler over and over at <sampleRate>Hz
  tcConfigure(SAMPLE_RATE);
  tcStartCounter();
}

void loop() {
  if (radio.available()) { //we have received a packet
    buffer_length = sizeof(buffer);
    radio.recv(buffer, &buffer_length); //read the values in the packet, store them in our audio buffer
    buffer_play_index = 0; // reset the position in the buffer we're reading audio samples for (start from the top of the buffer basically)
  }

}

//interrupt handler, gets called over and over at <sampleRate>Hz 
void TC5_Handler (void) { 
  if (buffer_play_index < buffer_length) { //if there's still stuff in the buffer to read
    analogWrite(AUDIO_OUT,buffer[buffer_play_index++]); //read it and output it on the DAC pin
  }
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //keep the interrupt timer going
}
