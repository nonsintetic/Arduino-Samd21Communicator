#include <RFM69.h>
#include <RFM69registers.h>
#include <RFM69_stream.h>
#include "adc.h"
#include "timer.h"
#include "tng_chirp.h"
#include "filter.h"

//---------- SETTINGS - CHANGE THESE -----------
#define AUDIO_IN A5     //MICROMPHONE in pin - for best results use an pre-amplified microphone, either with set gain (best) or with auto gain control (you'll get a lot of noise when it's silent)
#define PUSH_TO_TALK 10 //Push to talk key, you push it.. then talk :)
#define RFM69_CS      5 // --> NSS on the RF module
#define RFM69_IRQ     6 // --> DIO0 on the RF module
#define RFM69_RST     9 // --> RST on the RF module
#define LED_PIN      13 // LED pin for debug etc.
#define FREQUENCY   RF69_433MHZ 
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HCW true
#define NETWORKID     22   // Network ID - must be same for all modules (0 to 255)
#define MYNODEID      1    // CHANGE THIS FOR EACH NODE - The node's ID (0 to 255)
#define TONODEID      255    // Destination node ID (0 to 254, but you can use 255 to broadcast to all nodes on network)
//------------ SETTINGS - OPTIONAL -------------
//#define DEBUG_SINE //outputs a sinewave (a constant tone) instead of audio from mic
#define DEBUG_LOOPBACK //outputs audio from mic on the same device's speaker, good for hearing what you're sending
#define SAMPLE_RATE 12500     //at how many Hz the audio from the analog pin will be sampled
#define PACKET_SIZE 64        //how many bytes each audio packet has, over 60 doesn't seem to work reliably. no real reason to change unless messing with sample rates
//-------------- END OF SETTINGS ---------------

//fast digital reads/writes
volatile uint32_t *setPin = &PORT->Group[g_APinDescription[LED_PIN].ulPort].OUTSET.reg;
volatile uint32_t *clrPin = &PORT->Group[g_APinDescription[LED_PIN].ulPort].OUTCLR.reg;
const uint32_t  PinMASK = (1ul << g_APinDescription[LED_PIN].ulPin);
bool ledState = false;

//sinewave
PROGMEM  prog_uchar sine256[]  = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};
uint8_t sinePos = 0;

//AUDIO TX sample buffer related variables
#define AUDIO_OUT A0    //AUDIO OUT out pin (has to be the DAC pin for best results, this is always pin A0 on an AT SAM D21 microcontroller)
uint8_t audioSample = 0;

//AUDIO RX sample buffer
#define PACKET_SIZE 64 //size of the packet is closely related to the FIFO size on the module
static uint8_t sendbuffer[PACKET_SIZE];

//state machine
#define STATE_RX    0
#define STATE_TX    1
#define STATE_WAIT  2
#define STATE_END   3
#define STATE_SFX   4 //playing a sound effect
uint8_t machineState = STATE_SFX; //this is so it plays the sound once when you start it up (so you know audio is working)
uint8_t machineStateNext = STATE_WAIT;
uint16_t zeroPackets = 0; //used to track end transmiission frames;

//SFX variables
unsigned int sfxPosition = 0; //position within a sound-effect, for tracking which sample we're on

//button and interaction vars
unsigned long btnTxTimer = 0;
boolean btnTxDown = false;
unsigned long mls; //millis

//encryption key
uint8_t cypher;

RFM69_stream radio = RFM69_stream(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQ);

void setup() {
  //Reset the RFM69 module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  
  radioIdle();
  
  Serial.begin(9600);

  //configure the pins
  pinMode(AUDIO_IN, INPUT);
  pinMode(AUDIO_OUT, OUTPUT);
  pinMode(PUSH_TO_TALK, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogWriteResolution(9);   //we're only using 8bit audio so we don't need more

  //initialize the ADC, it will be used to read the audio data
  initADC(AUDIO_IN);

  //setup key
  cypher = random(255);

  //Start an interrupt timer that will call TC5_handler over and over at <sampleRate>Hz
  tcConfigure(SAMPLE_RATE);
  tcStartCounter();
}

void loop() {
  mls = millis();
  handleButtons();
  if (machineState == STATE_WAIT) {
    //we have a message from another radio
    if(radio.receiveDone()) {
      Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
      Serial.print((char*)radio.DATA);
      Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
      
      if (strstr((char *)radio.DATA, "Cypher"))  {
        cypher = *(radio.DATA+8);
        if (radio.ACKRequested()) {        //check if sender wanted an ACK
          radio.sendACK();
          Serial.println(" - ACK sent, start stream RX");
        }
        startStreamingRX();
      }  
    }
  }
}

//init radio and sleep
void radioIdle() {
  machineState = STATE_WAIT;
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.dataRate10k();
  #if IS_RFM69HCW == true
  radio.setHighPower(true); // Always use this for RFM69HCW and RFM69HW
  #endif
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  //radio.encrypt(ENCRYPTKEY);
}

void startStreamingRX() {
  machineState = STATE_RX;
  radio.streamingModeRX();
  zeroPackets = 0;
}

void startStreaming() {
  zeroPackets = 0;
  // intialize buffer to send packet start sequence to synchronize reciever
  for(int i = 0; i < PACKET_SIZE; i++) {
    sendbuffer[i] = 0xAA;
  }
  Serial.print("FILLING FIFO..");
  radio.stream(TONODEID, sendbuffer, PACKET_SIZE);
  Serial.println("done");
  audioSample = 0;
}

//interrupt handler, gets called over and over at <SAMPLE_RATE> Hz
void TC5_Handler (void) {
  /*
  if(ledState == true) {
    *setPin = PinMASK;
  } else {
    *clrPin = PinMASK;
  }
  */
  ledState = !ledState;
  if(machineState == STATE_TX || machineState == STATE_END) {
    #ifdef DEBUG_SINE
    audioSample = sine256[sinePos];
    sinePos++;
    if(sinePos > 256) sinePos = 0;
    #else
    audioSample = analogReadFast(AUDIO_IN); //get a reading from ADC, this will be an 8bit (1byte) value
    #endif
    if(!digitalRead(RFM69_IRQ)) {
      if(machineState == STATE_END) {
        if(zeroPackets < 500) {
          radio.sendStreamByte(0xAA);
          zeroPackets++;
        } else {
          radioIdle();
        }
      } else {        
        radio.sendStreamByte(audioSample);
      }
    }
    #ifdef DEBUG_LOOPBACK    
    analogWrite(AUDIO_OUT,audioSample);  //OPTIONAL: write the value to the DAC pin so we can debug - disable if not used
    #endif
  } else if( machineState == STATE_RX) {
    if(digitalRead(RFM69_IRQ)) {
      //read data from the RF module, write it to the DAC pin
      audioSample = (uint8_t) radio.receiveStreamByte();
      analogWrite(AUDIO_OUT, audioSample);
      
      if(audioSample == 0xAA || audioSample == 0x55) {
        zeroPackets++;
        if(zeroPackets > 100) {
          radioIdle();
          zeroPackets = 0;
        }
      } else if(zeroPackets > 0){
        zeroPackets--;
      }
    }
  } else if( machineState == STATE_SFX) {
    playSound();
  }
  
  //don't change the following line
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //keep the timer going
}

//scans the various buttons and controls and calls the necessary functions, also debounces
void handleButtons() {
  //button MENU debounce
  if( digitalRead(PUSH_TO_TALK) == LOW && (mls - btnTxTimer) >= 500 && btnTxDown == false ) { //more than 1000 ms after last press
    buttonTxAction();
    btnTxTimer = mls;
    btnTxDown = true;
  } else {
    btnTxDown = false;
  }
}

//action for pressing the PUSH TO TALK button
//TODO: add sound fx
void buttonTxAction() {
  Serial.print("TX BUTTON. State:");
  Serial.println(machineState);
  
  if(machineState == STATE_WAIT) {
    //start transmission
    cypher = random(255);
    char radiopacket[10] = "Cypher: ";
    radiopacket[8] = cypher;
    radiopacket[9] = '\0';
    Serial.print("Sending cypher...:");
    Serial.println(radiopacket);
    if (radio.sendWithRetry(TONODEID, radiopacket, 10)) { //target node Id, message as string or byte array, message length
      Serial.println("OK");
    }
    Serial.println("State: Transmit");
    machineState = STATE_TX;
    radio.streamingMode();
    startStreaming();
        
  } else if (machineState == STATE_TX) {
    //send stop frame
    Serial.print("STATE: Stopping transmission");
    machineState = STATE_END;

  } else if (machineState == STATE_END){
    //radioIdle();
  } else if (machineState == STATE_RX){
    Serial.println("STATE: Wait");
    radioIdle();
  }
}

//each time it runs it writes an audio sample from an audio file array to the DAC
void playSound() {
  if(sfxPosition == 0) { Serial.println("CHIRP!"); } 
  if(sfxPosition < tngChirpLength){               //if we're not done playing the sound
    sfxPosition++;                                //keep track of which sample we're on
    analogWrite(AUDIO_OUT,tngChirp[sfxPosition]); //write the sample to the DAC
  } else {                                        //if we're at the end of the sound
    sfxPosition = 0;
    machineState = machineStateNext;              //go to the next state
    machineStateNext = 0;                         //reset the next state
  }
}
