#include <SPI.h>
#include <RH_RF69.h>
#include "adc.h"
#include "timer.h"
#include "tng_chirp.h"

//#define DEBUG_SINE //outputs a sinewave instead of audio from mic
//#define DEBUG_LOOPBACK //outputs all samples to local DAC too for debug on the scope
//#define DEBUG_INTERRUPT

//---------- SETTINGS - CHANGE THESE -----------
#define AUDIO_IN A5     //MICROMPHONE in pin - for best results use an pre-amplified microphone, either with set gain (best) or with auto gain control (you'll get a lot of noise when it's silent)
#define PUSH_TO_TALK 10 //Push to talk key, you push it.. then talk :)
#define RFM69_CS      5 // --> NSS on the RF module
#define RFM69_IRQ     6 // --> DIO0 on the RF module
#define RFM69_RST     9 // --> RST on the RF module
#define LED_PIN      13 // LED pin for debug etc.
//#define FREQUENCY 315.0 //290-340 for the 315MHz module
#define FREQUENCY 433.0 //404-510 for the 433MHz module
//#define FREQUENCY 868.0 //862-890 for the 868MHz module
//#define FREQUENCY 915.0 //890-1020 for the 915MHz module
//------------ SETTINGS - OPTIONAL -------------
uint8_t syncwords[] = { 0x2d, 0x64 }; //this set of two bytes identifies a network (channel), they can be any 2 byte values, but they have to be identical on sender and receiver, or they won't hear eachother
#define SAMPLE_RATE 15000     //at how many Hz the audio from the analog pin will be sampled
#define NUM_BUFFERS 2         //how many buffers we're using to store the audio samples (we need a buffer in case transmission is unreliable)
#define USE_SQUELCH 0         //this is like squelch on a walkie-talkie (or voice detection in TeamSpeak), if the audio is lower than SQUELCH_THRESHOLD then we don't transmit at all. Set this to 1 to enable
#define SQUELCH_THRESHOLD 30  //the audio level under which we don't send (compares to a value from the ADC, so it's from 0-255)
#define PACKET_SIZE 60        //how many bytes each audio packet has, max is 60
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
volatile uint8_t  buffers[NUM_BUFFERS][PACKET_SIZE];
volatile uint16_t sample_index = 0;          // Of the next sample to write
volatile uint8_t  buffer_index = 0;          // Of the bufferbeing filled
volatile bool     buffer_ready[NUM_BUFFERS]; // Set when a buffer is full
uint8_t next_tx_buffer = 0; // These hold the state of the high level transmitter code
uint8_t audioSample = 0;

//AUDIO RX sample buffer
uint8_t buffer_length = 0;
uint8_t buffer[PACKET_SIZE];
uint16_t buffer_play_index = 0;

//state machine
#define STATE_RX    0
#define STATE_TX    1
#define STATE_SFX   2 //playing a sound effect
uint8_t machineState = STATE_SFX; //this is so it plays the sound once when you start it up (so you know audio is working)
uint8_t machineStateNext = STATE_RX;
//SFX variables
unsigned int sfxPosition = 0; //position within a sound-effect, for tracking which sample we're on

//button and interaction vars
unsigned long btnTxTimer = 0;
boolean btnTxDown = false;
unsigned long mls; //millis

RH_RF69 radio(RFM69_CS,RFM69_IRQ); //initialize the RadioHead library


void setup() {
  //Reset the RFM69 module
  pinMode(RFM69_RST, OUTPUT);
  
  resetModule();

  Serial.begin(9600);

  //configure the pins
  pinMode(AUDIO_IN, INPUT);
  pinMode(AUDIO_OUT, OUTPUT);
  pinMode(PUSH_TO_TALK, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //analogWriteResolution(9);   //we're only using 8bit audio so we don't need more

  //initialize the ADC, it will be used to read the audio data
  initADC(AUDIO_IN);

  //initialize radio module
  if (!radio.init()) Serial.println("RFM69 - init failed");
  radio.setModemConfig(RH_RF69::GFSK_Rb250Fd250); ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz
  if (!radio.setFrequency(FREQUENCY)) Serial.println("RFM69 - setFrequency failed"); //CHANGE THIS to your frequency
  
  radio.setTxPower(20); //For RFM69HW and RFM69HCW transmit power is between 14 and 20
  radio.setSyncWords(syncwords, sizeof(syncwords)); //set the network/channel, read above 'syncwords' declaration for more info
  
  //Start an interrupt timer that will call TC5_handler over and over at <sampleRate>Hz
  tcConfigure(SAMPLE_RATE);
  tcStartCounter();
}

void loop() {
  mls = millis();
  handleButtons();
  if(machineState == STATE_RX) {
    if (radio.available()) {
      buffer_length = sizeof(buffer);
      radio.recv(buffer, &buffer_length);
      buffer_play_index = 0; // Trigger the interrupt playing of this buffer from the start
    }
  } else if( machineState == STATE_TX ) {
    transmitAudioPacket(); //if we have enough data send it to the other radios
  }
}

//checks if the next audio sample buffer in line is ready (full) to be sent and sends it via the radio
void transmitAudioPacket() {
   if (buffer_ready[next_tx_buffer]) {
      #if USE_SQUELCH
        // Honour squelch settings
        uint8_t min_value = 255;
        uint8_t max_value = 0;
        uint16_t i;
        for (i = 0; i < PACKET_SIZE; i++)
        {
          if (buffers[next_tx_buffer][i] > max_value)
            max_value = buffers[next_tx_buffer][i];
          if (buffers[next_tx_buffer][i] < min_value)
            min_value = buffers[next_tx_buffer][i];
        }
        if (max_value - min_value > SQUELCH_THRESHOLD)
      #endif
      {
        //radio.waitPacketSent(); // Make sure the previous packet has gone
        radio.send((uint8_t*)buffers[next_tx_buffer], PACKET_SIZE); //send one of the buffers via radio
        buffer_ready[next_tx_buffer] = false; //the buffer we just emptied is no longer ready to be sent
        next_tx_buffer = (next_tx_buffer + 1) % NUM_BUFFERS; //set current buffer to next buffer
      }
   }
}

//Takes an 8 bit value (adcSample) and stores it one of the available audio sample buffers
void storeAudioSample(uint8_t adcSample) {
  buffers[buffer_index][sample_index++] = adcSample;
  if (sample_index >= PACKET_SIZE) {
    sample_index = 0;
    buffer_ready[buffer_index] = true;
    buffer_index = (buffer_index + 1) % NUM_BUFFERS;
    // If the next buffer is still still full, we have an overrun
    //if (buffer_ready[buffer_index]) Serial.println("AUDIO: Buffer Overrun");
  }
}


//interrupt handler, gets called over and over at <SAMPLE_RATE> Hz
void TC5_Handler (void) {
  #ifdef DEBUG_INTERRUPT
  if(ledState == true) {
    *setPin = PinMASK;
  } else {
    *clrPin = PinMASK;
  }
  ledState = !ledState;
  #endif

  if(machineState == STATE_TX) {
    #ifdef DEBUG_SINE
    audioSample = sine256[sinePos];
    sinePos+= 10;
    if(sinePos > 256) sinePos = 0;
    #else
    audioSample = analogReadFast(AUDIO_IN); //get a reading from ADC, this will be an 8bit (1byte) value
    #endif
    
    storeAudioSample(audioSample);
    
    #ifdef DEBUG_LOOPBACK    
    analogWrite(AUDIO_OUT,audioSample);  //OPTIONAL: write the value to the DAC pin so we can debug - disable if not used
    #endif
  } else if( machineState == STATE_RX) {
    if (buffer_play_index < buffer_length) {
      //analogWrite(AUDIO_OUT,filtr.step( (float) buffer[buffer_play_index++] ));
      analogWrite(AUDIO_OUT,buffer[buffer_play_index++]);
    } else {
      analogWrite(AUDIO_OUT,0);
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
  Serial.println("Pressed TX button");
  if(machineState == STATE_RX) {
    Serial.println("STATE: Sending Audio");
    cleanBuffers();
    machineState = STATE_TX;
    //machineState = STATE_SFX; //play the sound
    //machineStateNext = STATE_TX; //then go to Tx mode (the sound player checks this when it's done)
  } else {
    Serial.println("STATE: Listening for audio");
    radio.setModeRx();
    machineState = STATE_RX;
    cleanBuffers();
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

//reset audio buffers before sending to avoid clicks
void cleanBuffers() {
  buffer_index = 0;
  sample_index = 0;
  buffer_play_index = 0;
  next_tx_buffer = 0;
  byte i;
  for (i = 0; i < NUM_BUFFERS; i++) {
    buffer_ready[i] = false;
  }
}

void resetModule(){
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
}

