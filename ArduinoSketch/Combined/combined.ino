#include <SPI.h>
#include <RH_RF69.h>
#include "adc.h"
#include "timer.h"
#include "tng_chirp.h"

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
#define SAMPLE_RATE 11025     //at how many Hz the audio from the analog pin will be sampled
#define NUM_BUFFERS 2         //how many buffers we're using to store the audio samples (we need a buffer in case transmission is unreliable)
#define USE_SQUELCH 0         //this is like squelch on a walkie-talkie (or voice detection in TeamSpeak), if the audio is lower than SQUELCH_THRESHOLD then we don't transmit at all. Set this to 1 to enable
#define SQUELCH_THRESHOLD 30  //the audio level under which we don't send (compares to a value from the ADC, so it's from 0-255)
#define PACKET_SIZE 60        //how many bytes each audio packet has, over 60 doesn't seem to work reliably. no real reason to change unless messing with sample rates
//-------------- END OF SETTINGS ---------------

//fast digital reads/writes
volatile uint32_t *setPin = &PORT->Group[g_APinDescription[LED_PIN].ulPort].OUTSET.reg;
volatile uint32_t *clrPin = &PORT->Group[g_APinDescription[LED_PIN].ulPort].OUTCLR.reg;
const uint32_t  PinMASK = (1ul << g_APinDescription[LED_PIN].ulPin);
bool ledState = false;

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
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  Serial.begin(9600);

  //configure the pins
  pinMode(AUDIO_IN, INPUT);
  pinMode(AUDIO_OUT, OUTPUT);
  pinMode(PUSH_TO_TALK, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(8);   //we're only using 8bit audio so we don't need more

  //initialize the ADC, it will be used to read the audio data
  initADC(AUDIO_IN);

  //initialize radio module
  if (!radio.init()) Serial.println("init failed");
  if (!radio.setFrequency(FREQUENCY)) Serial.println("setFrequency failed"); //CHANGE THIS to your frequency
  //radio.setModemConfig(RH_RF69::GFSK_Rb250Fd250); ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz
  radio.setTxPower(15); //For RFM69HW and RFM69HCW transmit power is between 14 and 20
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
        radio.waitPacketSent(); // Make sure the previous packet has gone
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
    if (buffer_ready[buffer_index]) Serial.println("Overrun");
  }
}

//interrupt handler, gets called over and over at <SAMPLE_RATE> Hz
void TC5_Handler (void) {
  if(machineState == STATE_TX) {
    audioSample = analogReadFast(AUDIO_IN); //get a reading from ADC, this will be an 8bit (1byte) value
    storeAudioSample(audioSample); //Store it in one of the buffers
    //analogWrite(AUDIO_OUT,audioSample);  //OPTIONAL: write the value to the DAC pin so we can debug - disable if not used
  } else if( machineState == STATE_RX) {
    if (buffer_play_index < buffer_length) {
      analogWrite(AUDIO_OUT,buffer[buffer_play_index++]);
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
  Serial.println("TXBTN");
  if(machineState == STATE_RX) {
    Serial.println("STATE TX");
    cleanBuffers();
    //machineState = STATE_TX;
    machineState = STATE_SFX; //play the sound
    machineStateNext = STATE_TX; //then go to Tx mode (the sound player checks this when it's done)
  } else {
    Serial.println("STATE RX");
    machineState = STATE_RX;
  }
  
}

//each time it runs it writes an audio sample from an audio file array to the DAC
void playSound() {
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
  //TODO: might have to set bufferReady to false for each buffer
}

