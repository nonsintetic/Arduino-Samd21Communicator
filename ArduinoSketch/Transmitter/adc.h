/* 
 * This file contains some functions to make analog reading much faster on SAM D21 boards like the Adafruit Feather M0 
 * see https://forum.arduino.cc/index.php?topic=341345.0 for analysis of this method
 */


// ADC - Wait for synchronization of registers between the clock domains
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

//analogRead() alternative - uses less conservative register settings to make the ADC much much faster
//without this we cannot read the audio data fast enough
uint8_t analogReadFast(uint8_t analogPin) {
  ADCsync();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[analogPin].ulADCChannelNumber; // Selection for the positive ADC input
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
  ADCsync();
  ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Wait till conversion done
  ADCsync();
  uint8_t valueRead = ADC->RESULT.reg;
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable the ADC 
  ADCsync();
  ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
  return valueRead;
}
void initADC(uint8_t analogPin){
  ADCsync();
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain select as 1X
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  // Set sample length and averaging
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
  ADCsync();
  ADC->SAMPCTRL.reg = 0x0A;  ; //sample length in 1/2 CLK_ADC cycles Default is 3F
  //Control B register
  int16_t ctrlb = 0x530;       // 403 Control register B hibyte = prescale, lobyte is resolution and mode 
  ADCsync();
  ADC->CTRLB.reg =  ctrlb     ; 
  analogReadFast(analogPin);  //Discard first conversion after setup as ref changed
}
