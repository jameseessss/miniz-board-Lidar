// class implementation for SAMD21 PWM

// Generate 20kHz PWM with SAMD21
// this file uses SAMD21 chip functions and interacts with low-level registers
// the documentation on these function is very obscure, but once you
// understand it it's quite straightforward, some helpful sources:
// official datasheet
// https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/ also
// check https://forum.arduino.cc/t/changing-arduino-zero-pwm-frequency/334231/3
// about CC and WO[n] mapping
// https://forum.arduino.cc/t/samd21-wo-and-cc-register-mapping/852449/2

// To use the PWM function on SAMD21,
// First setup a clock source
// the 48MHz chip clock is connected to GCLK4 (Generic Clock Controller)
// which is then connected to TCC0 (Timer/Counter for Control)
// You can apply clock divider to GCLK, or prescaler to TCC to control the
// counter frequency Details can be found in relevant code, currently we use
// 20kHz. There are three TCC, TCC0/1/2. TCC3 is only available in later model
// SAMD21 and not available on arduino nano iot 33 board

// Second we select a PWM mode, we use single slope PWm but can also use dual
// slope note that dual slope would halve the frequency

// Third we set the PER (TOP) register for TCC0, this is the value that TCC0
// counts to before resetting, this can be used to fine tune frequency

// Fourth we configure the individual outputs from TCC0, this is denoted as
// TCC0/WO[n] where n is a number between 0 to 7 TCC0 only has 4 independent
// channels, WO[0]-WO[3], the remaining channel 4-7 repeats the first 4 channels
// It's also possible to have channel 4-7 invert the signal of 0-3
// it is designed this way because on an H bridge, two inputs are always
// inverted we DON'T invert 4-7, so for us, 0-3 is the same as 4-7 we can't set
// channel 4-7 directly, if we want to output to channel 5 we set channel 1. We
// need to do this because due to the pin layouts, sometimes the pin we have
// access to can only be connected to channel 4-7

// Fifth we need to cnnect a pin (e.g. PB11 ) to an PWM output (e.g. TCC0/WO[5])
// this contains two parts, first enable the port multiplexer for that pin
// and second connect the pin to a desired function. A pin in multiplexer mode
// can be connected to different functions (A-G), you can find the table in the
// SAMD21 datasheet. TCC functions are usually function E or F
// One last thing is that the number of registers for function connection
// is half the number of pins, and one register controls two pins.
// for example in PORTB, PB21 and PB20 are both controller by PMUX[10]
// if you wish to connect an odd pin (e.g. PB21 to function 'F'),
//  use PORT_PMUX_PMUX'O'_'F'
// if you wish to connect an even pin (e.g. PB20 to function 'E'),
//  use PORT_PMUX_PMUX'E'_'E'

// to find the PORT and pin you can use this
// PORT: g_APinDescription[pinNumber].ulPort
// PIN: g_APinDescription[pinNumber].ulPin

// here's an example for step 3-5, step 1-2 are in pwmSetup() and are universal

//  // Arduino D3 (Nano IoT) PB11 TCC0/WO[5] function F
//  TCC0->CC[5 % 4].reg = int(duty_cycle * (pwm_period-1));
//  // not entirely necessary
//  PORT->Group[PORTB].DIRSET.reg |= PORT_PB11;      // Set pin as output
//  PORT->Group[PORTB].OUTCLR.reg |= PORT_PB11;      // Set pin to low
//  // Enable the port multiplexer for PB11
//  PORT->Group[PORTB].PINCFG[11].bit.PMUXEN = 1;
//  // alternative way to do the above
//  //PORT->Group[PORTB].PINCFG[11].reg |= PORT_PINCFG_PMUXEN;

//  // Connect TCC0 timer to PB11. Function F is TCC0/WO[5] for PB11.
//  // Odd pin num (pin_no = 2*n + 1): use PMUXO
//  // Even pin num (pin_no = 2*n): use PMUXE
//  // PMUX[x]: here x = pin_no / 2
//  // for more detail check link in top of file
//  PORT->Group[PORTB].PMUX[11 >> 1].reg |= PORT_PMUX_PMUXO_F;

#include <Arduino.h>
#ifndef PWM_H
#define PWM_H
// valid pin: 5,6,9,10
class PWM {
private:
  // const static uint32_t pwm_period = 2400 - 1;
  const static uint32_t pwm_period = 24000 - 1;

public:
  static void setup() {
    // Number to count to with PWM (TOP value). Frequency can be calculated by
    // freq = GCLK4_freq / (TCC0_prescaler * (1 + TOP_value))
    /// TOP = (clock freq)/(desired freq)/prescaler - 1
    // TOP = 2400-1, freq = 20k
    // TOP = 48-1, freq = 1M
    // resolution = log(TOP)/log(2)

    // Because we are using TCC0, limit period to 24 bits
    // pwm_period = ( pwm_period < 0x00ffffff ) ? pwm_period : 0x00ffffff;

    // Enable and configure generic clock generator 4
    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |         // Improve duty cycle
                        GCLK_GENCTRL_GENEN |       // Enable generic clock gen
                        GCLK_GENCTRL_SRC_DFLL48M | // Select 48MHz as source
                        GCLK_GENCTRL_ID(4);        // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY)
      ; // Wait for synchronization

    // Set clock divider of 1 to generic clock generator 4
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | // Divide 48 MHz by 1
                       GCLK_GENDIV_ID(4);   // Apply to GCLK4 4
    while (GCLK->STATUS.bit.SYNCBUSY)
      ; // Wait for synchronization

    // Enable GCLK4 and connect it to TCC0 and TCC1
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable generic clock
                        GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4
                        GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0/1
    while (GCLK->STATUS.bit.SYNCBUSY)
      ; // Wait for synchronization

    // Divide counter by 1 giving 48 MHz (20.83 ns) on each TCC0 tick
    TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

    // Use "Normal PWM" (single-slope PWM): count up to PER, match on CC[n]
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM; // Select NPWM as waveform

    // REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output
    // polarity on all TCC0 outputs
    //                TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on
    //                TCC0
    while (TCC0->SYNCBUSY.bit.WAVE)
      ; // Wait for synchronization

    // Set the period (the number to count to (TOP) before resetting timer)
    TCC0->PER.reg = pwm_period;
    while (TCC0->SYNCBUSY.bit.PER)
      ;
  }

  static void set(int pinNumber, float duty_cycle) {
    int reg_val = int(duty_cycle * (pwm_period - 1));
    reg_val = (reg_val < 0) ? 0 : reg_val;
    reg_val = (reg_val > pwm_period - 1) ? pwm_period - 1 : reg_val;

    switch (pinNumber) {
    case 3:
      // PB11 TCC0/WO[5] function F
      TCC0->CC[5 % 4].reg = reg_val;
      PORT->Group[PORTB].DIRSET.reg = PORT_PB11; // Set pin as output
      PORT->Group[PORTB].OUTCLR.reg = PORT_PB11; // Set pin to low
      // Enable the port multiplexer for PB11
      PORT->Group[PORTB].PINCFG[11].bit.PMUXEN = 1;
      // identical to above
      // PORT->Group[PORTB].PINCFG[11].reg |= PORT_PINCFG_PMUXEN;

      // Connect TCC0 timer to PB11. Function F is TCC0/WO[5] for PB11.
      // Odd pin num (pin_no = 2*n + 1): use PMUXO
      // Even pin num (pin_no = 2*n): use PMUXE
      // PMUX[x]: here x = pin_no / 2
      // for more detail check link in top of file
      PORT->Group[PORTB].PMUX[11 >> 1].reg |= PORT_PMUX_PMUXO_F;
      break;

    case 2:
      // PB10 TCC0/WO[4] function F
      TCC0->CC[4 % 4].reg = reg_val;
      // PORT->Group[PORTB].DIRSET.reg = PORT_PB10;      // Set pin as output
      // PORT->Group[PORTB].OUTCLR.reg = PORT_PB10;      // Set pin to low
      PORT->Group[PORTB].PINCFG[10].bit.PMUXEN = 1;
      PORT->Group[PORTB].PMUX[10 >> 1].reg |= PORT_PMUX_PMUXE_F;
      break;

    case 5:
      // PA05 TCC0/WO[1] function E
      TCC0->CC[1].reg = reg_val;
      PORT->Group[PORTA].DIRSET.reg = PORT_PA05; // Set pin as output
      PORT->Group[PORTA].OUTCLR.reg = PORT_PA05; // Set pin to low
      PORT->Group[PORTA].PINCFG[5].bit.PMUXEN = 1;
      PORT->Group[PORTA].PMUX[5 >> 1].reg |= PORT_PMUX_PMUXO_E;
      break;

    case 6:
      // PA04 TCC0/WO[0] function E
      TCC0->CC[0].reg = reg_val;
      PORT->Group[PORTA].DIRSET.reg = PORT_PA04; // Set pin as output
      PORT->Group[PORTA].OUTCLR.reg = PORT_PA04; // Set pin to low
      PORT->Group[PORTA].PINCFG[4].bit.PMUXEN = 1;
      PORT->Group[PORTA].PMUX[4 >> 1].reg |= PORT_PMUX_PMUXE_E;
      break;

    case 9:
      // PA20 TCC0/WO[6] -> %4=2 function F
      TCC0->CC[2].reg = reg_val;
      PORT->Group[PORTA].DIRSET.reg = PORT_PA05; // Set pin as output
      PORT->Group[PORTA].OUTCLR.reg = PORT_PA05; // Set pin to low
      PORT->Group[PORTA].PINCFG[20].bit.PMUXEN = 1;
      PORT->Group[PORTA].PMUX[20 >> 1].reg |= PORT_PMUX_PMUXE_F;
      break;

    case 10:
      // PA21 TCC0/WO[7] %4 = 3function F
      TCC0->CC[3].reg = reg_val;
      PORT->Group[PORTA].DIRSET.reg = PORT_PA04; // Set pin as output
      PORT->Group[PORTA].OUTCLR.reg = PORT_PA04; // Set pin to low
      PORT->Group[PORTA].PINCFG[21].bit.PMUXEN = 1;
      PORT->Group[PORTA].PMUX[21 >> 1].reg |= PORT_PMUX_PMUXO_F;
      break;

    default:
      Serial.println("Unsupported pin");
    }

    // Enable output (start PWM)
    TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
    while (TCC0->SYNCBUSY.bit.ENABLE)
      ; // Wait for synchronization
  }
};

#endif
