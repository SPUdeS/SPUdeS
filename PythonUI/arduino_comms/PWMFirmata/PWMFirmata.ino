/*
 * Firmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please click on the following link
 * to open the list of Firmata client libraries in your default browser.
 *
 * https://github.com/firmata/arduino#firmata-client-libraries
 */

/* Supports as many analog inputs and analog PWM outputs as possible.
 *
 * This example code is in the public domain.
 */
#include <Firmata.h>

byte analogPin = 0;
const byte pwm0pin = 11;     // PB5, OC1A
const byte pwm1pin = 12;     // PB6, OC1B
const byte pwm2pin = 5;      // PE5, OC3A
const byte pwm3pin = 2;      // PE4, OC3B
const byte pwm4pin = 3;      // PE5, OC3C
const byte pwm5pin = 6;      // PH3, OC4A
const byte pwm6pin = 7;      // PH4, OC4B
const byte pwm7pin = 8;      // PH5, OC4C
const byte pwm8pin = 46;     // PL3, OC5A
const byte pwm9pin = 45;    // PL4, OC5B
const byte pwm10pin = 44;    // PL5, OC5C

const uint16_t MAX_PWM_VALUE = 65535U;

//----------------------------------------------------------------------------------------------------
// Setup timers for 16 bit PWM
void initPWM() {
  noInterrupts();

  TCCR3A = 1 << WGM31 | 1 << COM3A1 | 1 << COM3B1 | 1 << COM3C1; // set on top, clear OC on compare match
  TCCR3B = 1 << CS30  | 1 << WGM32 | 1 << WGM33;   // clk/1, mode 14 fast PWM
  TCCR4A = 1 << WGM41 | 1 << COM4A1 | 1 << COM4B1 | 1 << COM4C1; // set on top, clear OC on compare match
  TCCR4B = 1 << CS40  | 1 << WGM42 | 1 << WGM43;   // clk/1, mode 14 fast PWM
  TCCR5A = 1 << WGM51 | 1 << COM5A1 | 1 << COM5B1 | 1 << COM5C1; // set on top, clear OC on compare match
  TCCR5B = 1 << CS50  | 1 << WGM52 | 1 << WGM53;   // clk/1, mode 14 fast PWM
  ICR3 = MAX_PWM_VALUE;
  ICR4 = MAX_PWM_VALUE;
  ICR5 = MAX_PWM_VALUE;

  interrupts();
  pinMode(pwm0pin, OUTPUT);
  pinMode(pwm1pin, OUTPUT);
  pinMode(pwm2pin, OUTPUT);
  pinMode(pwm3pin, OUTPUT);
  pinMode(pwm4pin, OUTPUT);
  pinMode(pwm5pin, OUTPUT);
  pinMode(pwm6pin, OUTPUT);
  pinMode(pwm7pin, OUTPUT);
  pinMode(pwm8pin, OUTPUT);
  pinMode(pwm9pin, OUTPUT);
  pinMode(pwm10pin, OUTPUT);

}

//----------------------------------------------------------------------------------------------------
// Set a 16 bit PWM value for a channel
void setPWM(byte no, uint16_t pwm) {
  noInterrupts();
  switch (no) {
    case 0 :
      OCR1A = pwm;
      break;
    case 1 :
      OCR1B = pwm;
      break;
    case 2 :
      OCR3A = pwm;
      break;
    case 3 :
      OCR3B = pwm;
      break;
    case 4 :
      OCR3C = pwm;
      break;
    case 5 :
      OCR4A = pwm;
      break;
    case 6 :
      OCR4B = pwm;
      break;
    case 7 :
      OCR4C = pwm;
      break;
    case 8 :
      OCR5A = pwm;
      break;
    case 9 :
      OCR5B = pwm;
      break;
    case 10 :
      OCR5C = pwm;

  }
  interrupts();
}

void analogWriteCallback(byte pin, int value)
{
  if (IS_PIN_PWM(pin)) {
    pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
    //analogWrite(PIN_TO_PWM(pin), value);
    setPWM(PIN_TO_PWM(pin), map(value, 0, 180, 0,MAX_PWM_VALUE));
  }
}

void setup()
{
  initPWM();
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.begin(57600);

}

void loop()
{
  while (Firmata.available()) {
    Firmata.processInput();
  }
  // do one analogRead per loop, so if PC is sending a lot of
  // analog write messages, we will only delay 1 analogRead
  Firmata.sendAnalog(analogPin, analogRead(analogPin));
  analogPin = analogPin + 1;
  if (analogPin >= TOTAL_ANALOG_PINS) analogPin = 0;
}