/*
 * Copyright (C) 2014 John Greth (jgreth@cmu.edu)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *
 * CREDIT:
 * Glenn Sweeney - ADC interupt code at 
 * http://www.glennsweeney.com/tutorials/interrupt-driven-analog-conversion-with-an-atmega328p
 *
 *
 */

#define RIGHTLEFT 0
#define TOPBOTTOM 1
#define NE 2
#define NW 3
#define SE 4
#define SW 5

#define MAX_DEFECT_POINTS 1000

volatile int state = 0;
volatile int distance = 0;
volatile int defect_array_index;

volatile int at_left = 0;
volatile int at_right = 0;
volatile int at_top = 0;
volatile int at_bottom = 0;

volatile int NE_defect = 0;
volatile int NW_defect = 0;
volatile int SE_defect = 0;
volatile int SW_defect = 0;

volatile int adc_pin = 0;
volatile int adc_val[6] = {0,0,0,0,0,0};

const char[NUM_MESSAGES][] = {
    "\nCurrent Stage: ",
    "\nNot at Top"};

void setup () {
    setup_adc();
    setup_motors();

    state = 0;

    Serial.begin(9600);
}

void loop() {
  delay(500);
  Serial.print(";\nval0: ");
  Serial.print(adc_val[0], DEC);
  Serial.print(";  val1: ");
  Serial.print(adc_val[1], DEC);
  Serial.print(";  val2: ");
  Serial.print(adc_val[2], DEC);
  Serial.print(";  val3: ");
  Serial.print(adc_val[3], DEC);
  /*  if (adc_val[LEFT] > (1500 + adc_val[RIGHT])) {
        Serial.print("Left Edge\n");
    }
    if (adc_val[RIGHT] > (1500 + adc_val[LEFT])) {
        Serial.print("Right Edge\n");
    }
    if (adc_val[BOTTOM] > (1500 + adc_val[TOP])) {
        Serial.print("Bottom Edge\n");
    }
    if (adc_val[TOP] > (1500 + adc_val[BOTTOM])) {
        Serial.print("Top Edge\n");
    }
    */
}

ISR(ADC_vect)
{
    int analogVal = (ADCL | (ADCH << 8)) << 4;
    int curr_adc_pin = adc_pin;
    adc_pin = (adc_pin + 1) % 6;

    // Start the next ADC
    ADMUX &= B11110000;
    ADMUX |= adc_pin;
    ADCSRA |= B01000000;
    
    adc_val[curr_adc_pin] += (analogVal >> 2) - (adc_val[curr_adc_pin] >> 2);

    // Update things that use ADC values
    switch (curr_adc_pin) {
        case TOPBOTTOM:
            at_top    = (adc_val[TOPBOTTOM] > upper_threshold);
            at_bottom = (adc_val[TOPBOTTOM] < lower_threshold);
            break;
        case 
}

void setup_motors() {
    distance = 0;

    pinMode(L_EN, OUTPUT);
    pinMode(L_SP, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(R_SP, OUTPUT);
    pinMode(T_EN, OUTPUT);
    pinMode(T_SP, OUTPUT);
    pinMode(B_EN, OUTPUT);
    pinMode(B_SP, OUTPUT);

    pinMode(L_E1, OUTPUT);
    pinMode(R_E1, OUTPUT);
    pinMode(T_E1, OUTPUT);
    pinMode(B_E1, OUTPUT);

}

void setup_adc() {
 
    adc_pin = 0;

  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B01000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B10101111;
  
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();
  
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;
}

