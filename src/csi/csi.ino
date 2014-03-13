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
 * These sources were particularly helpful with figuring out the bare-bones
 * arduino features that were needed to get this project working smoothly.
 *
 * Glenn Sweeney - ADC interupt 
 * http://www.glennsweeney.com/tutorials/interrupt-driven-analog-conversion-with-an-atmega328p
 *
 * amandaghassaei - Timer interupt
 * http://www.instructables.com/id/Arduino-Timer-Interrupts/
 *
 */

#define DEBUG

// Analog pin defines
#define RIGHTLEFT 0
#define TOPBOTTOM 1
#define ACCEL 2

// Digital pin defines
#define RX 0 // Reserved
#define TX 1 // Reserved
#define L_E1 2 // Interrupt
#define T_E1 3 // Interrupt
#define NE 4
#define NW 5
#define SE 6
#define SW 7
// NOT_DEFINED 8
#define LR_SP 9  // pwm
#define B_SP 10 // pwm
#define T_SP 11 // pwm
#define TB_DIR 12
#define B_EN 13

// Constant defines
#define HIGH_THRESHOLD 16000
#define LOW_THRESHOLD 14000
#define MAX_DEFECT_POINTS 1000
#define SPEED 192
#define STALL_TORQUE 128
#define LR_OFFSET 64
#define OFFSET 10

#define LEFT LOW
#define RIGHT HIGH

volatile int state = 0;
volatile int distance = 0;
volatile int defect_array_index;

volatile int adc_pin = 0;
volatile int adc_val[6] = {0,0,0,0,0,0};

volatile int defect_points[8][MAX_DEFECT_POINTS];

void setup () {
    // PWM
    TCNT0  = 0; OCR0A = 15; TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (1 << CS00); TIMSK0 |= (1 << OCIE0A);
    
    // ADC
    adc_pin = 0;
    ADMUX |= B01000000; ADMUX &= B11010000; ADCSRA |= B11101111;
    
    // Defect sensing pins
    pinMode(NE, INPUT); pinMode(NW, INPUT);
    pinMode(SE, INPUT); pinMode(SW, INPUT);

    // Motors
    pinMode(LR_SP , OUTPUT);
    pinMode(B_EN, OUTPUT); pinMode(TB_DIR, OUTPUT);
    pinMode(T_SP, OUTPUT); pinMode(B_SP, OUTPUT);

    // Encoders
    pinMode(L_E1, INPUT); pinMode(T_E1, INPUT);
    attachInterrupt(0, enUD, CHANGE);
    attachInterrupt(1, enLR, CHANGE);

    state = 0;

    sei();
    Serial.begin(9600);
}

void loop() {
    // TODO: Later. Agregate defect data after up/down pass.
#ifdef DEBUG
    delay(500);
    if (digitalRead(NE) == HIGH) printf("NE Defect\n");
    if (digitalRead(NW) == HIGH) printf("NW Defect\n");
    if (digitalRead(SE) == HIGH) printf("SE Defect\n");
    if (digitalRead(SW) == HIGH) printf("SW Defect\n");
#endif
}

void enUD() {
    // TODO: Later. Add sanity check.

    // Take note of defects.
    if ((defect_array_index < MAX_DEFECT_POINTS) && (stage % 3 == 1)) {
        defect_points[NE][defect_array_index] = digitalRead(NE);
        defect_points[NW][defect_array_index] = digitalRead(NW);
        defect_points[SE][defect_array_index] = digitalRead(SE);
        defect_points[SW][defect_array_index] = digitalRead(SW);
    }
}

void enLR() {
    if (state % 3 == 0) {
        distance += 1;
        if (distance > ((3 * WIDTH / 2) + (WIDTH * state))) {
            state += 1;
            digitalWrite(B_EN, LOW);
            analogWrite(T_SP, 0);
            analogWrite(B_SP, 0);
            if (state % 6 == 1) analogWrite(LR_SP, STALL_TORQUE - LR_OFFSET);
            else                analogWrite(LR_SP, STALL_TORQUE + LR_OFFSET);
        }
    }
}

ISR(TIMER0_COMPA_vect) {
    // TODO: Later. Add tilt correction.
    // TODO: Add height correction while moving along top and bottom.
    // TODO: make corrections use PID
}

ISR(ADC_vect)
{
    int analogVal = (ADCL | (ADCH << 8)) << 4;
    int curr_adc_pin = adc_pin;
    adc_pin = (adc_pin + 1) % 3;

    // Start the next ADC
    ADMUX &= B11110000; ADMUX |= adc_pin; ADCSRA |= B01000000;
    
    adc_val[curr_adc_pin] += (analogVal >> 2) - (adc_val[curr_adc_pin] >> 2);

    // Update things that use ADC values
    
    // Moving up/down
    if ((curr_adc_pin == TOPBOTTOM) && 
           (((state % 6 == 1) && (adc_val[curr_adc_pin] <  LOW_THRESHOLD)) ||
            ((state % 6 == 4) && (adc_val[curr_adc_pin] > HIGH_THRESHOLD)))) {
        state++;
        analogWrite(LR_SP, STALL_TORQUE);
        analogWrite(B_SP, SPEED);
        analogWrite(T_SP, SPEED);
        digitalWrite(TB_DIR, LEFT);
        digitalWrite(B_EN, HIGH);
    }
    // Moving Left
    else if (((curr_adc_pin == LEFTRIGHT) && (state % 3 == 2)) &&
             (adc_val[curr_adc_pin] < LOW_THRESHOLD)) {
        state++;
        digitalWrite(TB_DIR, RIGHT);
    }
}

