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

// Analog pin defines
#define RIGHTLEFT 4
#define TOPBOTTOM 5
#define NE 0
#define NW 1
#define SE 2
#define SW 3

// Digital pin defines
#define ACCEL_L 4
#define ACCEL_R 5
#define L_E1 2
#define T_E1 3
#define LR_EN 7
#define L_SP 6  // pwm
#define LR_DIR 8
#define R_SP 9  // pwm
#define TB_EN 13
#define T_SP 11 // pwm
#define TB_DIR 12
#define B_SP 10 // pwm

// Constant defines
#define MAX_DEFECT_POINTS 1000
#define SPEED 192
#define OFFSET 10

#define TOP 0
#define BOTTOM 1
#define LEFT 2
#define RIGHT 3

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

volatile int defect_points[4][MAX_DEFECT_POINTS];

void setup () {
    // PWM
    TCNT0  = 0; OCR0A = 15; TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (1 << CS00); TIMSK0 |= (1 << OCIE0A);
    
    // ADC
    adc_pin = 0;
    ADMUX |= B01000000; ADMUX &= B11010000; ADCSRA |= B11101111;
    
    // Motors
    pinMode(L_EN, OUTPUT); pinMode(L_SP, OUTPUT);
    pinMode(R_EN, OUTPUT); pinMode(R_SP, OUTPUT);
    pinMode(T_EN, OUTPUT); pinMode(T_SP, OUTPUT);
    pinMode(B_EN, OUTPUT); pinMode(B_SP, OUTPUT);
    pinMode(L_E1, OUTPUT); pinMode(T_E1, OUTPUT);

    attachInterrupt(0, enUD, CHANGE);
    attachInterrupt(1, enLR, CHANGE);

    state = 0;

    sei();
    Serial.begin(9600);
}

void loop() {
    // TODO: agregate defect data after up/down pass
}

void enUD() {
    // TODO: Only needed as a sanity check, will implement later
}

void enLR() {
    if (state % 3 == 0) {
        distance += 1;
        if (distance > ((3 * WIDTH / 2) + (WIDTH * state))) {
            state += 1;
            // TODO: make motors go in the right direction
        }
    }
}

ISR(TIMER0_COMPA_vect) {
    // TODO: periodic tasks go here. Called at 1kHz.
}

ISR(ADC_vect)
{
    int analogVal = (ADCL | (ADCH << 8)) << 4;
    int curr_adc_pin = adc_pin;
    adc_pin = (adc_pin + 1) % 6;

    // Start the next ADC
    ADMUX &= B11110000; ADMUX |= adc_pin; ADCSRA |= B01000000;
    
    adc_val[curr_adc_pin] += (analogVal >> 2) - (adc_val[curr_adc_pin] >> 2);

    // Update things that use ADC values
    switch (curr_adc_pin) {
        case TOPBOTTOM:
            switch (state) {
                case 0: // Moving right at top
                    // TODO: TB ADC
                break;
                case 1: // Moving down
                    // TODO: TB ADC
                break;
                case 2: // Moving left at bottom
                    // TODO: TB ADC
                break;
                case 3: // Moving right at bottom
                    // TODO: TB ADC
                break;
                case 4: // Moving up
                    // TODO: TB ADC
                break;
                case 5: // Moving left at top
                    // TODO: TB ADC
                break;
            }
            break;
        case LEFTRIGHT:
            switch (state) {
                case 0: // Moving right at top
                    // TODO: LR ADC
                break;
                case 1: // Moving down
                    // TODO: LR ADC
                break;
                case 2: // Moving left at bottom
                    // TODO: LR ADC
                break;
                case 3: // Moving right at bottom
                    // TODO: LR ADC
                break;
                case 4: // Moving up
                    // TODO: LR ADC
                break;
                case 5: // Moving left at top
                    // TODO: LR ADC
                break;
            }
            break;
        default:
            if (state % 3 == 1) {
                if (defect_array_index < MAX_DEFECT_POINTS) {
                    defect_points[curr_adc_pin][defect_array_index] = analogVal;
                }
                if (curr_adc_pin == 3)
                    defect_array_index++;
            }
    }
}

