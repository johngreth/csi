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
#define LEFTRIGHT 1
#define TOPBOTTOM 0
#define ACCEL 2

// Digital pin defines
#define RX 0 // Reserved
#define TX 1 // Reserved
#define L_E1 2 // Interrupt
#define T_E1 3 // Interrupt
#define TB1 5
#define TB2 6
#define L2 9 // 4
#define R2 10 // 7
#define LR1 11 // 8
#define NE 9 
#define SW 10 
#define NW 11
#define SE 12

#define LEFT 0
#define RIGHT 1
#define UP 2
#define DOWN 3

// Constant defines
#define COLOR_SCALAR 10

#define HIGH_THRESHOLD_LR 10000L
#define LOW_THRESHOLD_LR 6000L
#define HIGH_THRESHOLD_TB 22000L
#define LOW_THRESHOLD_TB 20000L

#define MAX_DEFECT_POINTS 1000

#define HIGHSPEED 64
#define LOWSPEED 25

#define STALL_TORQUE 45

#define UPHIGHSPEED 220
#define UPLOWSPEED 170

#define DOWNHIGHSPEED2 255
#define DOWNLOWSPEED2 255
#define DOWNHIGHSPEED1 150
#define DOWNLOWSPEED1 150

#define BUFFER 100
#define WIDTH 450
#define UPHEIGHT 4900
#define DOWNHEIGHT 3500
#define SLACK 20

#define PWMRES 15
#define PWMFREQ 2

volatile int state = 0;
volatile int curr_width;
volatile int last_state_LR = 0;
volatile int last_state_TB = 0;
volatile int distance_LR = 0;
volatile int distance_TB = 0;
volatile int defect_array_index = 0;
volatile int next_dist;

volatile int adc_pin = 0;
volatile unsigned int widths[6];
volatile unsigned long adc_val[6] = {0,0,0,0,0,0};
volatile unsigned long raw_val[6] = {0,0,0,0,0,0};
volatile unsigned long min_val[6] = {0,0,0,0,0,0};
volatile unsigned long max_val[6] = {0,0,0,0,0,0};

volatile int defect_points[14][MAX_DEFECT_POINTS];

volatile int pwm_out[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile int pwmCtr = 0;

void en_TB() { distance_TB++; }

void en_LR() { distance_LR++; }

void setup () {
    cli();
    Serial.begin(9600);
    // Timer
    //TCNT1  = 0; OCR1A = (PWMFREQ * 1) - 1; TCCR1A = (1 << WGM12);
    //TCCR1B = (1 << CS10); TIMSK1 |= (1 << OCIE1A);
    //TCNT2  = 0; OCR2A = 6                     ; TCCR2A = (1 << WGM21);
    //TCCR2B = (1 << CS22); TIMSK2 |= (1 << OCIE2A);
    
    // ADC
    adc_pin = 0;
    ADMUX |= B01000000; ADMUX &= B11010000; ADCSRA |= B11101111;
    
    // Defect sensing pins
    // pinMode(NE, INPUT); pinMode(NW, INPUT);
    // pinMode(SE, INPUT); pinMode(SW, INPUT);

    // Motors
    pinMode(LR1, OUTPUT); pinMode(R2, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(TB1, OUTPUT); pinMode(TB2, OUTPUT);
    
    pinMode(13, OUTPUT);
    
    pwm_out[L2] = 255;
    pwm_out[R2] = 255;
    pwm_out[LR1] = 240;
    pwm_out[TB1] = 0;
    pwm_out[TB2] = 80;
        analogWrite(TB1, pwm_out[TB1]);
        analogWrite(TB2, pwm_out[TB2]);
        analogWrite(R2, pwm_out[R2]);
        analogWrite(L2, pwm_out[L2]);
        analogWrite(LR1, pwm_out[LR1]);
    distance_LR = 0;
    distance_TB = 0;
    
    state = 0;
    
    widths[0] = 1*WIDTH/2;
    widths[1] = 3*WIDTH/2;
    widths[2] = 5*WIDTH/2;
    widths[3] = 7*WIDTH/2;
    widths[4] = 9*WIDTH/2;
    widths[5] = 11*WIDTH/2;
    
    curr_width = widths[(state/18)%3];
    next_dist = curr_width - BUFFER - BUFFER;

    // Encoders
    attachInterrupt(1, en_TB, RISING);
    attachInterrupt(0, en_LR, RISING);

    sei();
}

void loop() {
    int sta = (state % 18);
    int st = sta / 3;
    int distance = distance_TB;
    if (st % 3 == 1) distance = distance_LR;
    
    if (    ((state % 9  ==  8) && (adc_val[LEFTRIGHT] > HIGH_THRESHOLD_LR)) ||
            ((state % 18 ==  5) && (adc_val[TOPBOTTOM] <  LOW_THRESHOLD_TB)) ||
            ((state % 18 == 14) && (adc_val[TOPBOTTOM] > HIGH_THRESHOLD_TB)))
        next_dist = 0;
        
    if (distance > next_dist) {
        state++;
        sta = (state % 18);
        st = sta / 3;
        if (state % 9 == 0) curr_width = widths[(state/9)%6];
        
        if (state % 3 == 0) { distance_TB = 0; distance_LR = 0; }
        
        pwm_out[L2] = 255;
        pwm_out[R2] = 255;
        pwm_out[LR1] = 240;
        pwm_out[TB1] = 0;
        pwm_out[TB2] = 0;
        
        switch (st) {
            case 0: case 3: 
                pwm_out[TB2] = 80;
            break;
            case 2: case 5: 
                pwm_out[TB1] = 60;
            break;
            case 1:
                pwm_out[LR1] = 255;
                pwm_out[L2] = 240;
                pwm_out[R2] = 240;
                break;
            case 4:
                pwm_out[LR1] = 100;
                break;
        }
        
        analogWrite(TB1, pwm_out[TB1]);
        analogWrite(TB2, pwm_out[TB2]);
        analogWrite(R2, pwm_out[R2]);
        analogWrite(L2, pwm_out[L2]);
        analogWrite(LR1, pwm_out[LR1]);
        
        next_dist = curr_width;
        if (st == 1) next_dist = DOWNHEIGHT;
        if (st == 4) next_dist = UPHEIGHT;
        if (state % 3 == 0) next_dist -= BUFFER + BUFFER;
        if (state % 3 == 1) next_dist -= BUFFER;
        if ((state % 9 == 5) && (state % 9 == 8)) next_dist += SLACK;
    }
    
    //if (state % 9 == 3) {
        delayMicroseconds(25000); 
        digitalWrite(13, HIGH);
        delayMicroseconds(25000);
        digitalWrite(13, LOW);
        
        Serial.print("\nState: ");
        Serial.print(state);
        Serial.print(" st: ");
        Serial.print(st);
        Serial.print("  D: ");
        Serial.print(distance);
        Serial.print("  ND: ");
        Serial.print(next_dist);
    //}
}

ISR(ADC_vect)
{
    unsigned long analogVal = (ADCL | (ADCH << 8)) << 4;
    adc_val[adc_pin] += (analogVal >> 6) - (adc_val[adc_pin] >> 6);
    adc_pin = (adc_pin + 1) % 6;
    ADMUX &= B11110000; ADMUX |= adc_pin; ADCSRA |= B01000000;
}
