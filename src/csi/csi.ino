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
#define L2 4
#define R2 7
#define LR1 8
#define NE 9 
#define SW 10 
#define NW 11
#define SE 12

// Constant defines
#define COLOR_SCALAR 10

#define HIGH_THRESHOLD_LR 10000L
#define LOW_THRESHOLD_LR 6000L
#define HIGH_THRESHOLD_TB 22000L
#define LOW_THRESHOLD_TB 20000L

#define MAX_DEFECT_POINTS 1000

#define HIGHSPEED 255
#define LOWSPEED 25

#define STALL_TORQUE 45

#define UPHIGHSPEED 220
#define UPLOWSPEED 170

#define DOWNHIGHSPEED2 255
#define DOWNLOWSPEED2 255
#define DOWNHIGHSPEED1 255
#define DOWNLOWSPEED1 150

#define BUFFER 20
#define WIDTH 100
#define HEIGHT 100
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

volatile int adc_pin = 0;
volatile unsigned long adc_val[6] = {0,0,0,0,0,0};
volatile unsigned long min_val[6] = {0,0,0,0,0,0};
volatile unsigned long max_val[6] = {0,0,0,0,0,0};

volatile int defect_points[14][MAX_DEFECT_POINTS];

volatile int pwm_out[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int pwmCtr = 0;

void badInt() {}

// Read defect sensors at regular intervals
void enLR() {
    // Moving up and down
    if ((state % 9 == 3) || (state % 9 == 4) || (state % 9 == 5)) {
        distance_LR++;
        if ((state % 9 == 3) && (distance_LR > HEIGHT - BUFFER)) {
            state++;
            if (state % 18 < 9) {
                aWrite(L2, DOWNLOWSPEED2);
                aWrite(R2, DOWNLOWSPEED2);
                aWrite(LR1, DOWNLOWSPEED1);
            }
            else {
                aWrite(L2, UPLOWSPEED);
                aWrite(R2, UPLOWSPEED);
            }
        }
        else if ((state % 9 == 4) && (distance_LR > HEIGHT)) {
            state++;
            adc_val[TOPBOTTOM] = (HIGH_THRESHOLD_TB + LOW_THRESHOLD_TB) / 2;
        }
        else if ((state % 9 == 5) && (distance_LR > HEIGHT + SLACK)) {
            state++;
            aWrite(L2, STALL_TORQUE);
            aWrite(R2, STALL_TORQUE);
            aWrite(TB1, HIGHSPEED);
            distance_TB = 0;
        }
        if (defect_array_index < MAX_DEFECT_POINTS) {
            switch (distance_LR % COLOR_SCALAR) {
                case 0: //defect_points[NE][defect_array_index] = digitalRead(NE); break;
                case 1: //defect_points[NW][defect_array_index] = digitalRead(NW); break;
                case 2: //defect_points[SE][defect_array_index] = digitalRead(SE); break;
                case 3: //defect_points[SW][defect_array_index] = digitalRead(SW); break;
                case 4: defect_array_index++; break;
            }
        }
    }
    
}

// Move right the correct distance
void enTB() {
    // Moving Left or Right
    if ((state % 9 < 3) || (state % 9 >= 6)) {
        distance_TB++;
        if ((state % 9 == 0) && (distance_TB > curr_width - BUFFER)) {
            state++;
            aWrite(TB1, LOWSPEED);
        }
        else if (((state % 9 == 1) || (state % 9 == 2)) && (distance_TB > curr_width - BUFFER)) {
            state++;
            if (state % 9 == 1) state++;
            aWrite(TB1, 0);
            if (state % 18 < 9) {
                aWrite(L2, DOWNHIGHSPEED2);
                aWrite(R2, DOWNHIGHSPEED2);
                aWrite(LR1, DOWNHIGHSPEED1);
            }
            else {
                aWrite(L2, UPHIGHSPEED);
                aWrite(R2, UPHIGHSPEED);
            }
            
        }
        else if ((state % 9 == 6) && (distance_TB > curr_width - BUFFER - BUFFER)) {
            state++;
            aWrite(TB2, LOWSPEED);
        }
        else if ((state % 9 == 7) && (distance_TB > curr_width - BUFFER)) { 
            state++;
            adc_val[LEFTRIGHT] = (HIGH_THRESHOLD_LR + LOW_THRESHOLD_LR) / 2;
        }
        else if ((state % 9 == 8) && (distance_TB > curr_width + SLACK)) {
            state++;
            distance_TB = 0;
            aWrite(TB2, 0);
            aWrite(TB1, HIGHSPEED);
        }
    }
}

void setup () {
    cli();
    Serial.begin(9600);
    // Timer
    TCNT1  = 0; OCR1A = (PWMFREQ * 1) - 1; TCCR1A = (1 << WGM12);
    TCCR1B = (1 << CS10); TIMSK1 |= (1 << OCIE1A);
    TCNT2  = 0; OCR2A = 6                     ; TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22); TIMSK2 |= (1 << OCIE2A);
    
    // ADC
    adc_pin = 0;
    ADMUX |= B01000000; ADMUX &= B11010000; ADCSRA |= B11101111;
    
    // Defect sensing pins
    pinMode(NE, INPUT); pinMode(NW, INPUT);
    pinMode(SE, INPUT); pinMode(SW, INPUT);

    // Motors
    pinMode(LR1, OUTPUT); pinMode(R2, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(TB1, OUTPUT); pinMode(TB2, OUTPUT);
    
    pinMode(13, OUTPUT);
    
    aWrite(LR1, 0); aWrite(L2, 0); aWrite(R2, 0);
    aWrite(TB2, HIGHSPEED);  aWrite(TB1, 0);
    distance_LR = 0; distance_TB = 0;
    
    state = 0;
    distance_TB = 0;
    distance_LR = 0;

    // Encoders
    //pinMode(L_E1, INPUT); pinMode(T_E1, INPUT);
    
    attachInterrupt(0, enTB, RISING);
    attachInterrupt(1, enLR, RISING);

    sei();
}

void loop() {

    
    // TODO: Later. Agregate defect data after up/down pass.

    delay(250); 
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    /*
    if (digitalRead(NE) == HIGH) Serial.print("NE Defect\n");
    if (digitalRead(NW) == HIGH) Serial.print("NW Defect\n");
    if (digitalRead(SE) == HIGH) Serial.print("SE Defect\n");
    if (digitalRead(SW) == HIGH) Serial.print("SW Defect\n"); */
    // Serial.print("State: ");
    // Serial.print(state);
    //Serial.print(adc_val[0]);
   //  Serial.print(state);
    //if (adc_val[0] > 300) Serial.print("!!! DEFECT !!! DEFECT !!! DEFECT !!!! DEFECT !!!\n");
    //else
    // Serial.print("\n");
    
    Serial.print("\nState: ");
    Serial.print(state);
    Serial.print(" TB min: ");
    Serial.print(min_val[TOPBOTTOM]);
    Serial.print("  max: ");
    Serial.print(max_val[TOPBOTTOM]);
    Serial.print(" LR min: ");
    Serial.print(min_val[LEFTRIGHT]);
    Serial.print("  max: ");
    Serial.print(max_val[LEFTRIGHT]);
    for (int i = 0; i < 6; i++) {
        min_val[i] = 0x8000;
        max_val[i] = 0;
    }
}

// PID control
ISR(TIMER2_COMPA_vect) {
    // adc_pin = adc_pin % 6;
    // TODO: Later. Add tilt correction.
    // TODO: Add height correction while moving along top and bottom.
    // TODO: make corrections use PID
}

// PWM timer
ISR(TIMER1_COMPA_vect) {
    
    // Moving up and down
    if ((state % 9 == 3) || (state % 9 == 4) || (state % 9 == 5)) {
        distance_LR++;
        if ((state % 9 == 3) && (distance_LR > HEIGHT - BUFFER)) {
            state++;
            if (state % 18 < 9) {
                aWrite(L2, DOWNLOWSPEED2);
                aWrite(R2, DOWNLOWSPEED2);
                aWrite(LR1, DOWNLOWSPEED1);
            }
            else {
                aWrite(L2, UPLOWSPEED);
                aWrite(R2, UPLOWSPEED);
            }
        }
        else if ((state % 9 == 4) && (distance_LR > HEIGHT))
            state++;
        else if ((state % 9 == 5) && (distance_LR > HEIGHT + SLACK)) {
            state++;
            aWrite(L2, STALL_TORQUE);
            aWrite(R2, STALL_TORQUE);
            aWrite(LR1, 0);
            aWrite(TB1, HIGHSPEED);
            distance_TB = 0;
        }
        if (defect_array_index < MAX_DEFECT_POINTS) {
            switch (distance_LR % COLOR_SCALAR) {
                case 0: //defect_points[NE][defect_array_index] = digitalRead(NE); break;
                case 1: //defect_points[NW][defect_array_index] = digitalRead(NW); break;
                case 2: //defect_points[SE][defect_array_index] = digitalRead(SE); break;
                case 3: //defect_points[SW][defect_array_index] = digitalRead(SW); break;
                case 4: defect_array_index++; break;
            }
        }
    }
    
    
    pwmCtr = (pwmCtr + PWMRES) % 256;
    
    if (pwmCtr < pwm_out[TB1])
        digitalWrite(TB1, HIGH);
    else
        digitalWrite(TB1,  LOW);
    if (pwmCtr < pwm_out[TB2])
        digitalWrite(TB2, HIGH);
    else
        digitalWrite(TB2,  LOW);
    if (pwmCtr < pwm_out[L2])
        digitalWrite(L2, HIGH);
    else
        digitalWrite(L2,  LOW);
    if (pwmCtr < pwm_out[R2])
        digitalWrite(R2, HIGH);
    else
        digitalWrite(R2,  LOW);
    if (pwmCtr < pwm_out[LR1])
        digitalWrite(LR1, HIGH);
    else
        digitalWrite(LR1,  LOW); 
}

ISR(ADC_vect)
{
    unsigned long analogVal = (ADCL | (ADCH << 8)) << 4;
    int curr_adc_pin = adc_pin;
    if (state % 9 == 5) adc_pin = TOPBOTTOM;
    else if (state % 9 == 8) adc_pin = LEFTRIGHT;
    else adc_pin = ACCEL;

    // Start the next ADC
    ADMUX &= B11110000; ADMUX |= adc_pin; ADCSRA |= B01000000;
    
    adc_val[curr_adc_pin] += (analogVal >> 6) - (adc_val[curr_adc_pin] >> 6);
    if (adc_val[curr_adc_pin] < min_val[curr_adc_pin]) 
        min_val[curr_adc_pin] = adc_val[curr_adc_pin];
    if (adc_val[curr_adc_pin] > max_val[curr_adc_pin])
        max_val[curr_adc_pin] = adc_val[curr_adc_pin];

    // Update things that use ADC values
    
    // Moving up/down
    if ((curr_adc_pin == TOPBOTTOM) && (
            ((state % 18 == 5 ) && (adc_val[curr_adc_pin] <  LOW_THRESHOLD_TB)) ||
            ((state % 18 == 14) && (adc_val[curr_adc_pin] > HIGH_THRESHOLD_TB)))) {
        state++;
        aWrite(L2, STALL_TORQUE);
        aWrite(R2, STALL_TORQUE);
        aWrite(LR1, 0);
        aWrite(TB1, HIGHSPEED);
        distance_TB = 0;
    }
    // Moving Left
    else if (((curr_adc_pin == LEFTRIGHT) && (state % 9 == 8)) &&
             (adc_val[curr_adc_pin] > HIGH_THRESHOLD_LR)) {
        state++;
        distance_TB = 0;
        aWrite(TB2, 0);
        aWrite(TB1, HIGHSPEED);
    }
}

void aWrite(int pin, int sp) {
    pwm_out[pin] = sp;
}
