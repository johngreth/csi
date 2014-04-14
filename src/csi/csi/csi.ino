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
 /*
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
*/
#define DEBUG

// Analog pin defines

// left/right line tracking sensor pin
#define LEFTRIGHT 5

// top/bottom line tracking sensor pin
#define TOPBOTTOM 6

// Top-right color sensor pin
#define NE 0

// Bottom-left color sensor pin
#define SW 1 

// Top-left color sensor pin
#define NW 2

// Bottom-right color sensor pin
#define SE 3

// Digital pin defines
#define RX 0 // Reserved
#define TX 1 // Reserved

// left/right encoder pin
#define L_E1 2 // Interrupt

// Top/bottom encoder pin
#define T_E1 3 // Interrupt

// Pin 1 for the top and bottom motors. Drives them to the left.
#define TB1 5

// Pin 2 for the top and bottom motors. Drives them to the right.
#define TB2 6

// Pin 2 for the left motor. Drives the motor up.
#define L2 9 // 4

// Pin 2 for the right motor. Drives the motor up.
#define R2 10 // 7

// pin 1 for the left and right motors. Drives the motors down.
#define LR1 11 // 8

// Constant defines

// Controls how often defect data is collected.
#define COLOR_SCALAR 10

// The factor to determine how agressively to filter the adc values
#define FILTER 2

// Use color sensing for edges
#define COLOR_SENSE

// Use ir for sensing edges
// #define IR_SENSE

// Values below which are an edge for the color sensor
#define COLOR_EDGE 500

// Values above which are a defect for the color sensor
#define COLOR_DEFECT 1500

// Values above this indicate being over the left edge.
#define HIGH_THRESHOLD_LR 13000L
#define LOW_THRESHOLD_LR 13000L

// Values above this indicate being over the top edge.
// PID control should be used to keep adc_val[TOPBOTTOM] close to this value.
#define HIGH_THRESHOLD_TB 8500L

// Values below this indicate being at the bottom of the wall.
#define LOW_THRESHOLD_TB 6000L

#define MAX_DEFECT_POINTS 1000

// Slow down for this number of encoder ticks before a turn.
#define BUFFER 1000

// Width in encoder ticks of a defect.
#define WIDTH 400

// Number of encoder ticks going from bottom to top
#define UPHEIGHT 4600

// Number of encoder ticks going from top to bottom
#define DOWNHEIGHT 3600

// Number of encoder ticks the robot is allowed to go beyond the expected value before it turns.
// This is a failsafe in case the sensors miss the edges.
#define SLACK 100

// Used by the manual pwm code that is not currently used.
#define PWMRES 15
#define PWMFREQ 2

// The step that is currently being executed. Diagram on Google Drive.
volatile int state = 0;

// The x position, in encoder ticks, that the robot should achieve this pass.
volatile int curr_width;

// The number of ticks each encoder has seen since the start of the current pass.
volatile int distance_LR = 0;
volatile int distance_TB = 0;

// The defect index currently being observed.
volatile int defect_array_index = 0;

// The distance at which the machine should go to the next state.
volatile int next_dist;

// The ADC pin that is currently being read.
volatile int adc_pin = 0;

// The x position of each pass.
volatile unsigned int widths[6];

// The most recent filtered ADC values
volatile unsigned long adc_val[6] = {0,0,0,0,0,0};

// The defect data collected on the current pass.
volatile int defect_points[14][MAX_DEFECT_POINTS];

// The pwm values for each pin. Currently not used.
volatile int pwm_out[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Not used.
volatile int pwmCtr = 0;

// Allows printing only after some amount of time.
volatile int printCtr = 0;

// These are called whenever the encoder ticks.
void en_TB() { distance_TB++; }
void en_LR() { distance_LR++; }

#define acc_scale_factor 16384 //16384 LSB/g
#define gyro_scale_factor 65.5  //131 LSB/(degree/s)

MPU6050 accelgyro;
int16_t ax, ay, az; //define 3-axis accelerometer variables
int16_t gx, gy, gz; //define 3-axis gyroscop variables
double  yaw=0; //define angle parameters
double dT=0, time=0; //define time parameters

void setup () {
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();
    accelgyro.setRate(0);//set the sampling rate to be 1khz
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);//set gyro to be +-250
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
    
    // Stop interrupts
    cli();
    
    // Start serial
    Serial.begin(9600);
    
    // Start Timers
    //TCNT1  = 0; OCR1A = (PWMFREQ * 1) - 1; TCCR1A = (1 << WGM12);
    //TCCR1B = (1 << CS10); TIMSK1 |= (1 << OCIE1A);
    //TCNT2  = 0; OCR2A = 6                     ; TCCR2A = (1 << WGM21);
    //TCCR2B = (1 << CS22); TIMSK2 |= (1 << OCIE2A);
    
    // Start the first ADC conversion
    adc_pin = 0;
    ADMUX |= B01000000; ADMUX &= B11010000; ADCSRA |= B11101111;

    // Motor pins are outputs
    pinMode(LR1, OUTPUT); pinMode(R2, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(TB1, OUTPUT); pinMode(TB2, OUTPUT);
    
    // For the LED
    pinMode(13, OUTPUT);
    
    // Start the motors moving right.
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
    
    // Reset the encoder values
    distance_LR = 0;
    distance_TB = 0;
    
    // Reset the state
    state = 0;
    
    // Set the x position of each pass
    widths[0] = 1*WIDTH/2;
    widths[1] = 3*WIDTH/2;
    widths[2] = 5*WIDTH/2;
    widths[3] = 7*WIDTH/2;
    widths[4] = 9*WIDTH/2;
    widths[5] = 11*WIDTH/2;
    
    // Set the width and distance of the first step.
    curr_width = widths[0];
    next_dist = curr_width - BUFFER - BUFFER;

    // Encoders
    attachInterrupt(1, en_TB, RISING);
    attachInterrupt(0, en_LR, RISING);

    // Enable interrupts
    sei();
}

void loop() { 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double accYaw = atan2(ay,ax)/PI*180;
    double gyroz = -(double)gz/gyro_scale_factor; //convert the gyroscope data to angular velocity and invert
    double gyro_dYaw = gyroz*dT;
    
    //comlementary filter
    double acc_gyro_weight = dT/(dT+0.001);
    yaw = acc_gyro_weight*(yaw+gyro_dYaw)+(1-acc_gyro_weight)*accYaw;
  
    int sta = (state % 18);
    int st = sta / 3;
    
    // Choose the relevant correct encoder value
    int distance = distance_TB;
    // Moving up or down
    if (st % 3 == 1) distance = distance_LR;
   
    #ifdef IR_SENSE
            // If you are in the detecting state and an edge is detected, always go to the next state.
            if (    ((state % 9  ==  8) && (adc_val[LEFTRIGHT] > HIGH_THRESHOLD_LR)) ||
                    ((state % 18 ==  5) && (adc_val[TOPBOTTOM] <  LOW_THRESHOLD_TB)) ||
                    ((state % 18 == 14) && (adc_val[TOPBOTTOM] > HIGH_THRESHOLD_TB))) {
                next_dist = 0;
                Serial.print("\nEDGE DETECTED");
            }
    #endif
    #ifdef COLOR_SENSE
            // If you are in the detecting state and an edge is detected, always go to the next state.
            if (    ((state % 9  ==  8) && (adc_val[SW] < COLOR_EDGE)) ||
                    ((state % 18 ==  5) && (adc_val[NE] < COLOR_EDGE)) ||
                    ((state % 18 == 14) && (adc_val[TOPBOTTOM] > HIGH_THRESHOLD_TB))) {
                next_dist = 0;
                Serial.print("\nEDGE DETECTED");
            }
    #endif
    
    // It is time to go to the next state
    if (distance > next_dist) {
        if ((state % 3 == 2) && (next_dist != 0))
            Serial.print("\nEDGE MISSED");
        state++;
        
        if ((state % 3 == 2))
            Serial.print("\nLOOKING FOR EDGE");
        sta = (state % 18);
        st = sta / 3;
        
        // Set a new desired width when switching to a new pass.
        if (state % 9 == 0) curr_width = widths[(state/9)%6];
        
        // Reset the encoder values when you change direction
        if (state % 3 == 0) { distance_TB = 0; distance_LR = 0; }
        
        // Reset the motor speeds to stationary
        pwm_out[L2] = 240;
        pwm_out[R2] = 240;
        pwm_out[LR1] = 225;
        pwm_out[TB1] = 255;
        pwm_out[TB2] = 255;
        
        switch (st) {
            // Moving right.
            case 0: case 3: 
                pwm_out[TB1] = 180;
            break;
            
            // Moving Left.
            case 2: case 5: 
                pwm_out[TB2] = 180;
            break;
            
            // Moving down.
            case 1:
                pwm_out[LR1] = 255;
                pwm_out[L2] = 230;
                pwm_out[R2] = 230;
            break;
                
            // Moving up.
            case 4:
                pwm_out[LR1] = 85;
            break;
        }
        
        // Write the speeds.
        analogWrite(TB1, pwm_out[TB1]);
        analogWrite(TB2, pwm_out[TB2]);
        analogWrite(R2, pwm_out[R2]);
        analogWrite(L2, pwm_out[L2]);
        analogWrite(LR1, pwm_out[LR1]);
        
        // Set next_dist according to the current state and width
        // Default moving left or right.
        next_dist = curr_width;
        
        // Moving down. 
        if (st == 1) next_dist = DOWNHEIGHT;
        
        // Moving up.
        if (st == 4) next_dist = UPHEIGHT;
        
        // Moving fast. Change states well before the destination.
        if (state % 3 == 0) next_dist -= BUFFER + BUFFER;
        
        // Moving slow. Starts looking for edges when it is close.
        if (state % 3 == 1) next_dist -= BUFFER;
        
        // Set the failsafe in case it misses the edge.
        if ((state % 9 == 5) && (state % 9 == 8)) next_dist += SLACK;
    }

    int corrected_value = pwm_out[R2];

    // PUT THE TILT CORRECTION HERE.
    // corrected_value starts out as the value that makes the robot go
    // straight. yaw is the angle from the IMU. Modify corrected_value to 
    // create a correction in the robot's dirrection

    analogWrite(R2, corrected_value);
    
    // delay 50 ms and blink the light.
    delayMicroseconds(25000); 
    digitalWrite(13, HIGH);
    delayMicroseconds(25000);
    digitalWrite(13, LOW);
        
    // Only print every 20 cycles
    printCtr = (printCtr + 1) % 20;
    
    if (printCtr == 0) {
        int xPos, yPos;
        // Moving up or down.
        if ((st % 3) == 1) {
            xPos = curr_width / 100;
            // moving down.
            if (st < 3) yPos = map(distance_LR, 0, DOWNHEIGHT, 28, 0);
            // Moving up.
            else yPos = map(distance_LR, 0, UPHEIGHT, 0, 28);
        }
        // Moving left or right.
        else {
            // At the top.
            if ((st == 0) || (st == 5)) yPos = 28;
            // At the bottom
            else yPos = 0;
            // Moving left.
            if ((st % 3) == 0) xPos = distance_TB / 100;
            // Moving right.
            else xPos = (curr_width - distance_TB) / 100;
        }
        
        // Print the status.
        Serial.print("\nState: ");
        Serial.print(state);
        Serial.print(" x: ");
        Serial.print(xPos);
        Serial.print(" y: ");
        Serial.print(yPos);
        Serial.print(" NE: ");
        Serial.print(adc_val[NE]);
        Serial.print(" NW: ");
        Serial.print(adc_val[NW]);
        Serial.print(" SE: ");
        Serial.print(adc_val[SE]);
        Serial.print(" SW: ");
        Serial.print(adc_val[SW]);
        
    }
}

// Called when an analog-digital conversion completes
ISR(ADC_vect)
{
    // Record and amplify the value.
    unsigned long analogVal = (ADCL | (ADCH << 8)) << 4;
    
    // Add the new value to the rolling exponential buffer for that pin.
    adc_val[adc_pin] += (analogVal >> FILTER) - (adc_val[adc_pin] >> FILTER);
    
    // Set the next ADC conversion to happen.
    adc_pin = (adc_pin + 1) % 4;
    
    // Start the next analog-digital conversion
    ADMUX &= B11110000; ADMUX |= adc_pin; ADCSRA |= B01000000;
}
