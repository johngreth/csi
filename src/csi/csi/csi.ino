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

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define DEBUG

// Digital pin defines
#define RX 0 // Reserved
#define TX 1 // Reserved

// Top and left encoder pins
#define L_E1 2 // Interrupt
#define T_E1 3 // Interrupt

// Motor pins
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

// Color sensor pins
#define NW 4
#define NE 7
#define SW 8
#define SE 12

// Button pin
#define BUTTON 13

// Constant defines

// Values below which are an edge for the color sensor
#define COLOR_EDGE 750000L

// Values above which are a defect for the color sensor
#define COLOR_DEFECT 300000L

#define MAX_DEFECT_POINTS 256

#define NUM_TIMES 16

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
#define SLACK 200

// The step that is currently being executed. Diagram on Google Drive.
byte abs_state = 0;
byte cycle_state = 0;
byte stage_state = 0;
byte stage = 0;
byte cycle = 0;

// The x position, in encoder ticks, that the robot should achieve this pass.
volatile int curr_width;

// The number of ticks each encoder has seen since the start of the current pass.
volatile int distance_LR = 0;
volatile int distance_TB = 0;

int defects[81];

// The distance at which the machine should go to the next state.
int next_dist;

// The x position of each pass.
int widths[] = {1*WIDTH/2, 3*WIDTH/2, 9*WIDTH/2, 11*WIDTH/2, 13*WIDTH/2};
int x_pos[] = {0, 1, 4, 5, 6};

// The defect data collected on the current pass.
volatile int defect_points[14][MAX_DEFECT_POINTS];

// The pwm values for each pin. 
byte pwm_out[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

boolean last[] = {false, false, false, false};
byte idx[] = {0, 0, 0, 0};
unsigned long latest_time[] = {0, 0, 0, 0};
byte pin[] = {NE, NW, SE, SW};
unsigned long times[4][NUM_TIMES];

// Allows printing only after some amount of time.
byte printCtr = 0;

char packet[168];

// These are called whenever the encoder ticks.
void en_TB() { distance_TB++; }
void en_LR() { distance_LR++; }

MPU6050 accelgyro;
int16_t ax, ay, az; //define 3-axis accelerometer variables
int16_t gx, gy, gz; //define 3-axis gyroscop variables
signed long yaw=0; //define angle parameters
unsigned long last_time=0; //define time parameters

signed long all_dT;

void setup () {
    // Start serial
    Serial.begin(9600);
    init_motors();
    init_state();
    attachInterrupt(1, en_TB, RISING);
    attachInterrupt(0, en_LR, RISING);
    sei();
    init_imu();
}

void loop() { 
    if (abs_state == 45) {
        analogWrite(TB1, 256);
        analogWrite(TB2, 256);
        analogWrite(R2, 256);
        analogWrite(L2, 256);
        analogWrite(LR1, 256);
        return;
    }
    unsigned long time = micros();
    update_yaw(time);
    update_colors(time);
    check_edge();
    
    // It is time to go to the next state
    if (get_dist() > next_dist) {
        debug_state_change();
        increment_state();
        set_speeds();
        set_next_dist();
    }

    control_R2();
    debug();
}

void init_state() {
    // Reset the encoder values
    distance_LR = 0;
    distance_TB = 0;
    
    // Reset the state
    abs_state = 0;
    cycle_state = 0;
    cycle = 0;
    stage_state = 0;
    stage = 0;
    
    // Set the width and distance of the first step.
    curr_width = widths[0];
    next_dist = curr_width - BUFFER - BUFFER;
    
    byte i;
    for (i = 0; i < 81; i++)
        defects[i] = 10000;
    for (i = 0; i < 167; i++)
        packet[i] = ',';
    packet[167] = '\0';
}

void init_motors() {
    // Motor pins are outputs
    pinMode(LR1, OUTPUT); pinMode(R2, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(TB1, OUTPUT); pinMode(TB2, OUTPUT);
    
    // For the LED
    pinMode(13, OUTPUT);
    
    // Start the motors moving right.
    pwm_out[L2] = 255;
    pwm_out[R2] = 255;
    pwm_out[LR1] = 180;
    pwm_out[TB1] = 0;
    pwm_out[TB2] = 80;
    analogWrite(TB1, pwm_out[TB1]);
    analogWrite(TB2, pwm_out[TB2]);
    analogWrite(R2, pwm_out[R2]);
    analogWrite(L2, pwm_out[L2]);
    analogWrite(LR1, pwm_out[LR1]);
}

void init_imu() {
    Wire.begin();
    accelgyro.initialize();
    accelgyro.setRate(0);//set the sampling rate to be 1khz
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);//set gyro to be +-250
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
    delay(1000);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    yaw = atan2(ay, ax)/PI*180;
    last_time = micros();
}

void update_colors(unsigned long time) {
    boolean next;
    int i;
    for (i = 0; i < 4; i++) {
        next = HIGH == digitalRead(pin[i]);
        if (next ^ last[i]) {
            byte next_idx = (idx[i] + 1) & (NUM_TIMES - 1);
            times[i][idx[i]] = time;
            latest_time[i] = time - times[i][next_idx];
            mark_defect(i);
            idx[i] = next_idx;
        }
        last[i] = next;
    }
}

void mark_defect(byte id) {
    int def = -1;
    if (latest_time[id] < COLOR_DEFECT) def = 1;
    byte pos = x_pos[cycle];
    if ((id == 0) || (id == 2)) pos += 2;
    if ((id == 2) || (id == 3)) pos += 18;
    if (stage == 1) {
        if (distance_LR > DOWNHEIGHT) return;
        pos += 9 * map(distance_LR, 0, DOWNHEIGHT, 0, 7);
        defects[pos] += def;
    }
    else if (stage == 4) {
        if (distance_LR > UPHEIGHT) return;
        pos += 9 * map(distance_LR, UPHEIGHT, 0, 0, 7);
        defects[pos] += def;
    }
}

void debug_state_change() {
    #ifdef DEBUG
    if (stage_state == 2) {
        if (next_dist != 0) {
            if (cycle_state == 5)
                Serial.print("\nMISSED BOTTOM EDGE");
            else if (cycle_state == 14)
                Serial.print("\nMISSED TOP EDGE");
            else 
                Serial.print("\nMISSED LEFT EDGE");
        }
        else {
            if (cycle_state == 5)
                Serial.print("\nBOTTOM EDGE DETECTED");
            else if (cycle_state == 14)
                Serial.print("\nTOP EDGE DETECTED");
            else 
                Serial.print("\nLEFT EDGE DETECTED");
        }
    }
    else if (stage_state == 1) {
        if (cycle_state == 4)
            Serial.print("\nLOOKING FOR BOTTOM EDGE");
        else if (cycle_state == 13)
            Serial.print("\nLOOKING FOR TOP EDGE");
        else 
            Serial.print("\nLOOKING FOR LEFT EDGE");
    }
    #endif
}

int get_dist() {
    // Moving up or down
    if ((stage == 1) || (stage == 4)) return distance_LR;
    // Choose the relevant correct encoder value
    return distance_TB;
}

void control_R2() {
    int straight_value = pwm_out[R2];
    
    int delta = map(yaw, -200, 200, -15, 15) - 5;
    /*
    if ((stage == 0) || (stage == 3))
        delta += 5;
    else if ((stage == 2) || (stage == 5))
        delta -= 5;
    */
    // PUT THE TILT CORRECTION HERE.
    // corrected_value starts out as the value that makes the robot go
    // straight. yaw is the angle from the IMU. Return the desired value.
    
    if (delta < 0)
        analogWrite(R2, straight_value + delta);
    else
        analogWrite(L2, straight_value - delta);
}

void check_edge() {
    // If you are in the detecting state and an edge is detected, always go to the next state.
    if (   (((cycle_state ==  8) || (cycle_state == 17)) && (latest_time[3] > COLOR_EDGE)) ||
            ((cycle_state == 14)                         && (latest_time[0] > COLOR_EDGE)) ||
            ((cycle_state == 5)                         && (digitalRead(BUTTON) == HIGH))) {
        next_dist = 0;
    }
}

void increment_state() {
    abs_state++;
    cycle_state++;
    stage_state++;
    if (stage_state == 3) {
        stage_state = 0;
        stage++;
        if ((stage == 3) || (stage == 6)) {
            cycle++;
            send_data();
            if (stage == 6) {
                stage = 0;
                cycle_state = 0;
            }
        }
    }

    // Set a new desired width when switching to a new pass.
    if ((cycle_state == 0) || (cycle_state == 9))
        curr_width = widths[cycle];
    
    // Reset the encoder values when you change direction
    if (stage_state == 0) { distance_TB = 0; distance_LR = 0; }
}

void send_data() {
    byte num_defects_1, num_defects_0;
    byte num_covered_1, num_covered_0;
    int i;
    for (i = 0; i < 81; i++) {
        if (defects[i] == 10000) packet[2*i+6] = '2';
        else {
            if (num_covered_1 == 9) {
                num_covered_1 = 0;
                num_covered_0++;
            } else num_covered_1++;
            if (defects[i] < 10000) {
                packet[2*i+6] = '1';
                if (num_defects_1 == 9) {
                    num_defects_1 = 0;
                    num_defects_0++;
                } else num_defects_1++;
            }
            else packet[2*i+6] = '0';
        }
    }
    packet[0] = char(num_covered_0 + 48);
    packet[1] = char(num_covered_1 + 48);
    packet[3] = char(num_defects_0 + 48);
    packet[4] = char(num_defects_1 + 48);
    Serial.write(packet);
}

void set_speeds() {
    // Reset the motor speeds to stationary
    pwm_out[L2] = 255;
    pwm_out[R2] = 255;
    pwm_out[LR1] = 200;
    pwm_out[TB1] = 255;
    pwm_out[TB2] = 255;
    
    switch (stage) {
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
    // analogWrite(R2, pwm_out[R2]);
    analogWrite(L2, pwm_out[L2]);
    analogWrite(LR1, pwm_out[LR1]);
}

void set_next_dist() {
    next_dist = curr_width;
    
    // Moving down. 
    if (stage == 1) next_dist = DOWNHEIGHT;
    
    // Moving up.
    else if (stage == 4) next_dist = UPHEIGHT;
    
    // Moving fast. Change states well before the destination.
    if (stage_state == 0) next_dist -= BUFFER + BUFFER;
    
    // Moving slow. Starts looking for edges when it is close.
    if (stage_state == 1) next_dist -= BUFFER;
    
    // Set the failsafe in case it misses the edge.
    if ((stage_state == 2) && (stage != 0) && (stage != 3))
        next_dist += SLACK;
}

void debug() { 
    // Only print every 20 cycles
    printCtr = (printCtr + 1) % 250;
    if (printCtr == 0) {
        int xPos, yPos;
        // Moving up or down.
        if ((stage == 1) || (stage == 4)) {
            xPos = curr_width / 100;
            // moving down.
            if (stage < 3) yPos = map(distance_LR, 0, DOWNHEIGHT, 28, 0);
            // Moving up.
            else yPos = map(distance_LR, 0, UPHEIGHT, 0, 28);
        }
        // Moving left or right.
        else {
            // At the top.
            if ((stage == 0) || (stage == 5)) yPos = 28;
            // At the bottom
            else yPos = 0;
            // Moving left.
            if ((stage == 0) || (stage == 3)) xPos = distance_TB / 100;
            // Moving right.
            else xPos = (curr_width - distance_TB) / 100;
        }
        
        // Print the status.
        Serial.print("\nABS_State: ");
        Serial.print(abs_state);
        Serial.print(" Cycle_State: ");
        Serial.print(cycle_state);
        Serial.print(" Stage_State: ");
        Serial.print(stage_state);
        Serial.print(" Stage: ");
        Serial.print(stage);
        Serial.print(" x: ");
        Serial.print(xPos);
        Serial.print(" y: ");
        Serial.print(yPos); 
        Serial.print(" NE: ");
        Serial.print(latest_time[0]);
        Serial.print(" NW: ");
        Serial.print(latest_time[1]);
        Serial.print(" SE: ");
        Serial.print(latest_time[2]);
        Serial.print(" SW: ");
        Serial.print(latest_time[3]);
        Serial.print(" Yaw: ");
        Serial.print(yaw);
        Serial.print(" dT: ");
        Serial.print(all_dT);
    } 
}

void update_yaw(unsigned long new_time) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // 100,000ths of a second
    signed long dT = (new_time-last_time)/10;
    all_dT = dT;
    signed long sdT = dT / 256;
    last_time = new_time;
   
    // 100ths of a dregree * 100
    signed long accYaw = long(atan2(ay,ax)*(5730.0*2.0)); // 1/100 ths of a degree * 10
    // 100ths of a degree * dT
    // 100*gz/65.5 (100ths of a degree/s) * dT/100,000 (s) * dT
    signed long  gyro_dYaw = (sdT*sdT*gz); // 1/100ths of a degree * dT
    
    //comlementary filter
    yaw = ((dT*yaw) - gyro_dYaw + accYaw) / (dT+5);
}
