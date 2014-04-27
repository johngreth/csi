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
#define SW 12
#define SA 8

// Button pin
#define BUTTON 13

// Constant defines

// Slow down for this number of encoder ticks before a turn.
#define BUFFER 1000

// Width in encoder ticks of a defect.
#define WIDTH 750

// Number of encoder ticks going from bottom to top
#define UPHEIGHT 690

// Number of encoder ticks going from top to bottom
#define DOWNHEIGHT 550

// The step that is currently being executed. Diagram on Google Drive.
byte abs_state = 0;
byte cycle_state = 0;
byte stage_state = 0;
byte stage = 0;
byte cycle = 0;

// The x position, in encoder ticks, that the robot should achieve this pass.
int curr_width;

int defects[81];

// The x position of each pass.
int widths[] = {50+1*WIDTH/2, 50+3*WIDTH/2, 50+5*WIDTH/2, 50+3*WIDTH/2, 50+1*WIDTH/2};
int x_pos[] = {0, 1, 4, 5, 6};

// The pwm values for each pin. 
byte pwm_out[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

signed long dT, ddT;

byte started;


