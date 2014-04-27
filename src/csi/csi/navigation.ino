
// Number of encoder ticks the robot is allowed to go beyond the expected value before it turns.
// This is a failsafe in case the sensors miss the edges.
#define SLACK 4000
#define UDSLACK 15

volatile int distance_LR = 0;
volatile int distance_TB = 0;

// The distance at which the machine should go to the next state.
int next_dist;

// These are called whenever the encoder ticks.
void en_TB() { distance_TB++; }
void en_LR() { distance_LR++; }

int get_dist() {
    // Moving up or down
    if ((stage == 1) || (stage == 4)) return distance_LR;
    // Choose the relevant correct encoder value
    return distance_TB;
}

void reset_dist() {
    distance_TB = 1;
    distance_LR = 1;
}

void check_edge() {
    // If you are in the detecting state and an edge is detected, always go to the next state.
    switch (cycle_state) {
        case 4: case 12:
            if (button_pressed()) {
                next_dist = 0;
                reset_dist();
            }
            break;
        case 3: case 13:
            if (get_adc(0) > 12500L) {
                next_dist = 0;
                reset_dist();
            }
            break;
        case 8: case 17:
                if (abs_state < B00001111) {
                    if (get_color_val(1) > COLOR_EDGE_L) {
                        next_dist = 0;
                        reset_dist();
                    }
                    if (get_color_val(3) > COLOR_EDGE_L) {
                        next_dist = 0;
                        reset_dist();
                    }
            }
            else { /*
                    if (get_color_val(0) > COLOR_EDGE_R) {
                        next_dist = 0;
                        reset_dist();
                    } */
                    if (get_color_val(2) > COLOR_EDGE_R) {
                        next_dist = 0;
                        reset_dist();
                    }
            }
    }
}

void set_speeds() {
    // Reset the motor speeds to stationary
    pwm_out[L2] = 255;
    pwm_out[R2] = 255;
    pwm_out[LR1] = 240;
    pwm_out[TB1] = 255;
    pwm_out[TB2] = 255;
    
    if (abs_state < 45) {
        switch (stage) {
            // Moving right.
            case 0: case 3: 
                if (button_pressed()) pwm_out[LR1] = 50;
                else {
                    if (abs_state < 15) pwm_out[TB2] = 190; 
                    else pwm_out[TB1] = 190;
                }
            break;
            
            // Moving Left.
            case 2: case 5: 
                if (button_pressed()) pwm_out[LR1] = 50;
                else {
                    if (abs_state < 15) pwm_out[TB1] = 190; 
                    else pwm_out[TB2] = 190;
                }
            break;
            
            // Moving up/down.
            case 1: case 4:
                if ((cycle_state & 1) == 0) { // Moving down
                    pwm_out[L2] = 240;
                    pwm_out[R2] = 240;
                    pwm_out[LR1] = 255;
                }
                else {
                    pwm_out[LR1] = 50;
                }
        }
    }
    // Write the speeds.
    analogWrite(TB1, pwm_out[TB1]);
    analogWrite(TB2, pwm_out[TB2]);
    analogWrite(R2, pwm_out[R2]);
    analogWrite(L2, pwm_out[L2]);
    analogWrite(LR1, pwm_out[LR1]);
}

int get_next_dist() { return next_dist; }

void set_next_dist() {
    next_dist = curr_width;
    if ((stage == 5) && (cycle == 3)) next_dist = 50+11*WIDTH/2;
    if (stage_state == 0) next_dist -= BUFFER + BUFFER;
    
    // Moving slow. Starts looking for edges when it is close.
    if (stage_state == 1) next_dist -= BUFFER;
    
    // Set the failsafe in case it misses the edge.
    if ((stage_state == 2) && (stage != 0) && (stage != 3))
        next_dist += SLACK;
    
    // Moving up/down. 
    if ((stage == 1) || (stage == 4)) next_dist = 30000;
    
    if (cycle_state == 5) {
        reset_dist();
        next_dist = 75;
    }
    
    if (cycle_state == 14) {
        reset_dist();
        next_dist = 0;
    }
    
    if (abs_state == 2)
        next_dist += WIDTH + (WIDTH / 2) - 150;
    
    /*
    if ((stage == 1) || (stage == 4)) {
        next_dist = UPHEIGHT;
        if (stage == 1)
            next_dist = DOWNHEIGHT;
            
        if (stage_state == 0) next_dist -= UDBUFFER + UDBUFFER;
        
        // Moving slow. Starts looking for edges when it is close.
        if (stage_state == 1) next_dist -= UDBUFFER;
        
        // Set the failsafe in case it misses the edge.
        if ((stage_state == 2) && (stage != 0) && (stage != 3))
            next_dist += UDSLACK;
    }
    */
}


