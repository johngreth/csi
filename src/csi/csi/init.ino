void setup () {
    // Start serial
    #ifdef DEBUG
    Serial.begin(115200);
    #else
    Serial.begin(9600);
    #endif
    
    init_motors();
    init_state();
    
    attachInterrupt(1, en_TB, RISING);
    attachInterrupt(0, en_LR, RISING);
    
    sei();
    init_imu();
    adc_init();
    reset_defects();
        
    #ifdef DEBUG
    while (Serial.read() == -1);
    #else
    started = 0;
    while (Serial.read() == -1);
    int vali;
    while (-1 == (vali = Serial.read())) ;
    byte val = vali;
    while (Serial.read() == -1);
    Serial.write(val);
    val = 1;
    Serial.write(val);
    /*
    delay(500);
    send_data();
    delay(5000);
    for (a = 0; a < 81; a++) {
        if (a % 4 == 0)
            defects[a]++;
        else
            defects[a]--;
    }
    send_data();
    */
    #endif
    set_speeds();
}

void init_state() {
    // Reset the encoder values
    reset_dist();
    
    // Reset the state
    abs_state = 0;
    cycle_state = 0;
    cycle = 0;
    stage_state = 0;
    stage = 0;
    
    // Set the width and distance of the first step.
    curr_width = widths[0];
    set_next_dist();
    
    byte i;
    for (i = 0; i < 81; i++)
        defects[i] = 10000;
}

void init_motors() {
    // Motor pins are outputs
    pinMode(LR1, OUTPUT); pinMode(R2, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(TB1, OUTPUT); pinMode(TB2, OUTPUT);
    
    analogWrite(TB1, 255);
    analogWrite(TB2, 255);
    analogWrite(R2, 255);
    analogWrite(L2, 255);
    analogWrite(LR1, 255);
    
    // For the LED
    pinMode(13, OUTPUT);
    
    pinMode(NW, INPUT); pinMode(NE, INPUT);
    pinMode(SW, INPUT); pinMode(SA, INPUT);
}
