
unsigned long last_time=0; //define time parameters

void loop() { 
    unsigned long time = micros();
    dT = (time-last_time)/10;
    last_time = time;
    
    update_yaw();
    update_colors(time);
    check_edge();
    
    if (button_pressed() || button_depressed()) set_speeds();
    
    // It is time to go to the next state
    if (get_dist() > get_next_dist()) {
        debug_state_change();
        increment_state();
        set_speeds();
        set_next_dist();
    }

    control_R2();
    debug();
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
    if (stage_state == 0) { reset_dist(); }
}


