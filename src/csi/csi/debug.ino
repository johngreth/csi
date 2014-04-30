
void debug_state_change() {
    #ifdef DEBUG
    if (stage_state == 2) {
        if (get_next_dist() != 0) {
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

byte printCtr = 0;

void debug() { 
    #ifdef DEBUG
    // Only print every 20 cycles
    printCtr = (printCtr + 1) % 250;
    if (printCtr == 0) {
        int xPos, yPos;
        // Moving up or down.
        if ((stage == 1) || (stage == 4)) {
            xPos = curr_width / 100;
            // moving down.
            if (stage < 3) yPos = map(get_dist(), 0, DOWNHEIGHT, 28, 0);
            // Moving up.
            else yPos = map(get_dist(), 0, UPHEIGHT, 0, 28);
        }
        // Moving left or right.
        else {
            // At the top.
            if ((stage == 0) || (stage == 5)) yPos = 28;
            // At the bottom
            else yPos = 0;
            // Moving left.
            if ((stage == 0) || (stage == 3)) xPos = get_dist() / 100;
            // Moving right.
            else xPos = (curr_width - get_dist()) / 100;
        }
        
        // Print the status.
        Serial.print("\nABS_State: ");
        Serial.print(abs_state);
        Serial.print(" Cycle_State: ");
        Serial.print(cycle_state);
        Serial.print(" Cycle: ");
        Serial.print(cycle);
        Serial.print(" Stage_State: ");
        Serial.print(stage_state);
        Serial.print(" Stage: ");
        Serial.print(stage); 
        Serial.print(" dist: ");
        Serial.print(get_dist());
        Serial.print(" next_dist: ");
        Serial.print(get_next_dist()); 
        Serial.print(" x: ");
        Serial.print(xPos);
        Serial.print(" y: ");
        Serial.print(yPos);
        Serial.print(" NE: ");
        Serial.print(get_color_val(0));
        Serial.print(" NW: ");
        Serial.print(get_color_val(1));
        Serial.print(" SE: ");
        Serial.print(get_color_val(2));
        Serial.print(" SW: ");
        Serial.print(get_color_val(3)); 
        Serial.print(" Yaw: ");
        Serial.print(get_yaw()); 
        Serial.print(" dT: ");
        Serial.print(dT); 
        Serial.print(" adc: ");
        Serial.print(get_adc(0));
        Serial.print(" adc: ");
        Serial.print(get_adc(1));
        Serial.print(" ddT: ");
        Serial.print(ddT); 
        Serial.print(" BF: ");
        Serial.print(bad_factor);
        Serial.print(" BF2: ");
        Serial.print(bad_factor2); 
        Serial.print(" BF3: ");
        Serial.print(bad_factor3); 
        Serial.print(" BF4: ");
        Serial.print(bad_factor4); 
    } 
    if (printCtr == 1) ddT = dT;
    #endif
}
