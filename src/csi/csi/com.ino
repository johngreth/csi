
byte packet[84];

void send_data() {
    #ifdef DEBUG
    int i;
    for (i = 0; i < 81; i++) {
        if (defects[i] == 10000) Serial.print(2);
        if (defects[i] >  10000) Serial.print(1);
        if (defects[i] <  10000) Serial.print(0);
        Serial.print(" ");
        if (i % 9 == 8) Serial.print("\n");
    }
    #else
    int i;
    packet[0] = 0;
    packet[1] = 0;
    for (i = 0; i < 81; i++) {
        if (defects[i] == 10000) packet[i+2] = 2;
        else {
            packet[0]++;
            if (defects[i] > 10000) {
                packet[i+2] = 1;
                packet[1]++;
            }
            else packet[i+2] = 0;
        }
    }
    packet[83] = -1;
    Serial.write(packet, 83);
    #endif
}


