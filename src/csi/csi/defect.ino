
// Values below which are an edge for the color sensor
#define COLOR_EDGE 115000L

// Values above which are a defect for the color sensor
#define COLOR_DEFECT 40000L

#define NUM_TIMES 4

boolean last[] = {false, false, false, false};
byte idx[] = {0, 0, 0, 0};
unsigned long latest_time[] = {0, 0, 0, 0};
byte pin[] = {NE, NW, SA, SW};
unsigned long times[4][NUM_TIMES];
byte valid[4][NUM_TIMES];

byte button_ctr;

unsigned long get_color_val(byte i) {
    if (valid[i][idx[i]])
        return latest_time[i];
    else
        return 110000L;
}

void update_colors(unsigned long time) {
    if (button_ctr != 0) button_ctr--;
    if (get_adc(1) > 8000L) button_ctr = 100;
    boolean next;
    int i;
    for (i = 0; i < 4; i++) {
        next = HIGH == digitalRead(pin[i]);
        if (next ^ last[i]) {
            byte next_idx = (idx[i] + 1) % NUM_TIMES;
            times[i][idx[i]] = time;
            latest_time[i] = time - times[i][next_idx];
            if (dT <= 400) mark_defect(i);
            valid[i][idx[i]] = 1;
            idx[i] = next_idx;
        }
        last[i] = next;
    }
    if (dT > 400) {
        byte a, b;
        for (a = 0; a < 4; a++)
            for (b = 0; b < NUM_TIMES; b++)
                valid[a][b] = 0;
    }
}

void mark_defect(byte id) {
    if (!valid[id][idx[id]]) return;
    int def = -1;
    if (latest_time[id] < COLOR_DEFECT) def = 1;
    byte pos = x_pos[cycle];
    int dist = get_dist();
    if ((id == 0) || (id == 2)) pos += 2;
    if ((id == 2) || (id == 3)) pos += 18;
    if (cycle_state == 4) {
        if (dist > DOWNHEIGHT) return;
        pos += 9 * map(dist, 0, DOWNHEIGHT, 0, 7);
        defects[pos] += def;
    }
    else if (cycle_state == 13) {
        if (dist > UPHEIGHT) return;
        pos += 9 * map(dist, UPHEIGHT, 0, 0, 7);
        defects[pos] += def;
    }
}

byte button_pressed() {
    return (button_ctr == 100);
}

byte button_depressed() {
    return (button_ctr == 1);
}


