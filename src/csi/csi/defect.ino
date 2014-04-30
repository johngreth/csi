
// Values below which are an edge for the color sensor
#define COLOR_EDGE_L 115000L
#define COLOR_EDGE_R 110000L

// Values above which are a defect for the color sensor
#define COLOR_DEFECT 25000L

#define NUM_TIMES 4

#define BUCKETS 7

byte data[4][7*BUCKETS*2];

int last_dist;

boolean last[] = {false, false, false, false};
byte idx[] = {0, 0, 0, 0};
unsigned long latest_time[] = {0, 0, 0, 0};
const byte pin[] = {NE, NW, SA, SW};
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
    byte bucket = 0;
    int dist = get_dist();
    if (cycle_state == 4) bucket = map(dist, 0, DOWNHEIGHT, 0, 7);
    else bucket = map(get_dist(), UPHEIGHT, 0, 0, 7);
    if (bucket >= 7*BUCKETS*2) bucket = 7*BUCKETS*2 - 1;
    boolean next;
    int i;
    for (i = 0; i < 4; i++) {
        next = HIGH == digitalRead(pin[i]);
        if (next ^ last[i]) {
            byte next_idx = (idx[i] + 1) % NUM_TIMES;
            times[i][idx[i]] = time;
            latest_time[i] = time - times[i][next_idx];
            if (dT <= 500) mark_defect(i, dist);
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

void mark_defect(byte id, int dist) {
    if (!valid[id][idx[id]]) return;
    int def = -1;
    if (latest_time[id] < COLOR_DEFECT) def = 1;
    if (cycle_state == 4) {
        byte pos = map(dist, 0, DOWNHEIGHT, 0, 7*BUCKETS);
        if (pos >= 7*BUCKETS*2) return;
        data[id][pos] += def;
        last_dist = dist;
    }
    else if (cycle_state == 13) {
        byte pos = map(dist, 0, UPHEIGHT, 0, 7*BUCKETS);
        if (pos >= 7*BUCKETS*2) return;
        data[id][pos] += def;
        last_dist = dist;
    }
}

void set_defects() {
    byte i, id;
    for (i = 0; i < 7; i++) {
        byte loc = (i << 1) | B00000001;
        int cap_dist = DOWNHEIGHT;
        if (cycle_state >= 9) { 
            loc = 14 - loc; 
            cap_dist = UPHEIGHT; 
        }
        byte cell = map(map(loc, 0, 14, 0, last_dist), 0, cap_dist, 0, 7*BUCKETS);
        byte base_pos = (9 * i) + x_pos[cycle];
        for (id = 0; id < 4; id++) {
            byte pos = base_pos;
            if ((id == 0) || (id == 2)) pos += 2;
            if ((id == 2) || (id == 3)) pos += 18;
            defects[pos] -= (128 + 128 + 128);
            defects[pos] += data[id][cell-1];
            defects[pos] += data[id][cell];
            defects[pos] += data[id][cell+1];
        }
    }
}

void reset_defects() {
    int i, j;
    for (i = 0; i < 4; i++) for (j = 0; j < 7*BUCKETS*2; j++) data[i][j] = 128;
    last_dist = 0;
}

byte button_pressed() {
    return (button_ctr == 100);
}

byte button_depressed() {
    return (button_ctr == 1);
}


