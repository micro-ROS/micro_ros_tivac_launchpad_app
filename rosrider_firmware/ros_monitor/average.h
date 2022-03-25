#ifndef AVERAGE_H_
#define AVERAGE_H_

#define SIZE 8

uint16_t cs_left_array[SIZE];
uint16_t cs_right_array[SIZE];

uint8_t i_cs_left = 0;
uint8_t i_cs_right = 0;

uint32_t cs_left_sum = 0L;
uint32_t cs_right_sum = 0L;

// TODO: add kalman filter here

void init_average() {
    for(int i=0; i<SIZE; i++) {
        cs_left_array[i] = 0;
        cs_right_array[i] = 0;
    }
}

uint16_t add_cs_left(uint16_t cs) {
    cs_left_array[i_cs_left] = cs;
    cs_left_sum += cs;
    i_cs_left++;
    if(i_cs_left==SIZE) { i_cs_left = 0; }
    cs_left_sum -= cs_left_array[i_cs_left];
    return cs_left_sum / SIZE;
}

uint16_t add_cs_right(uint16_t cs) {
    cs_right_array[i_cs_right] = cs;
    cs_right_sum += cs;
    i_cs_right++;
    if(i_cs_right==SIZE) { i_cs_right = 0; }
    cs_right_sum -= cs_right_array[i_cs_right];
    return cs_right_sum / SIZE;
}

#endif