#pragma once

//Bar sensor reading
void llegir_barra_sensors(void);

//Bar sensor readig calibrated from 0 to 255
uint16_t read_sensor_bar_calibrated(void);

//Calibrate Bar sensor during 10 seconds
void calibrate_sensors(void);

extern uint8_t sensors_min_reading[6];
extern uint8_t sensors_max_reading[6];
