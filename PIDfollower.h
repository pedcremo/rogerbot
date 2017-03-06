#pragma once

// Sensors. Inputs and Outputs
#define _SENSOR_D2          (PIND & _BV(PORTD7))
#define _SENSOR_D1          (PINC & _BV(PORTC3))
#define _SENSOR_D0          (PINC & _BV(PORTC2))
#define _SENSOR_I0          (PINC & _BV(PORTC1))
#define _SENSOR_I1          (PINC & _BV(PORTC0))
#define _SENSOR_I2          (PIND & _BV(PORTD4))

//Follow line track
void PID_line_following(int direction);

//Bar sensor reading

uint16_t llegir_barra_sensors(void);
int PID_obtenir_errorp(void);

#define BV(bit)			(1 << bit)
#define clearBit(byte,bit) 	(byte &= ~BV(bit))
