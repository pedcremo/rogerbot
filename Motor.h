#pragma once

//Motor functions
void setSpeed(int16_t speedLeft,int16_t speedRight);
void Motor_right_forward(unsigned char pwm);
void Motor_right_reverse(unsigned char pwm);
void Motor_left_forward(unsigned char pwm);
void Motor_left_reverse(unsigned char pwm);
void Motor_acceleracio_progressiva(void); //Accelera poc a poc fins la g_velocitat de creuer
void Motor_init(void);
