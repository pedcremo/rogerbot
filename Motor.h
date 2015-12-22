#ifndef Motor_H
#define Motor_H

//Motor functions
void Motor_right_forward(unsigned char pwm);
void Motor_right_reverse(unsigned char pwm);
void Motor_left_forward(unsigned char pwm);
void Motor_left_reverse(unsigned char pwm);
void Motor_acceleracio_progressiva(void); //Accelera poc a poc fins la velocitat de creuer
void Motor_init(void);

#endif
