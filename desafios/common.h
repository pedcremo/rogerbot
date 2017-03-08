#pragma once

extern int ticks_fora_circuit; //Per corregir comportament anomal del robot quan llig punt roig del centre dels segments. A vegades lectures -9 o 9 com si estiguerem fora del circuit
extern int rescue_estat_actual; //Per mantindre l estat en la prova de Rescat

//Rescue functions and state finite machine states
void adjust_speed_to_a_threshold(int current_read,int desired_read);
char es_negre(void);
int finding_line(int speedM1,int speedM2, int direction); //Return 0 if not found , 1 if found
void follow_line_until_crossroad(int crossroad_number);
void follow_line_fast(void); //State 1
void move_robot(int speedM1,int speedM2,int direction_,int milliseconds_);
