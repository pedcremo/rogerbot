#pragma once

// Baby Orangutan frequency (20MHz) defined in Makefile
#define FORWARD 0
#define BACKWARD 1

//EEPROM STORED VALUES
#define VELOCITAT_ADDR_EEPROM 0
#define TELEMETRY_ENABLED_ADDR_EEPROM 1
#define KP_ADDR_EEPROM 2
#define KD_ADDR_EEPROM 3
#define STRATEGY 4
#define CURVE_CORRECTION 5


//Global variables shared across project
extern uint8_t sensors[6];
extern uint8_t Kp;  //48 lf 18 pots
extern uint8_t  Kd;//148 lf 28 pots multiplicador 10
extern volatile uint8_t velocitat; //180 lf 95 pots max 255
extern volatile uint8_t turbo; //Speed increment
extern uint8_t telemetry_enabled;
extern uint8_t curve_correction;
extern char strategy;
extern volatile char start;
extern uint16_t compass_direction_to_follow;
extern uint8_t DEBUG;

//Main functions
void init_ports(void);
void load_eeprom_settings(void);
void delay_ms(int ms);
void rescue_state_machine(void);
void rescue_state_machine_2015(void);
void rescue_state_machine_2016(void);

//LineFollowing functions
//int obtenir_errorp(void);
void inicializar_timer1(void);
//int obtenir_errord(void);
//void line_following(int direction);
void prova_compass_direction(void);

//ADC functions
void init_ADC(void);
uint16_t readADC(uint8_t channel);
uint8_t readADC8(uint8_t channel);

// //Rescue functions and state finite machine states
// void adjust_speed_to_a_threshold(int current_read,int desired_read);
// char es_negre(void);
// int finding_line(int speedM1,int speedM2, int direction); //Return 0 if not found , 1 if found
// void follow_line_until_crossroad(int crossroad_number);
// void follow_line_fast(void); //State 1
// void move_robot(int speedM1,int speedM2,int direction_,int milliseconds_);

//Ping functions
uint16_t get_current_millis(void);
void init_current_millis(void);
//int ping(void);
//void init_encoders(void);
//void reset_encoder(void);
//int get_encoder_counts(void);
