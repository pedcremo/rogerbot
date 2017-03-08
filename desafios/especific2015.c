#include <avr/io.h>
#include <util/delay.h>

#include "common.h"
#include "../Main.h"
#include "../Motor.h"
#include "../USART_and_telemetry.h"
#include "../Ping.h"
#include "../PIDfollower.h"


//Pots negres i blancs en un cercle de 8 segments

void rescue_state_machine_2015(){
	char dir_robot=0; //0 Cap endins ,1 cap a fora
	char lectura_pot=0;
	int ping_read=0;
	int rescue_old_estat=9;
	int delay_debug = 2;
	init_current_millis();

	while ( 1 )
	{
		if (DEBUG && (rescue_estat_actual != rescue_old_estat)){ //DEBUG
				USART_transmitByte(83); //S
				USART_transmitByte(61); //=
				USART_transmitByte(48+rescue_estat_actual);
				USART_transmitByte(32); //Space
				USART_transmitByte(13); //Carriage return
		}
		rescue_old_estat=rescue_estat_actual;
		switch(rescue_estat_actual){
			case 0:
				if (finding_line(velocitat,velocitat,FORWARD)==0) rescue_estat_actual=0;
				else rescue_estat_actual=1;
				break;
			case 1:
				follow_line_fast();
				delay_ms(2);
				//if (DEBUG) USART_transmitByte(48); //Print 5
				//if (telemetry_enabled) USART_transmitByte(48);
				if (strategy=='c'){ //Si estrategia = e ni mirem color ni res que se li parega
					//if (telemetry_enabled) USART_transmitByte(49);
					while (get_current_millis()<800){Motor_left_forward(0);Motor_right_forward(0);} //2 ms
					if (ping()<45 ){ //Si el pot esta a menys de 58 mm frenem i llegim color
						init_current_millis();

						Motor_right_forward(0);Motor_left_forward(0);
						if (DEBUG) {USART_transmitByte(50);delay_ms(delay_debug);} //Print 2
						_delay_ms(30);
						ping_read=ping();
						if (ping_read>30 && ping_read<45){
							if (PID_obtenir_errorp()>1){
								move_robot(velocitat/4,velocitat/3,FORWARD,35);
						 	}else if(PID_obtenir_errorp()<-1){
								move_robot(velocitat/3,velocitat/4,FORWARD,35);
						 	}else{
							 	move_robot(velocitat/3,velocitat/3,FORWARD,35);
							}
							if (DEBUG) {USART_transmitByte(51);delay_ms(delay_debug*2);}// Print 3
						}
						lectura_pot=es_negre(); //0 Blanc, 1 negre
						//Si el pot es negre i estem anant cap endins o el pot es blanc i estem
						//anant cap a fora
						if((lectura_pot==1 && dir_robot == 0) || (lectura_pot==0 && dir_robot == 1)  ){

							move_robot(velocitat,velocitat,BACKWARD,20); //Retrocedim 1cm
							move_robot(velocitat,-velocitat,FORWARD,230);
							//move_robot(velocitat*0.6,velocitat,FORWARD,250);
							while (finding_line(velocitat*0.42,velocitat,FORWARD) == 0){};
							Motor_right_forward(0);Motor_left_forward(0);
							if (DEBUG){delay_ms(delay_debug);}
							move_robot(velocitat*0.8,velocitat,FORWARD,180);

							if (DEBUG){delay_ms(delay_debug);}

							while (finding_line(-velocitat/2,velocitat/2,FORWARD) == 0){};

							if (dir_robot==0) dir_robot=1;
							else dir_robot=0;
							if (DEBUG) {Motor_right_forward(0);Motor_left_forward(0);USART_transmitByte(52);delay_ms(delay_debug*2);}//Print 4
						}
						if (DEBUG) USART_transmitByte(53); //Print 5
						turbo=20;
						rescue_estat_actual=3;
					}
				}//Fi strategy
				break;
			case 2:
				//move_robot(velocitat,velocitat,BACKWARD,36);//Robot cap enrere 2 cm
				if(dir_robot == 1){ //Si el robot va cap a fora del circuit
					turbo=30;
					move_robot(-velocitat,velocitat,FORWARD,185);
					dir_robot=0;
					while (finding_line((velocitat+20)*0.55,velocitat+20,FORWARD) == 0){};
					Motor_right_forward(0);Motor_left_forward(0);

					move_robot(velocitat,velocitat*0.8,FORWARD,180);
					if (DEBUG) {USART_transmitByte(48);delay_ms(delay_debug);}// Print 0
					while (finding_line(-velocitat/2,velocitat/2,FORWARD) == 0){};
					Motor_right_forward(0);Motor_left_forward(0);
					if (DEBUG) {delay_ms(delay_debug);}
					turbo=0;
				}else{ //Si el robot va cap endins del circuit i s ha acabat segment

					move_robot(velocitat,velocitat,BACKWARD,180);
					//delay_ms(3000);
					move_robot(velocitat,-velocitat,FORWARD,170); //Girem sobre si mateix sentit antihorari uns 60graus
					//delay_ms(3000);
					//if (DEBUG) delay_ms(5000);
					dir_robot=1; //Canviem direccio. Ara anirem de dins cap a fora
					//while (finding_line(velocitat,velocitat*0.55,FORWARD) == 0){}; //Recte fins trobar la linia
					while (finding_line(velocitat*0.8,velocitat,FORWARD) == 0){};
					Motor_right_forward(0);Motor_left_forward(0);
					if (DEBUG) {USART_transmitByte(50);delay_ms(delay_debug);} //Print 2
					delay_ms(10);
					move_robot(velocitat*0.8,velocitat,FORWARD,135);

					if (DEBUG) {USART_transmitByte(51);delay_ms(delay_debug);} //Print 3
					move_robot(velocitat/2,-velocitat/2,FORWARD,120); //Seguretat
					while (finding_line(velocitat/2,-velocitat/2,FORWARD) == 0){};
					Motor_right_forward(0);Motor_left_forward(0);

					if (DEBUG) {USART_transmitByte(52);delay_ms(delay_debug);} //Print 4

				}
				if (DEBUG) USART_transmitByte(53); //Print 4
				turbo=0;
				rescue_estat_actual=1;
				break;
			case 3:
				follow_line_fast();
				break;
			default:
				_delay_us(600);
				Motor_right_forward(0);
				Motor_left_forward(0);
				break;

		}
		/*if (telemetry_enabled){ //DEBUG
			USART_transmitByte(13); //Carriage return
		}*/
		_delay_us(20);
	}
}