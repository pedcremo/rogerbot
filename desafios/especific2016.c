#include <avr/io.h>
#include <util/delay.h>

#include "common.h"
#include "../Main.h"
#include "../Motor.h"
#include "../USART_and_telemetry.h"
#include "../PIDfollower.h"


//FALTA DESCRIPCIO

void rescue_state_machine_2016(){

		rescue_estat_actual=0;

		int pots_rescatats=0;
		int lectures_negre=0;
		int lectures_blanc=0;
		int rescue_old_estat=9;
		int proporcional = 0;
		int tics_fora = 0;

		while (1){
			if (g_DEBUG && (rescue_estat_actual != rescue_old_estat)){ //g_DEBUG
					USART_transmitByte(83); //S
					USART_transmitByte(61); //=
					USART_transmitByte(48+rescue_estat_actual);
					USART_transmitByte(32); //Space
					USART_transmitByte(13); //Carriage return
			}
			rescue_old_estat=rescue_estat_actual;

			switch(rescue_estat_actual){
				case 0: //Estat solament executat a l'inici per fer que el robot trobe la linia

					if (finding_line(g_velocitat/2,g_velocitat,FORWARD)==0) rescue_estat_actual=0;
					else {
						rescue_estat_actual=1;
					}

					break;
				case 1: //Robot encarat en la linia recta on conflueixen tots els encreuaments
					//Motor_right_forward(0);Motor_left_forward(0);//STOP
					//delay_ms(300);
					tics_fora=0;
					move_robot(g_velocitat,g_velocitat/2,FORWARD,180); //Moure robot avant 100 pasos d encoder
					proporcional=finding_line(-g_velocitat/5,-g_velocitat/2,FORWARD);
					while (tics_fora<50){//Enrere fins que els dos sensors d enmig
						proporcional=finding_line(-g_velocitat/5,-g_velocitat/2,FORWARD);
						if (proporcional==2){//els 2 sensors centrals
							tics_fora+=1;
						}
					};

					rescue_estat_actual=2;
					break;
				case 2: //BUG PROVEN. Cada vegada que emprenen la recta dels creuaments parem en el creuament marcat per pots_rescatats+1
					//g_velocitat=100;
					follow_line_until_crossroad(pots_rescatats+1);
					rescue_estat_actual = 3;
					break;

				case 3: // Col·loquem robot perpendicular al encreuament i seguim linia fins final


					move_robot(-g_velocitat,g_velocitat,FORWARD,100);
					if (g_DEBUG) {USART_transmitByte(48);_delay_us(150);} //Print 0
					tics_fora=0;
					proporcional=finding_line(-g_velocitat/2,g_velocitat/2,FORWARD);
					while (tics_fora<30){//Girem a dreta fins trobar linia
						proporcional=finding_line(-g_velocitat/2,g_velocitat/2,FORWARD);
						if (proporcional==2){//els 2 sensors centrals
							tics_fora+=1;
						}
					}

					if (g_DEBUG) {USART_transmitByte(49);_delay_us(150);} //Print 1
					proporcional=PID_obtenir_errorp();
					while (proporcional != 9 || proporcional != -9){ //Mentres no estiguem fora de linia
						proporcional=PID_obtenir_errorp();

						if (proporcional == 9 || proporcional == -9) {
							tics_fora+=1;
							if (tics_fora>50) break;
						}
						PID_line_following(FORWARD); //Seguiment de linia Forward
						_delay_us(850);
						if (g_DEBUG) {USART_transmitByte(50);_delay_us(150);} //Print 2
						//if (g_DEBUG) {USART_transmitByte(50);delay_ms(1);} //Print 2
					};
					tics_fora=0;
					rescue_estat_actual=4;
					break;

				case 4://Recorregut de 50 cm des de que acaba la linia de pot fins que deixem pot blanc en primera franja negra o negre en la segona i tornem per la linia lateral guia al punt e partida
					tics_fora=0;
					lectures_negre=0;
					lectures_blanc=0;
					int COLOR=0; //0 blanc,1 negre
					int dx=0;
					//move_robot(g_velocitat,g_velocitat,FORWARD,300);
					proporcional=finding_line(g_velocitat,g_velocitat,FORWARD);
					while (proporcional == 0){
						_delay_ms(1);
						if (es_negre()){
							lectures_negre+=1;
							proporcional=finding_line(g_velocitat,g_velocitat,FORWARD);
						}else{
							lectures_blanc+=1;
							if ((lectures_blanc >lectures_negre) && (lectures_blanc>=190)){
								proporcional=finding_line(45,45,FORWARD);
							}else{
								proporcional=finding_line(g_velocitat,g_velocitat,FORWARD);
							}
						}

					}
					if (lectures_negre>lectures_blanc){ //Si pot negre
						COLOR=1;//NEGRE
						if (g_DEBUG) {USART_transmitByte(49);_delay_us(150);} //Print 1
						while(finding_line(g_velocitat,g_velocitat,FORWARD)!=0){}//Mentres estem en zona negra
						move_robot(g_velocitat,g_velocitat,FORWARD,400);
						while(finding_line(45,45,FORWARD)==0){}//Mentres estem en zona blanca

					}else{ //Si blanc parem
						if (g_DEBUG) {USART_transmitByte(48);_delay_us(150);} //Print 0
						//g_velocitat=100;
						//move_robot(-g_velocitat,-g_velocitat,FORWARD,100);
						//while(1) {Motor_right_forward(0);Motor_left_forward(0);}//STOP
					}

					pots_rescatats+=1;
					if (pots_rescatats>=8) pots_rescatats=0;
					//move_robot(g_velocitat,-g_velocitat,FORWARD,300); //Gira 90 graus
					if (g_DEBUG) {USART_transmitByte(50);_delay_us(150);} //Print 2
					if (proporcional==3){//Entrem amb sensors esquerre primer
						dx = -10;

					}else if (proporcional==1){ //Entrem amb sensors de la dreta primer
						dx = 65;
					}else{
						dx = 15;
					}

					if (COLOR==0){//BLANC
						move_robot(-g_velocitat/2,g_velocitat/2,FORWARD,210+dx); //90 graus
					}else{
						move_robot(-g_velocitat/2,g_velocitat/2,FORWARD,210+dx-(pots_rescatats*5)); //90 graus
						/*move_robot(-g_velocitat/2,g_velocitat/2,FORWARD,310+dx); //90 graus
						if (pots_rescatats>1){
							move_robot(g_velocitat*0.9,g_velocitat,FORWARD,500);
						}*/
						//move_robot(-g_velocitat/2,g_velocitat/2,FORWARD,365+dx-(pots_rescatats*5)); //90 graus
					}
					//while (finding_line(-g_velocitat/2,g_velocitat/2,FORWARD) != 0){};//Girem a dreta fins trobar blanc
					if (g_DEBUG) {USART_transmitByte(51);_delay_us(150);} //Print 3

					if (pots_rescatats>=3) {
						move_robot(g_velocitat*0.85,g_velocitat,FORWARD,250);
						move_robot(g_velocitat,g_velocitat*0.85,FORWARD,300);
						while (finding_line(g_velocitat*1.15,g_velocitat,FORWARD) == 0){}; //Avancem fins linia guia lateral
						Motor_right_forward(0);
						Motor_left_forward(0);

						move_robot(g_velocitat*0.7/3,g_velocitat/3,FORWARD,10);
					}else{
						while (finding_line(g_velocitat,g_velocitat,FORWARD) == 0){}; //Avancem fins linia guia lateral
						Motor_right_forward(0);
						Motor_left_forward(0);

						move_robot(g_velocitat*0.7/3,g_velocitat/3,FORWARD,10);
					}

					while (finding_line(-g_velocitat/2,g_velocitat/2,FORWARD) == 0){};//Girem a dreta fins trobar linia

					proporcional=PID_obtenir_errorp();
					while (tics_fora < 150){ //Mentres no estiguem centrats
						proporcional=PID_obtenir_errorp();
						if (proporcional==0 ) tics_fora+=1;
						PID_line_following(FORWARD); //Seguiment de linia Forward
						_delay_ms(1);

					}
					if (g_DEBUG) {USART_transmitByte(52);_delay_us(150);} //Print 4

					tics_fora=0;
					while (tics_fora<30){ //Mentres no estiguem fora de linia
						proporcional=PID_obtenir_errorp();

						if (proporcional == 9 || proporcional == -9) {
							tics_fora+=1;
							//if (tics_fora>50) break;
						}
						PID_line_following(FORWARD); //Seguiment de linia Forward
						_delay_us(600);
						if (g_DEBUG) {USART_transmitByte(50);_delay_us(150);} //Print 2
						//if (g_DEBUG) {USART_transmitByte(50);delay_ms(1);} //Print 2
					};

					Motor_right_forward(0);
					Motor_left_forward(0);
					_delay_ms(500);
					move_robot(g_velocitat/2,g_velocitat/2,FORWARD,60); //Avant una mica
					while (finding_line(-g_velocitat/3,g_velocitat/3,FORWARD) != 1){};//Girem a dreta fins trobar linia

					rescue_estat_actual=2;
					break;
				default:
					//move_robot(0,0,FORWARD,300); //PARA
					Motor_right_forward(0);
					Motor_left_forward(0);
					break;
			}
		}

}