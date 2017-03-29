#include <avr/io.h>
#include <util/delay.h>

#include "../Main.h"
#include "../PIDfollower.h"
#include "../Motor.h"

int ticks_fora_circuit=0; //Per corregir comportament anomal del robot quan llig punt roig del centre dels segments. A vegades lectures -9 o 9 com si estiguerem fora del circuit
int rescue_estat_actual=0; //Per mantindre l estat en la prova de Rescat

//Check if can is black. 1 means black, 0 means white
char es_negre(){
	int sharpIR=200;
 	int threshold=17; //Below this value is black
	sharpIR=readADC(4); //Read IR sensor.Be sure to be less than 3cm from target
	//char is_black=0; //No by default

	if (sharpIR<threshold){
		return 1; //Black
	}else{ //Could be white or black but we are not aligned properly

		/*move_robot(g_velocitat/2,-g_velocitat/2,FORWARD,5); //Seguretat
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(g_velocitat/2,-g_velocitat/2,FORWARD,5);
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(-g_velocitat/2,g_velocitat/2,FORWARD,15);
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(-g_velocitat/2,g_velocitat/2,FORWARD,5);
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(g_velocitat/2,-g_velocitat/2,FORWARD,10); //Tornem al mig
		*/
		return 0; //White
	}
}

int finding_line(int speedM1, int speedM2, int direction){
	int proporcional = PID_obtenir_errorp();
	if (proporcional==9 || proporcional == -9){
			if (direction==0){
				if (speedM1>=0) Motor_right_forward(speedM1);else Motor_right_reverse(-speedM1);
				if (speedM2>=0) Motor_left_forward(speedM2);else Motor_left_reverse(-speedM2);
			}else{
				Motor_right_reverse(speedM1);
				Motor_left_reverse(speedM2);
			}
			return 0;

	}else{
			//Sensors centrals
			if (proporcional>=-1 && proporcional<=1){
				return 2;
			//Sensors esquerre
			}else if (proporcional>-9 && proporcional<-1){
				return 3;
			//Sensors dreta
			}else{
				return 1;
			}
	}
	_delay_us(600);
}

//Trobem creuament X i parem
void follow_line_until_crossroad(int crossroad_number){
	int crossroads_found=0;
	int proporcional = 0;
    int beginning_crossroad_found=0;
	int times_out=0;//Vegades fora
	int lectures_seguides_creuament=0;

	while(1){
		proporcional=PID_obtenir_errorp();

		//Si topem encreumant. Ho sabem pq estem en linia i _SENSOR_D1 i D2 activats al mateix temps
		if ( (proporcional > -9 && proporcional <9) && (_SENSOR_D1 && _SENSOR_D2)){
			times_out=0;
			lectures_seguides_creuament+=1;
			if (lectures_seguides_creuament>5) beginning_crossroad_found=1;
			//if (g_DEBUG) {USART_transmitByte(56);_delay_us(150);} //Print 8
		//Estem fora de linia
		}else if (proporcional==9 || proporcional == -9){
			lectures_seguides_creuament=0;
			times_out+=1;
			if (beginning_crossroad_found == 1){
				//if (g_DEBUG) {USART_transmitByte(51);_delay_us(150);} //Print 3
				//while (finding_line(-g_velocitat/3,g_velocitat/3,FORWARD) == 0){};//Girem a dreta fins trobar linia
				crossroads_found+=1;
				beginning_crossroad_found=0;
				if (crossroads_found == crossroad_number){
					//if (g_DEBUG) {USART_transmitByte(54);_delay_us(150);} //Print 6
					break; //Eixim del bucle
				}
			}
			//if (g_DEBUG) {USART_transmitByte(52);_delay_us(150);} //Print 4
			if (times_out>150) break; //Si estem fora mes del compte
		//Estem en línia pero _SENSOR_D1 i D2 no estan activats al mateix temps
		}else{

			if (beginning_crossroad_found){
				lectures_seguides_creuament=0;
				_delay_us(250);
				proporcional=PID_obtenir_errorp();
				if  (proporcional>-9 && proporcional < 9){ //Dins linia
					beginning_crossroad_found=0;
					crossroads_found+=1;
					while (finding_line(-g_velocitat/3,g_velocitat/3,FORWARD) == 0){};//Girem a dreta fins trobar linia
					//if (g_DEBUG) {USART_transmitByte(55);_delay_us(150);} //Print 7
				}
				//if (g_DEBUG) {USART_transmitByte(53);_delay_us(150);} //Print 5
				if (crossroads_found == crossroad_number){
					//if (g_DEBUG) {USART_transmitByte(54);_delay_us(150);} //Print 6
					break; //Eixim del bucle
				}
			}
		}
		PID_line_following(FORWARD); //Forward
		_delay_us(850);
		//_delay_ms(1);
	}

	//rescue_estat_actual=2;
}

void follow_line_fast(void){
	int proporcional = PID_obtenir_errorp();
	if (proporcional==9 || proporcional == -9){
		ticks_fora_circuit+=1;
		if (ticks_fora_circuit>50){ //Si ens hem eixit de veritat. HACK. A vegades tenim lectures de 9 i -9 en els punts centrals rojos dels segments de la prova de rescat
			Motor_right_forward(0);
			Motor_left_forward(0);
			rescue_estat_actual=2;
		}
	}else{
		ticks_fora_circuit=0;
		PID_line_following(FORWARD); //Forward
		_delay_ms(1);
	}
}

void adjust_speed_to_a_threshold(int current_read,int desired_read){
	int inc_speed=0;

	if (current_read<desired_read){
		inc_speed = (desired_read-current_read)/7;
	}else{
		inc_speed = -(current_read-desired_read)/7;
	}
	g_velocitat+=inc_speed;

}

void move_robot(int speedM1,int speedM2, int direction_, int encoder_steps_){
	//reset_encoder();
	//encoder_counts=0;
	//print_encoder_counts();
	int cont_wheel=0;
	char old_state=0; 
	if((PIND & 0x04) != 0) old_state=1;
	else old_state=0;

	
	if(direction_==1){
		Motor_right_reverse(speedM1);
		Motor_left_reverse(speedM2);
	}else{
		if (speedM1>=0) Motor_right_forward(speedM1);else Motor_right_reverse(-speedM1);
		if (speedM2>=0) Motor_left_forward(speedM2);else Motor_left_reverse(-speedM2);
	}
	//_delay_us(10);
	while(cont_wheel<encoder_steps_){ //We read continously PD2 changes
		if((PIND & 0x04) != 0){ //HIGH
			if (old_state == 0) {
				cont_wheel++;
				old_state=1;
			}
		}else{ //LOW
			if(old_state == 1){
				cont_wheel++;
				old_state=0;
			}
		}
	}
	//print_encoder_counts();
	Motor_right_forward(0);
	Motor_left_forward(0);
}