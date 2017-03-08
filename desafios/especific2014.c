#include <avr/io.h>
#include <util/delay.h>

#include "common.h"
#include "../Main.h"
#include "../Motor.h"


//State finite machine to solve rescue contest 2014
//Cercle on calia posar tots els pots negres al mig


void rescue_state_machine_2014(){
	while ( 1 )
	{
		switch(rescue_estat_actual){
			case 0:
				if (finding_line(velocitat,velocitat,0)==0) rescue_estat_actual=0;
				else rescue_estat_actual=1;
				break;
			case 1:
				follow_line_fast();
				break;
			case 2:
				move_robot(velocitat,velocitat*0.6,1,680); //Menegem robot enrere 200 ms
				delay_ms(3000);
				while (finding_line(velocitat*0.6,velocitat,1)== 0){};
				rescue_estat_actual=0; //Canvi d'estat
				break;
			default:
				_delay_us(600);
				Motor_right_forward(0);
				Motor_left_forward(0);

		}
		_delay_us(400);
	}
}