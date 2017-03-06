#include <avr/io.h>
#include <util/delay.h>
#include "Motor.h"
#include "Main.h"


//static int brake_left_counts=0; //Counter used to implement abs brakes
//static int brake_right_counts=0; //Counter used to implement abs brakes

//We start from 0 to cruiser speed defined in global var velocitat in velocitat*2 ms
void Motor_acceleracio_progressiva(){
	int i=0;
	//Acceleracio poc a poc del pwm a l inici
	for (i=0;i<255;i++){
		if (i<velocitat){
			Motor_right_forward(i);	     //Motor derecho.
			Motor_left_forward(i);	    //Motor izquierdo.
		}else{
			Motor_right_forward(velocitat);Motor_left_forward(velocitat);
		}
		_delay_ms(1);
	}
}

//Funcions per controlar la velocitat i direcció dels
//motors. PWM controla la velocitat, valor entre 0-255.
void Motor_right_reverse(unsigned char pwm)
{
	//if (pwm==0) brake_right_counts++;else brake_right_counts=0;
	OCR0A = 0;
	OCR0B = pwm;
}
void Motor_right_forward(unsigned char pwm)
{
	//if (pwm==0) brake_right_counts++;else brake_right_counts=0;
	OCR0B = 0;
	/*if (brake_right_counts>70){
		//OCR0A = 20;
		Motor_right_reverse(50);
		//if (brake_right_counts>10) brake_right_counts=0;
	}else{
		OCR0A = pwm;
	}*/
	OCR0A = pwm;
}
void Motor_left_forward(unsigned char pwm)
{
	//if (pwm==0) brake_left_counts++;else brake_left_counts=0;

	OCR2B = 0;
	/*if (brake_left_counts>70){
		//OCR2A = 20;
		Motor_left_reverse(50);
		//if (brake_left_counts>10) brake_left_counts=0;
	}else{
		OCR2A = pwm;
	}*/
	OCR2A = pwm;
}
void Motor_left_reverse(unsigned char pwm)
{
	//if (pwm==0) brake_left_counts++;;else brake_left_counts=0;
	OCR2B = pwm;
	OCR2A = 0;
}

//Configuración del hardware del micro que controla los motores.
void Motor_init(void)
{
	// configure for inverted PWM output on motor control pins:
	// set OCxx on compare match, clear on timer overflow
	// Timer0 and Timer2 count up from 0 to 255
	TCCR0A = TCCR2A = 0xF3;
	// use the system clock/8 (=2.5 MHz) as the timer clock
	TCCR0B = TCCR2B = 0x02;
	// initialize all PWMs to 0% duty cycle (braking)
	OCR0A = OCR0B = OCR2A = OCR2B = 0;
	// set PWM pins as digital outputs (the PWM signals will not
	// appear on the lines if they are digital inputs)
	DDRD |= (1 << PORTD3) | (1 << PORTD5) | (1 << PORTD6);
	DDRB |= (1 << PORTB3);
}
