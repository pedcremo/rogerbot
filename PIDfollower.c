#include <avr/io.h>

#include "PIDfollower.h"
#include "USART_and_telemetry.h"
#include "Motor.h"
#include "Main.h"


int PID_obtenir_errorp(void)
{
	char errorp=0;
	static char ultimo_errorp=0;
	char contador_sensor=0;

	//if(((PINC & 0x04) != 0) && ((PINC & 0x02) != 0))
	if (_SENSOR_D0 && _SENSOR_I0){//Central sensors on track
		errorp=0;
		return(0);
  }

	#ifdef _SENSOR_I3
			if (_SENSOR_I3) //I3  -7 on track
			{
					errorp = errorp - 0x09;
					contador_sensor++;
			}
	#endif

	//if((PIND & 0x10) != 0) //I2 PD4 -7
	if (_SENSOR_I2){
		errorp = errorp - 0x08;
		contador_sensor++;
	}


	//if((PINC & 0x01) != 0) //I1 PC0 -3
	if (_SENSOR_I1){
		errorp = errorp - 0x03;
		contador_sensor++;
	}

	//if((PINC & 0x02) != 0) //I0 PC1 -1
	if (_SENSOR_I0){
		errorp = errorp - 0x01;
		contador_sensor++;
	}

	//if((PINC & 0x04) != 0) //D0 PC2 +1
	if (_SENSOR_D0){
		errorp = errorp + 0x01;
		contador_sensor++;
	}

	//if((PINC & 0x08) != 0) //D1 PC3 +3
	if (_SENSOR_D1){
		errorp = errorp + 0x03;
		contador_sensor++;
	}


	//if((PIND & 0x80) != 0) //D2 PD7 +7
	if (_SENSOR_D2){
		errorp = errorp + 0x08;
		contador_sensor++;
	}

	#ifdef _SENSOR_D3
			if (_SENSOR_D3) //D3  +7 on track
			{
					errorp = errorp + 0x09;
					contador_sensor++;
			}
	#endif

	//uart_putchar(errorp);
	//uart_putchar('\n');

	if(contador_sensor != 0)
	{
		errorp = errorp / contador_sensor;
		ultimo_errorp = errorp;
		//return(Kp * (int)errorp);
		return((int)errorp);
	}
	else
	{
		if(ultimo_errorp < 0)
			errorp = -0x09;
		else
			errorp = 0x09;

		ultimo_errorp = errorp;
		//return((int)errorp * Kp);
		return((int)errorp);
	}
}

int obtenir_errord(void)
{
	int error = 0;
	static int error_old = 0;
	static int errord=0;
	static int errord_old = 0;
	static int tic = 1;  // 1
	static int tic_old = 1; //


	int diferencia = 0;

	//if(((PINC & 0x02) != 0) && ((PINC & 0x04) != 0))
	if (_SENSOR_D0 && _SENSOR_I0 )  //Central sensors on track
		error=0;
	//else if((PINC & 0x04) != 0) //D0 PC2 +1
	else if (_SENSOR_D0) //
		error = 1;
	//else if((PINC & 0x02) != 0) //I0 PC1 -1
	else if (_SENSOR_I0)
		error = -1;
	//else if((PINC & 0x08) != 0) //D1 PC3 +3
	else if (_SENSOR_D1)
	  error=3;
	//else if((PINC & 0x01) != 0) //I1 PC0 -3
	else if (_SENSOR_I1)
		error = -3;
	//else if((PIND & 0x80) != 0) //D2 PD7 +7
	else if (_SENSOR_D2)
		error = 8;
	//else if((PIND & 0x10) != 0) //I2 PD4 -7
	else if (_SENSOR_I2)
		error = -8;

	else
		{
			if (error_old < 0)
				error = -9;
			else if(error_old > 0)
				error = 9;
		}

	//Càlcul de la velocitat mitjana de l'error.
	if (error == error_old)
	{
		tic = tic + 1;
		if(tic > 30000)
			tic = 30000;
		if(tic > tic_old)
			errord = errord_old/tic;

	}
	else
	{
		tic++;
		diferencia = error - error_old;
		errord = (Kd*100)*(diferencia)/tic; //error mig
		errord_old = errord;
		tic_old=tic;
		tic=0;
	}

	error_old = error;
	return(errord);
}
void PID_line_following(int direction){ //0 forward,1 backwards
	int errort=0;
	uint8_t Kp_aux=Kp;
	int derivatiu_aux=0;
	int speed_M1=0;
	int speed_M2=0;
	int proporcional = PID_obtenir_errorp();
	int derivatiu = obtenir_errord();
	int velocitat_incrementada=velocitat+turbo;

	if (velocitat_incrementada >254) velocitat_incrementada=255;
	if (proporcional==1 || proporcional==-1 || proporcional==0) {
			//proporcional=0; //Little HACK
			Kp_aux=Kp/2;
			derivatiu_aux=derivatiu/3;
	}else{
			Kp_aux=Kp;
			derivatiu_aux=derivatiu;
	}

	errort = (proporcional*Kp_aux) + derivatiu_aux;
	//errort = (proporcional*Kp) + derivatiu;
	/*if (proporcional>=0x09 || proporcional<=-0x09) cont_corba+=1;
	else cont_corba=0;*/

	if(errort > velocitat_incrementada)
		errort = velocitat_incrementada;
	else if(errort < - velocitat_incrementada)
		errort = - velocitat_incrementada;

	if(errort>=0){
		speed_M1=velocitat_incrementada - errort;
		speed_M2=velocitat_incrementada;
		//if (cont_corba>=3) Motor_right_reverse(speed_M2);
		//else
		if (direction==0){//FORWARD
    			Motor_left_forward(speed_M2);     //Motor esquerra.
			if (speed_M1<=0){
				Motor_right_reverse((speed_M2*curve_correction)/100);     //Motor dret.
			}else{
				Motor_right_forward(speed_M1);     //Motor dret.
			}
		}else{
			Motor_right_reverse(speed_M2);     //Motor dret.
    			Motor_left_reverse(speed_M1);     //Motor esquerre.
		}

	//}else if(errort<0){
	}else{
		speed_M1=velocitat_incrementada;
		speed_M2=velocitat_incrementada+errort;
		//if (cont_corba>=3) Motor_left_reverse(speed_M1);
		//else
		if (direction==0){//FORWARD
			Motor_right_forward(speed_M1);     //Motor dret.
			//Motor_left_forward(speed_M2);     //Motor esquerre.
			if (speed_M2<=0){
    				Motor_left_reverse((speed_M1*curve_correction)/100);     //Motor esquerre.
			}else{
						Motor_left_forward(speed_M2);     //Motor esquerre.
			}
		}else{
			Motor_left_reverse(speed_M1);     //Motor esquerre.
    			Motor_right_reverse(speed_M2);     //Motor dret.
		}
	}/*else{

		speed_M1=velocitat_incrementada;
		speed_M2=velocitat_incrementada;
		if (direction==0){
    			Motor_left_forward(speed_M2);
        		Motor_right_forward(speed_M1);
		}else{
			Motor_left_reverse(speed_M2);
        		Motor_right_reverse(speed_M1);
		}
	}*/
	if (telemetry_enabled){
			USART_transmitByte(proporcional >> 8);
			USART_transmitByte(proporcional & 0xFF);
			USART_transmitByte(derivatiu >> 8);
			USART_transmitByte(derivatiu & 0xFF);
			USART_transmitByte(speed_M2);//Motor esquerre
			USART_transmitByte(speed_M1);//Motor dret

	}

    TIFR1 |= (1<<OCF1A);
}
