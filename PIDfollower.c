#include <avr/io.h>

#include "PIDfollower.h"
#include "USART_and_telemetry.h"
#include "Motor.h"
#include "Main.h"
/*#include <pololu/orangutan.h> //Si volem usar coses de les llibreries pololu de la barra de sensors o altres cal fer un include i instal·lar al SO la llibreria de libpololu_avr
//Llegir https://www.pololu.com/docs/0J20/all#3.k
//He modificat el Makefile per incloure en ldflags la llibreria de pololu

//unsigned int sensors[5]; // an array to hold sensor values
unsigned char qtr_analog_pins[] = {0,1,2,3,4,5};
unsigned int sensors[6];

void init_pololu_bar_sensors(void){
	qtr_analog_init(qtr_analog_pins, 6, 4, 255); //6 sensors 4 mostres per sensor No emitter pin 255
	// optional: wait for some input from the user, such as  a button press

	// then start calibration phase and move the sensors over both
	// reflectance extremes they will encounter in your application:
	int i;
	for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
	{
		qtr_calibrate(1); //QTR_EMITTERS_ON
		delay(20);
	}
}
unsigned int QTR_obtenir_errorp(){
	int position=read_line(sensors,1);//QTR_EMITTERS_ON
  return position;
}*/

uint8_t PID_obtenir_errorp(void)
{
	uint16_t errorp=0;
	//int lectura_analogica=0;
	static int ultimo_errorp=0;
	uint16_t contador_sensor=0;
	uint16_t acumula_sensor=0;
	uint16_t aux = 0;
	//if(((PINC & 0x04) != 0) && ((PINC & 0x02) != 0))
	if (_SENSOR_D0 && _SENSOR_I0){//Central sensors on track
		errorp=28;
		return((uint8_t) errorp);
  }


	#ifdef _SENSOR_I3
			if (_SENSOR_I3) //I3  -7 on track
			{
					errorp = errorp - 38;
					contador_sensor++;
			}
	#endif

	//if((PIND & 0x10) != 0) //I2 PD4 -7
	if (_SENSOR_I2){

		//errorp = errorp - 27;
		errorp += 0;
		acumula_sensor+=1000;
		contador_sensor++;
	}


	//if((PINC & 0x01) != 0) //I1 PC0 -3
	if (_SENSOR_I1){
		//errorp = errorp - 15;
		aux=readADC(0);
		errorp += 15*aux;
		acumula_sensor += aux;
		contador_sensor++;
	}

	//if((PINC & 0x02) != 0) //I0 PC1 -1
	if (_SENSOR_I0){
		//errorp = errorp - 4;
		aux=readADC(1);
		errorp += 26*aux;
		acumula_sensor += aux;
		contador_sensor++;
	}

	//if((PINC & 0x04) != 0) //D0 PC2 +1
	if (_SENSOR_D0){
		//errorp = errorp + 4;
		aux=readADC(2);
		errorp += 30 * aux;
		acumula_sensor += aux;
		contador_sensor++;
	}

	//if((PINC & 0x08) != 0) //D1 PC3 +3
	if (_SENSOR_D1){
		//errorp = errorp + 15;
		aux=readADC(3);
		errorp += 41*aux;
		acumula_sensor +=aux;
		contador_sensor++;
	}


	//if((PIND & 0x80) != 0) //D2 PD7 +7
	if (_SENSOR_D2){
		//errorp = errorp + 27;
		aux=readADC(4);
		errorp += 56*aux;
		acumula_sensor +=aux;
		contador_sensor++;
	}

	#ifdef _SENSOR_D3
			if (_SENSOR_D3) //D3  +7 on track
			{
					errorp = errorp + 38;
					contador_sensor++;
			}
	#endif

	//uart_putchar(errorp);
	//uart_putchar('\n');

	if(contador_sensor != 0)
	{
		//errorp = errorp / contador_sensor;
		errorp=errorp/acumula_sensor;
		ultimo_errorp = errorp;
		//return(Kp * (int)errorp);
		return ((uint8_t) errorp);
	}
	else
	{
		if(ultimo_errorp < 28)
			//errorp = -50;
			errorp = 0;
		else
			//errorp = 50;
			errorp = 56;

		ultimo_errorp = errorp;
		//return((int)errorp * Kp);
		return ((uint8_t) errorp);
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
		error = 4;
	//else if((PINC & 0x02) != 0) //I0 PC1 -1
	else if (_SENSOR_I0)
		error = -4;
	//else if((PINC & 0x08) != 0) //D1 PC3 +3
	else if (_SENSOR_D1)
	  error=15;
	//else if((PINC & 0x01) != 0) //I1 PC0 -3
	else if (_SENSOR_I1)
		error = -15;
	//else if((PIND & 0x80) != 0) //D2 PD7 +7
	else if (_SENSOR_D2)
		error = 27;
	//else if((PIND & 0x10) != 0) //I2 PD4 -7
	else if (_SENSOR_I2)
		error = -27;
	#ifdef _SENSOR_D3
	else if (_SENSOR_D3)
		error = 38;
	#endif
	#ifdef _SENSOR_I3
	else if (_SENSOR_I3)
		error = -38;
	#endif

	else
		{
			if (error_old < 0)
				error = -50;
			else if(error_old > 0)
				error = 50;
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
		errord = (Kd*10)*(diferencia)/tic; //error mig
		errord_old = errord;
		tic_old=tic;
		tic=0;
	}

	error_old = error;
	return(errord);
}
void PID_line_following(int direction){ //0 forward,1 backwards
	int errort=0;
	static int proporcional_old = 0;
	int speed_M1=0;
	int speed_M2=0;
	int proporcional = PID_obtenir_errorp()-28;
	//int derivatiu = obtenir_errord();
	int velocitat_incrementada=velocitat+turbo;

	if (velocitat_incrementada >254) velocitat_incrementada=255;
	//if (proporcional==1 || proporcional==-1) proporcional=0; //Little HACK

	errort = (Kp * proporcional + Kd * (proporcional-proporcional_old))/12;
  proporcional_old=proporcional;
	/*
	if(errort > velocitat_incrementada)
		errort = velocitat_incrementada;
	else if(errort < - velocitat_incrementada)
		errort = - velocitat_incrementada;

	int total_esquerra=velocitat_incrementada+errort;
	int total_dreta=velocitat_incrementada-errort;

	if (total_esquerra>=255) total_esquerra=255;
	else if (total_esquerra<=-255) total_esquerra=-255;

	if (total_esquerra>=0)
		Motor_left_forward(total_esquerra);     //Motor esquerre.
	else
	  Motor_left_reverse(total_esquerra*(-1));

	if (total_dreta>=0)
		Motor_right_forward(total_dreta);     //Motor dret.
  else
	  Motor_right_reverse(total_dreta*(-1));
  */
	if(errort>0){
		speed_M1=velocitat_incrementada - errort;
		speed_M2=velocitat_incrementada;
		//if (cont_corba>=3) Motor_right_reverse(speed_M2);
		//else
		if (direction==0){//FORWARD
    			Motor_left_forward(speed_M2);     //Motor esquerre.
			if (speed_M1<=0){
				Motor_right_reverse((speed_M2*curve_correction)/100);     //Motor dret.
			}else{
				Motor_right_forward(speed_M1);     //Motor dret.
			}
		}else{
			Motor_right_reverse(speed_M2);     //Motor dret.
    			Motor_left_reverse(speed_M1);     //Motor esquerre.
		}

	}else if(errort<0){
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
	}else{

		speed_M1=velocitat_incrementada;
		speed_M2=velocitat_incrementada;
		if (direction==0){
    			Motor_left_forward(speed_M2);
        		Motor_right_forward(speed_M1);
		}else{
			Motor_left_reverse(speed_M2);
        		Motor_right_reverse(speed_M1);
		}
	}

	if (telemetry_enabled){
			USART_transmitByte(proporcional >> 8);
			USART_transmitByte(proporcional & 0xFF);
			//USART_transmitByte(derivatiu >> 8);
			//USART_transmitByte(derivatiu & 0xFF);
			USART_transmitByte(speed_M2);
			USART_transmitByte(speed_M1);

	}

    TIFR1 |= (1<<OCF1A);
}
