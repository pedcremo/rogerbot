#include <avr/io.h>

#include "PIDfollower.h"
#include "USART_and_telemetry.h"
#include "Motor.h"
#include "Main.h"

// formula is:
//
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
/*uint16_t llegir_barra_sensors(){

	//uint8_t aux=0;
	uint8_t i=0;
	float sensor_locations[6]={0,114.5,214.5,285.5,393,500}; //CNY70 sensors are not distributed equidistantly so we need these calculations
	uint8_t numSensorsActivated = 0;
	static uint32_t ultima_lectura=0;
	uint32_t lectura_actual;
	uint8_t cont=1;
	uint32_t acum=0;

	if (_SENSOR_I2) {
		sensors[0] = 254;
		numSensorsActivated+=1;
		acum+=sensors[0];
	}else sensors[0] = 0;

	for (i=0;i<4;i++){
		sensors[i+1] = readADC8(i);
		if (sensors[i+1]<35) {
			sensors[i+1]=0;
		}else{
			if (sensors[i+1]>230) {
				sensors[i+1]=254;
			}
			numSensorsActivated+=1;
			acum+=(uint32_t) sensors[i+1]* sensor_locations[cont];
		}
		cont++;
	}

	if (_SENSOR_D2)	{
		sensors[5] = 254;
		numSensorsActivated+=1;
		acum+=(uint32_t) sensors[5]*500;
	}else sensors[5] = 0;

	if (numSensorsActivated>0){
		// formula is:
		//
		//    0*value0 + 1*value1 + 2*value2 + ...
		//   --------------------------------------------
		//         value0  +  value1  +  value2 + ...
		lectura_actual=acum/(sensors[0]+sensors[1]+sensors[2]+sensors[3]+sensors[4]+sensors[5]);
		ultima_lectura=lectura_actual;
	}else{
		if (ultima_lectura<250)
			lectura_actual=0;
		else
			lectura_actual=500;
	}
	return (uint16_t) lectura_actual;
}*/

void update_sensors(){

    //uint8_t aux=0;
    uint8_t i=0;
    if (_SENSOR_I2) {sensors[0] = 254;}

    for (i=0;i<4;i++){
        sensors[i+1] = readADC8(i);
    }
    if (_SENSOR_D2) {sensors[5] = 254;}
}

uint16_t llegir_barra_sensors(){
   update_sensors();
   /*
   * Esta variable guarda la posicion de la linea calculada
   */
  uint16_t posicion = 0;

  /*
   * Esta variable sirve para trackear la ultima posicion de linea buena antes de que se perdiera
   */
  static uint16_t posicionAnterior = 0;

  /* 
   *  En el siguiente codigo vamos a calcular la posicion de la linea
   *  Los sensores miden entre 0 y 1023
   *  Esto nos da un valor de linea teorico minimo de -3*1023 y uno maximo de 3*1023
   */

  uint8_t numeroSensoresDetectando = 0;
  if (_SENSOR_I2)
    numeroSensoresDetectando++;
  if (sensors[1] > 100)
    numeroSensoresDetectando+=2;
  if (sensors[2] > 100)
    numeroSensoresDetectando+=2;
  if (sensors[3] > 100)
    numeroSensoresDetectando+=2;
  if (sensors[4] > 100)
    numeroSensoresDetectando+=2;
  if (_SENSOR_D2)
    numeroSensoresDetectando++;
  if (numeroSensoresDetectando > 1)
  {
    posicion = (-4 * (sensors[0]*4)) + (-2 * (sensors[1]*4)) + (-1 * (sensors[2]*4)) + (1 * (sensors[3]*4))+ (2 * (sensors[4]*4))+ (4 * (sensors[5]*4));
    posicionAnterior = posicion;
  }
  else
  {
    posicion = posicionAnterior * 11 / 10; //Esto sirve para dar un valor un poco mayor al extremo cuando perdemos la linea
  }
  //NOTA: Aqui divido por 100 para tener valores entre -40 y 40, siendo esto una resolucion de unos 0.6mm, pero es posible que otro valor sea mejor
  return posicion/100;
}
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
	//Si tilt activat
	//clearBit(DDRB,2);//set PB2 as digital input
	//clearBit(PORTB,2); //set PB2 LOW  (HIGH IMPEDANCE)
	/*if 	(bit_is_set(PINB,2)) {
		velocitat_incrementada=velocitat;
	}else{
		velocitat_incrementada=velocitat/3;
	}*/

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

void PID_line_followingNEW(int direction){ //0 forward,1 backwards

	int speed_M1=0;
	int speed_M2=0;
	uint16_t proporcional = 0;
	uint16_t derivative = 0;
	static uint16_t last_proporcional = 0;

	int velocitat_incrementada=velocitat+turbo;

	uint16_t position = llegir_barra_sensors();

	// The "proportional" term should be 0 when we are on the line.
	proporcional = position - 250;
	derivative = proporcional - last_proporcional;

	// Remember the last position.
	last_proporcional = proporcional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the right.  If it is a negative number, the robot will
		// turn to the left, and the magnitude of the number determines
		// the sharpness of the turn.
		int power_difference = (int) proporcional*100/Kp + derivative*(Kd/3);

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.

		if(power_difference > velocitat_incrementada)
			power_difference = velocitat_incrementada;
		if(power_difference < -velocitat_incrementada)
			power_difference = -velocitat_incrementada;

		if(power_difference < 0){
			Motor_left_forward(velocitat_incrementada+power_difference);
			Motor_right_forward(velocitat_incrementada);

		}else{
			Motor_left_forward(velocitat_incrementada);
			Motor_right_forward(velocitat_incrementada-power_difference);
		}

	if (telemetry_enabled){
			USART_transmitByte(proporcional >> 8);
			USART_transmitByte(proporcional & 0xFF);
			USART_transmitByte(derivative >> 8);
			USART_transmitByte(derivative & 0xFF);
			USART_transmitByte(speed_M2);//Motor esquerre
			USART_transmitByte(speed_M1);//Motor dret

	}

    TIFR1 |= (1<<OCF1A);
}
