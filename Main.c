//Robot rogerbot by Pere Crespo
// Baby Orangutan frequency (20MHz)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

//My rogerbot libraries
#include "Main.h"
#include "Motor.h"
#include "PIDfollower.h"
#include "SensorBar.h"
#include "USART_and_telemetry.h"
#include "desafios/common.h"
#include "desafios/especific2014.h"
//#include "desafios/especific2015.h"
//#include "desafios/especific2016.h"


//Start button. Interruptor. Inputs.
#define PULSADOR PORTB0

//PB2

/*********** Ajust comportament robot per defecte *********/
/***** Variables globals ********/
//Constants Regulador PD 30:1 HP
//uint8_t cont_corba=0;
uint8_t g_sensors[6];
uint8_t g_Kp = 12;  //48 lf 18 pots
uint8_t  g_Kd = 78;//225 lf 28 pots multiplicador 100
volatile uint8_t g_velocitat = 190; //225 lf 95 pots max 255
uint8_t g_telemetry_enabled = 0; //1 enabled, 0 disabled
//V 7.39 vel 100 g_Kp 20 g_Kd 700 9,2 segons pista taller rogerbot1
char g_strategy = 'a'; //a -> By default line following using interrupts, b -> pots rescue
uint8_t g_curve_correction = 0; //0 Means we brake internal wheel, other value means we change wheel direction backward a percentage of the max speed
volatile char g_start=0;
volatile uint8_t g_turbo = 0; //Increment de la g_velocitat que posem en certs moments
uint16_t compass_direction_to_follow = 0;
/* No globals */
uint8_t g_DEBUG = 0; //1 enabled, 0 disabled

//volatile int encoder_counts=0; //Check wheel encoder counts

int main( void )
{
	char pulsador = 1;
	int voltage = 0;
	init_ports();
	Motor_init(); //Set up motors ports and pwm

	// Set the baud rate to BAUD bits per second.  Each byte takes ten bit
	// times, so you can get at most 7200 bytes per second at 115200 speed.
	// We transmit 32bits in 0,27 ms
	USART_init(); //Enables usart comunication with rogerbot using  interrupts (bluetooth mainly)
	init_ADC(); //Set up ADC

	sei(); //As we use interruptions for USART communication we enable them

	//Load Rogerbot settings from eeprom (speed, Kp, Kd ...)
	load_eeprom_settings();
	delay_ms(2000);
	calibrate_sensors();
	//init_encoders();
	while(g_start == 0)
	{
		pulsador = PINB & (1<<PULSADOR);
		if (pulsador == 0) g_start=1;
        //if (g_start==1) break;
	}
	delay_ms(5000);//Esperem 5 segons després de la polsació boto

	voltage = readADC(6); //Returns analog value that multiplied by 0,004887585533 * 2  equals voltage

	/*
	  If g_telemetry_enabled we disable usart interrupts and all usart communications become syncronous. We will be unable
	  to change speed, stop , g_start the robot or any other interaction
	  In some situations or robot strategies g_telemetry_enabled means we are in debug mode
	*/
	if (g_telemetry_enabled || g_DEBUG){
			USART_disable_interrupts();
	}
	//a=line following using interrupts, b= rescue pots negres, c=rescue pots negres i blancs, d=rescue pots negres i blancs (CEC)
	if (g_strategy=='a'){ //Line following using timer interrupt. Every ms we check sensors
		Motor_acceleracio_progressiva();
		inicializar_timer1();// We use the timer also as millis counter in other strategies

	}else if (g_strategy=='b'){ //Rescue g_strategy. Only Black cans and all go inside
		//init_encoders();
		adjust_speed_to_a_threshold(voltage,770);
		rescue_state_machine_2014();
	}else if (g_strategy=='c' || g_strategy=='d') {//Rescue g_strategy. White cans inside, black ones outside. 'd' blind g_strategy w-b-w-b-w-b-w-b
		//init_encoders();
		delay_ms(400);
		//Read 780 means we have 7,55 v. Our ideal
		adjust_speed_to_a_threshold(voltage,770);
		//rescue_state_machine_2015();

	//Desafio robot 2016
  	}else if (g_strategy=='e'){
		//rescue_state_machine_2016();

	}else{
		//Motor_acceleracio_progressiva();
		//inicializar_timer1();// We use the timer also as millis counter in other strategies
		//prova_compass_direction();
	}

	while ( 1 )
	{

	}
	return 0;
}



uint16_t get_current_millis(){ //Timer1 16 bits
	return TCNT1;
}
void init_current_millis(){ //Timer1 16 bits every tick are 3.2 micro seconds if F_CPU 20MHZ
	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at F_CPU/64
	TCNT1 = 0; // Reset timer value
}


void delay_ms(int ms){
	int i=0;
	for (i=0;i<ms;i++){
		_delay_ms(1);
	}
}


//M1 right motor speed, M2 left motor speed, Direction FORWARD or BACKWARDS, encoder_steps number of left wheel encoder steps (every 300 steps) one wheel revolution.

/*void print_encoder_counts(){
	if (g_DEBUG){
		if (encoder_counts == 0) {
			USART_transmitByte('e');USART_transmitByte('0');USART_transmitByte('.');
		}if (encoder_counts>1000){
			USART_transmitByte('e');USART_transmitByte('1');USART_transmitByte('0');USART_transmitByte('0');USART_transmitByte('0');USART_transmitByte('.');
		}else{
			USART_transmitByte('e');
			USART_transmitByte('0'+(encoder_counts/100));
			USART_transmitByte('0'+((encoder_counts/10) % 10));
			USART_transmitByte('0'+(encoder_counts % 10));
			USART_transmitByte('.');
		}
	}
}*/

void load_eeprom_settings(void){
	uint8_t aux=0;
	//Get eeprom g_velocitat
	aux = eeprom_read_byte((uint8_t *) VELOCITAT_ADDR_EEPROM);
	if (aux>0 && aux <255) g_velocitat=aux;
	else if (g_velocitat >0 && g_velocitat <255) eeprom_write_byte((uint8_t *) VELOCITAT_ADDR_EEPROM,g_velocitat);
	//Get g_telemetry_enabled
	aux = eeprom_read_byte((uint8_t *) TELEMETRY_ENABLED_ADDR_EEPROM);
	if (aux==0 || aux==1) g_telemetry_enabled=aux;
	//GET eeprom g_Kp
	aux = eeprom_read_byte((uint8_t *) KP_ADDR_EEPROM);
	if (aux>0 && aux <255) g_Kp=aux;
	//GET eeprom g_Kd
	aux = eeprom_read_byte((uint8_t *) KD_ADDR_EEPROM);
	if (aux>=0 && aux <255) g_Kd=aux;
	//GET eeprom g_strategy (Rescue or line following)
	aux = eeprom_read_byte((uint8_t *) STRATEGY);
	if (aux>=0 && aux <255) g_strategy=aux;
	//GET eeprom g_curve_correction
	aux = eeprom_read_byte((uint8_t *) CURVE_CORRECTION);
	if (aux>=0 && aux <100) g_curve_correction=aux;

}
//Li passem el port analogic que volem llegir i ens retorna la lectura (0 .. 7)
uint16_t readADC(uint8_t channel) {
//uint16_t readADC(uint8_t channel) {
  ADMUX = (0xf0 & ADMUX) | channel;
  ADCSRA |= (1 << ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  return (ADC);//10 bits
  // return (ADCH);//8 bits
}

uint8_t readADC8(uint8_t channel) {
	// Usa como referencia AVcc (REFS0), alinea a la izda porque solo vamos a
	// usar los 8 bits mas altos (ADLAR) y selecciona el canal (p.266)
	ADMUX = _BV(REFS0) | _BV(ADLAR) | channel;

	// Inicia la conversion
	ADCSRA |= _BV(ADSC) | _BV(ADEN);

	// Espera a que acabe (TODO: espera a la IRQ?)
	do {/*
		chSysLock();
		int msg = chThdSuspendTimeoutS(&trp, TIME_INFINITE);
		(void)msg;
		chSysUnlock();
	*/
	} while (ADCSRA & _BV(ADSC)) ;

	// Devuelve la muestra
	return ADCH;
}

//Iniciem el sistema de lectura analogic-digital
void init_ADC(void){
 ADMUX |= (1 << REFS0);                  /* reference voltage on AVCC */
 //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);     /* ADC clock prescaler /8 */
 ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // prescaler = 128
 ADCSRA |= (1 << ADEN);                                 /* enable ADC */
}

/*
void reset_encoder(){
	encoder_counts=0;
}

int get_encoder_counts(){
	return encoder_counts;
}
//Interrupt Service Routine for INT0
ISR(INT0_vect)
{
  	encoder_counts++;

}*/

/*void init_encoders(){ //Interrupt 0

	//Do it in init_ports()
	//clearBit(DDRD,2);//set PB2 as digital input
	//clearBit(PORTD,2); //set PB2 LOW  (HIGH IMPEDANCE)
	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
    	// PD2 (PCINT0 pin) is now an input
    	PORTD |= (1 << PORTD2);    // turn On the Pull-up
    	// PD2 is now an input with pull-up enabled

	EICRA |= (1 << ISC00);	// Trigger INT0 on change
	EIMSK |= (1 << INT0);	// Enable INT0


	sei();				//Enable Global Interrupt

}*/

void init_ports(void)
{
   DDRD=0x6A;     //0110 1010
   PORTD=0x00;
   DDRB=0x26;     //0010 0010
   PORTB=0x00;
   DDRC=0x00;     //0000 0000
   PORTC=0x00;
}


void inicializar_timer1(void) //Configura el timer y la interrupción.
{
    OCR1A= 0x0138; //009C 0.5 ms,0138 1 ms. 0C35 10ms, 0x0271 2ms.
    TCCR1B |=((1<<WGM12)|(1<<CS11)|(1<<CS10));    //Los bits que no se tocan a 0 por defecto
    TIMSK1 |= (1<<OCIE1A);
    sei();
}


// Interrupcio de temps activada cada x milisegons (depenent de inicialiar_timer1 en current_millis emmagatzemem el num de ms
ISR(TIMER1_COMPA_vect)
{
	PID_line_followingNEW(FORWARD);
}
