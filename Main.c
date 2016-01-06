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
#include "USART_and_telemetry.h"


//Local macros
#define BV(bit)			(1 << bit)
#define setBit(byte,bit) 	(byte |= BV(bit))
#define clearBit(byte,bit) 	(byte &= ~BV(bit))
#define toggleBit(byte,bit) 	(byte ^= BV(bit))

//Start button. Interruptor. Inputs.
#define PULSADOR PORTB0

//PB2
#define PING_PORT 5

/*********** Ajust comportament robot per defecte *********/
/***** Variables globals ********/
//Constants Regulador PD 30:1 HP
//uint8_t cont_corba=0;
uint8_t Kp = 44;  //48 lf 18 pots
uint8_t  Kd = 48;//225 lf 28 pots multiplicador 100
volatile uint8_t velocitat = 198; //225 lf 95 pots max 255
uint8_t telemetry_enabled = 0; //1 enabled, 0 disabled
//V 7.39 vel 100 kp 20 kd 700 9,2 segons pista taller rogerbot1
char strategy = 'a'; //a -> By default line following using interrupts, b -> pots rescue
uint8_t curve_correction = 0; //0 Means we brake internal wheel, other value means we change wheel direction backward a percentage of the max speed
volatile char start=0;
volatile uint8_t turbo = 0; //Increment de la velocitat que posem en certs moments

/* No globals */
uint8_t DEBUG = 0; //1 enabled, 0 disabled

int rescue_estat_actual=0; //Per mantindre l estat en la prova de Rescat
int ticks_fora_circuit=0; //Per corregir comportament anomal del robot quan llig punt roig del centre dels segments. A vegades lectures -9 o 9 com si estiguerem fora del circuit
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

	//Load Rogerbot settings from eeprom (speed, kp, kd ...)
	load_eeprom_settings();
	//init_encoders();
	while(pulsador != 0)
	{
		pulsador = PINB & (1<<PULSADOR);
    		if (start==1) break;
	}

	voltage = readADC(6); //Returns analog value that multiplied by 0,004887585533 * 2  equals voltage

	/*
	  If telemetry_enabled we disable usart interrupts and all usart communications become syncronous. We will be unable
	  to change speed, stop , start the robot or any other interaction
	  In some situations or robot strategies telemetry_enabled means we are in debug mode
	*/
	if (telemetry_enabled || DEBUG){
			USART_disable_interrupts();
	}
	//a=line following using interrupts, b= rescue pots negres, c=rescue pots negres i blancs, d=rescue pots negres i blancs (CEC)
	if (strategy=='a'){ //Line following using timer interrupt. Every ms we check sensors
		Motor_acceleracio_progressiva();
		inicializar_timer1();// We use the timer also as millis counter in other strategies

	}else if (strategy=='b'){ //Rescue strategy. Only Black cans and all go inside
		//init_encoders();
		adjust_speed_to_a_threshold(voltage,770);
		rescue_state_machine();
	}else if (strategy=='c' || strategy=='d') {//Rescue strategy. White cans inside, black ones outside. 'd' blind strategy w-b-w-b-w-b-w-b
		//init_encoders();
		delay_ms(400);
		//Read 780 means we have 7,55 v. Our ideal
		adjust_speed_to_a_threshold(voltage,770);

		rescue_state_machine_2015();
	}else{
		Motor_acceleracio_progressiva();
		inicializar_timer1();// We use the timer also as millis counter in other strategies
	}

	while ( 1 )
	{
	}
	return 0;
}

void adjust_speed_to_a_threshold(int current_read,int desired_read){
	int inc_speed=0;

	if (current_read<desired_read){
		inc_speed = (desired_read-current_read)/7;
	}else{
		inc_speed = -(current_read-desired_read)/7;
	}
	velocitat+=inc_speed;

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
//Ping sensor. Ultrasound echo en PING_PORT. Retorna milimetres de distancia
int ping(){
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	setBit(DDRB,PING_PORT);//set PB2 as digital output
	clearBit(PORTB,PING_PORT); //set PB2 LOW
	_delay_us(2);
	setBit(PORTB,PING_PORT); //set PB2 HIGH
	_delay_us(5);
	clearBit(PORTB,PING_PORT); //set PB2 LOW
	//_delay_us(20);


	int pulseStart = 0;
	init_current_millis(); //Reset to 0 millis counter
	// The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
	clearBit(DDRB,PING_PORT);//set PB2 as digital input
	clearBit(PORTB,PING_PORT); //set PB2 LOW  (HIGH IMPEDANCE)
	loop_until_bit_is_set(PINB,PING_PORT);

	pulseStart = get_current_millis();

	while(get_current_millis() - pulseStart < 625)         // loop for 2 ms max
	{
  		// break out of loop when signal goes low
		if (!bit_is_set(PINB,PING_PORT)) break;
	}
	//unsigned long pulseLength = get_ticks() - pulseStart;  // sonar pulse travel time in units of 0.4 us

	pulseStart= get_current_millis() - pulseStart; //Number of ticks of 3.2 microseconds each one
    	_delay_us(200); //Short delay until next read
	// La velocitat del so es 340 m/s o 29 microsegons per centimetre o 2,9 microsegons cada milimetre.
  	// EL ping viatja en sentit d'anada i tornada, per tant per esbrinar la distancia al objecte
  	// agafem sols la mitat del recorregut
  	return (pulseStart*3.2)/2/2.9;

}

//State finite machine to solve rescue contest 2014
void rescue_state_machine(){
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
		_delay_us(400);
	}
}
//Check if can is black. 1 means black, 0 means white
char es_negre(){
	int sharpIR=200;
 	int threshold=17; //Below this value is black
	sharpIR=readADC(4); //Read IR sensor.Be sure to be less than 3cm from target
	char is_black=0; //No by default

	if (sharpIR<threshold){
		return 1; //Black
	}else{ //Could be white or black but we are not aligned properly

		move_robot(velocitat/2,-velocitat/2,FORWARD,5); //Seguretat
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(velocitat/2,-velocitat/2,FORWARD,5);
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(-velocitat/2,velocitat/2,FORWARD,15);
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(-velocitat/2,velocitat/2,FORWARD,5);
		sharpIR=readADC(4);
		delay_ms(10);
		if (sharpIR<threshold) is_black=1;
		move_robot(velocitat/2,-velocitat/2,FORWARD,10); //Tornem al mig
		return is_black;
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
			_delay_us(100);
			return 0;

	}else{
			//Sensors centrals
			if (proporcional>=-1 && proporcional<=1){
				return 2;
			}else{
				return 1;
			}
	}
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


//M1 right motor speed, M2 left motor speed, Direction FORWARD or BACKWARDS, encoder_steps number of left wheel encoder steps (every 300 steps) one wheel revolution.

/*void print_encoder_counts(){
	if (DEBUG){
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

void load_eeprom_settings(void){
	uint8_t aux=0;
	//Get eeprom velocitat
	aux = eeprom_read_byte((uint8_t *) VELOCITAT_ADDR_EEPROM);
	if (aux>0 && aux <255) velocitat=aux;
	//Get telemetry_enabled
	aux = eeprom_read_byte((uint8_t *) TELEMETRY_ENABLED_ADDR_EEPROM);
	if (aux==0 || aux==1) telemetry_enabled=aux;
	//GET eeprom Kp
	aux = eeprom_read_byte((uint8_t *) KP_ADDR_EEPROM);
	if (aux>0 && aux <255) Kp=aux;
	//GET eeprom Kd
	aux = eeprom_read_byte((uint8_t *) KD_ADDR_EEPROM);
	if (aux>=0 && aux <255) Kd=aux;
	//GET eeprom strategy (Rescue or line following)
	aux = eeprom_read_byte((uint8_t *) STRATEGY);
	if (aux>=0 && aux <255) strategy=aux;
	//GET eeprom curve_correction
	aux = eeprom_read_byte((uint8_t *) CURVE_CORRECTION);
	if (aux>=0 && aux <100) curve_correction=aux;

}
//Li passem el port analogic que volem llegir i ens retorna la lectura (0 .. 7)
uint16_t readADC(uint8_t channel) {
  ADMUX = (0xf0 & ADMUX) | channel;
  ADCSRA |= (1 << ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  return (ADC);
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
   DDRB=0x26;     //0010 0110
   PORTB=0x00;
   DDRC=0x00;     //0000 0000
   PORTC=0x00;
}


void inicializar_timer1(void) //Configura el timer y la interrupción.
{
    OCR1A= 0x009C; //009C 0.5 ms,0138 1 ms. 0C35 10ms, 0x0271 2ms.
    TCCR1B |=((1<<WGM12)|(1<<CS11)|(1<<CS10));    //Los bits que no se tocan a 0 por defecto
    TIMSK1 |= (1<<OCIE1A);
    sei();
}


// Interrupcio de temps activada cada x milisegons (depenent de inicialiar_timer1 en current_millis emmagatzemem el num de ms
ISR(TIMER1_COMPA_vect)
{
	PID_line_following(FORWARD);
}
