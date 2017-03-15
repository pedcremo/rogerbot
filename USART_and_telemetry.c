#define BAUD 115200 //BT serial speed. We use USART

#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>


#include <stdio.h>
#include "USART_and_telemetry.h"
#include "Main.h"
#include "PIDfollower.h"
#include "SensorBar.h"
#include "Ping.h"



#include "math.h"
#include <stdlib.h>
//declare buffer
u8buf buf;

char buffAux[BUF_SIZE]="";


/** Section devoted to USART (serial) communications using BlueTooth **/
//initialize buffer
static void BufferInit(u8buf *buf)
{
		//set index to start of buffer
		buf->index=0;
		strcpy(buf->buffer,"");
}

static void clearBuffer(){
	BufferInit(&buf);
	strcpy(buffAux,"");
}

static void addTobuffer(char *t){
	strcat(buffAux,t);  // copies string 't' to 'buffAux'
}


static void reverse(char *string)
{
	int length, c;
	char *begin, *end, temp;

	length = strlen(string);
	begin  = string;
	end    = string;

	for (c = 0; c < length - 1; c++)
		end++;

	for (c = 0; c < length/2; c++)
	{
		temp   = *end;
		*end   = *begin;
		*begin = temp;

		begin++;
		end--;
	}
}

static void flushBuffer(){
		strcpy(buf.buffer,buffAux);
		strcat(buf.buffer,"z.");
		reverse(buf.buffer);
		buf.index=strlen(buf.buffer);
}


//write to buffer routine
static uint8_t BufferWrite(u8buf *buf, uint8_t u8data)
{
		if (buf->index<BUF_SIZE)
		{
				buf->buffer[buf->index] = u8data;
				//increment buffer index
				buf->index++;
				return 0;
		}
				else return 1;
}
static char * convert_uint8_to_char_array(uint8_t numero){
	char * text;
	text = malloc(4);
	text[0]='0'+(numero/100);
	text[1]='0'+((numero/10) % 10);
	text[2]='0'+(numero % 10);
	text[3]='\0';
	return text;
}
static char * convert_uint16_to_char_array(uint16_t numero){
	char * text;
	text = malloc(5);
	text[0]='0';
	text[1]='0'+(numero/100);
	text[2]='0'+((numero/10) % 10);
	text[3]='0'+(numero % 10);
	text[4]='\0';
	return text;
}
static uint8_t BufferRead(u8buf *buf, volatile uint8_t *u8data)
{
		if(buf->index>0){
				//Convert strategy to string. Null terminated
				char * strategy_s;
				strategy_s = malloc(2);
				strategy_s[0] = strategy;
				strategy_s[1] = '\0';

				if (buf->buffer[0]=='l'){ //Load Rogerbot settings
					load_eeprom_settings();

					clearBuffer();
					addTobuffer(convert_uint8_to_char_array(velocitat));

					if (telemetry_enabled)
						addTobuffer(",1,");
					else
						addTobuffer(",0,");

					addTobuffer(convert_uint8_to_char_array(Kp));
					addTobuffer(",");
					addTobuffer(convert_uint8_to_char_array(Kd));
					addTobuffer(",");
					addTobuffer(strategy_s);
					addTobuffer(",");
					addTobuffer(convert_uint8_to_char_array(curve_correction));
					flushBuffer();

				}else if(buf->buffer[0]=='s'){ //Store Rogerbot settings */
					if (buf->buffer[7]=='.'){//Si arriba el caracter final . guardem sinó no ens arrisquem
							velocitat=buf->buffer[1];
							if (velocitat>=0 && velocitat<=255){
								eeprom_write_byte((uint8_t *) VELOCITAT_ADDR_EEPROM,velocitat);
							}
							telemetry_enabled=buf->buffer[2];
							if (telemetry_enabled==0 || telemetry_enabled==1){
								eeprom_write_byte((uint8_t *) TELEMETRY_ENABLED_ADDR_EEPROM,telemetry_enabled);
							}
							Kp=buf->buffer[3];
							if (Kp>=0 && Kp<=255){
								eeprom_write_byte((uint8_t *) KP_ADDR_EEPROM,Kp);
							}
							Kd=buf->buffer[4];
							if (Kd>=0 && Kd<=255){
								eeprom_write_byte((uint8_t *) KD_ADDR_EEPROM,Kd);
							}
							strategy=buf->buffer[5];
							if (strategy>=97 && strategy<=102){
								eeprom_write_byte((uint8_t *) STRATEGY,strategy);
							}
		          				curve_correction=buf->buffer[6];
							if (curve_correction>=0 && curve_correction<=255){
		          					eeprom_write_byte((uint8_t *) CURVE_CORRECTION,curve_correction);
							}
					}
					buf->index=1;
					buf->buffer[1]='o';
					buf->buffer[0]='.';
				}else if(buf->buffer[0]=='i'){ //Read sharp infrared sensor */
					//buf->index=7;
					uint16_t sharp=readADC(4); //Read ADC4
					clearBuffer();
					if (sharp < 1000){
						addTobuffer(convert_uint16_to_char_array(sharp));
					}
					if (sharp <17){
						addTobuffer("BLACK");
					}else{
						addTobuffer("WHITE");
					}

					flushBuffer();

				}else if(buf->buffer[0]=='v'){ //Read voltage from divisor */
					uint16_t voltage=readADC(6); //Read ADC6
					clearBuffer();
					if (voltage < 1000){

						addTobuffer(convert_uint16_to_char_array(voltage));
					}else{
						addTobuffer("1000");
					}
					flushBuffer();

				}else if(buf->buffer[0]=='d'){ //Debugging purposes
					//buf->index=5;
					/*uint16_t inc_speed=0;
					uint16_t voltage=readADC(6); //Read ADC6
					clearBuffer();
					if (voltage<770){
						inc_speed = (770-voltage)/5;
						addTobuffer("+");
					}else{
						inc_speed = (voltage-770)/5;
						addTobuffer("+");
					}
					addTobuffer(convert_uint16_to_char_array(inc_speed));*/
					llegir_barra_sensors();
					uint8_t i=0;
					clearBuffer();
					for (i=0;i<6;i++){
						addTobuffer(convert_uint8_to_char_array(sensors[i]));						
						if (i != 5) addTobuffer(",");
						else addTobuffer("\n");
					}
					flushBuffer();
				}else if(buf->buffer[0]=='p'){ //Read ping parallax */
					uint16_t ping_=ping(); //Read PB2
					clearBuffer();
					addTobuffer(convert_uint16_to_char_array(ping_));
					flushBuffer();
				
				}else if(buf->buffer[0]=='a'){ //Start Rogerbot line following
					 buf->index=1;
					 buf->buffer[1]='o';
					 buf->buffer[0]='.';
					 /*clearBuffer();
					 addTobuffer("o.");
					 flushBuffer();*/
					 start=1;
					 //Load Rogerbot settings from eeprom (speed, kp, kd ...)
					 load_eeprom_settings();
				}else if(buf->buffer[0]=='z'){ //Stop Rogerbot line following
					buf->index=1;
					buf->buffer[1]='o';
					buf->buffer[0]='.';
					/*clearBuffer();
					addTobuffer("o.");
					flushBuffer();*/
					start =0;
					//velocitat=0;
				}else if(buf->buffer[0]=='j'){ //Read min max and calibrated values for every sensor
					uint16_t A0 = read_sensor_bar_calibrated();
					uint8_t i=0;
					clearBuffer();
					if (A0==0)addTobuffer("0");
					else addTobuffer(convert_uint16_to_char_array(A0));
					addTobuffer("|");
					for (i=0;i<6;i++){
						addTobuffer(convert_uint8_to_char_array(sensors[i]));
						addTobuffer("(");
						addTobuffer(convert_uint8_to_char_array(sensors_min_reading[i]));
						addTobuffer(",");
						addTobuffer(convert_uint8_to_char_array(sensors_max_reading[i]));
						addTobuffer(")");
						if (i != 5) addTobuffer(",");
						else addTobuffer("\n");
					}
					flushBuffer();

				}else if(buf->buffer[0]=='t'){ //Send Bar test sensor reading

					int read_sensor=PID_obtenir_errorp();
					clearBuffer();
					if (read_sensor<0){
						addTobuffer("-");
						read_sensor=-read_sensor;
					}else{
						addTobuffer("0");
					}

					if (read_sensor==9) addTobuffer("9");
					if (read_sensor==8) addTobuffer("8");
					if (read_sensor==7) addTobuffer("7");
					if (read_sensor==6) addTobuffer("6");
					if (read_sensor==5) addTobuffer("5");
					if (read_sensor==4) addTobuffer("4");
					if (read_sensor==3) addTobuffer("3");
					if (read_sensor==2) addTobuffer("2");
					if (read_sensor==1) addTobuffer("1");
					if (read_sensor==0) addTobuffer("0");

					flushBuffer();
				}

				*u8data=buf->buffer[buf->index];
				buf->index--;
				return 0;
		}
		else return 1;
}

void USART_transmitByte(uint8_t data){
	loop_until_bit_is_set(UCSR0A,UDRE0);
	UDR0 = data;
}

void USART_init(void)
{
	BufferInit(&buf);

	UBRR0H = UBRRH_VALUE;              /* defined in setbaud.h */
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif

	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable reception and RC complete interrupt
	UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);

}
//We call this function when we want enable telemetry or debug mode because we want to focus
//our attention in information generated internally that we want to transmit
void USART_disable_interrupts(void){
	//disable reception and RX Complete interrupt
	UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
	//disable transmission and UDR0 empty interrupt
	UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
	//enable transmission and reception
	UCSR0B = (1 << TXEN0) | (1 <<RXEN0);
}

//RX Complete interrupt service routine
ISR(USART_RX_vect)
{
		uint8_t u8temp;
		u8temp=UDR0;
		//check if period char or end of buffer
		if ((BufferWrite(&buf, u8temp)==1)||(u8temp=='.') )
		{
				//disable reception and RX Complete interrupt
				UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
				//enable transmission and UDR0 empty interrupt
				UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
		}
}
//UDR0 Empty interrupt service routine
ISR(USART_UDRE_vect)
{
		//if index is not at start of buffer
		if (BufferRead(&buf, &UDR0)==1)
		{
				//start over
				//reset buffer
				BufferInit(&buf);
				//disable transmission and UDR0 empty interrupt
				UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
				//enable reception and RC complete interrupt
				UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
		}
}
