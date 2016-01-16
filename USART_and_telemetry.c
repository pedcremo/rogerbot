#define BAUD 115200 //BT serial speed. We use USART

#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "USART_and_telemetry.h"
#include "Main.h"
#include "PIDfollower.h"

//declare buffer
u8buf buf;

/** Section devoted to USART (serial) communications using BT **/
//initialize buffer
static void BufferInit(u8buf *buf)
{
		//set index to start of buffer
		buf->index=0;
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
static uint8_t BufferRead(u8buf *buf, volatile uint8_t *u8data)
{
		if(buf->index>0){
				/*buf->index--;
				*u8data=buf->buffer[buf->index];*/
				if (buf->buffer[0]=='l'){ //Load Rogerbot settings
					uint8_t old_velocitat=velocitat;
					load_eeprom_settings();

					_delay_ms(2);
					buf->index=20;
					buf->buffer[20]='0'+(velocitat/100);
					buf->buffer[19]='0'+((velocitat/10) % 10);
					buf->buffer[18]='0'+(velocitat % 10);
					buf->buffer[17]=',';

					if (old_velocitat==0) velocitat=0;//If the robot is stopped we mantain the same state

					if (telemetry_enabled)
						buf->buffer[16]='1';
					else
						buf->buffer[16]='0';

					buf->buffer[15]=',';
					buf->buffer[14]='0'+(Kp/100);
					buf->buffer[13]='0'+((Kp/10) % 10);
					buf->buffer[12]='0'+(Kp % 10);

					buf->buffer[11]=',';
					buf->buffer[10]='0'+(Kd/100);
					buf->buffer[9]='0'+((Kd/10) % 10);
					buf->buffer[8]='0'+(Kd % 10);

					buf->buffer[7]=',';
					buf->buffer[6]=strategy;

          buf->buffer[5]=',';
          buf->buffer[4]='0'+(curve_correction/100);
          buf->buffer[3]='0'+((curve_correction/10) % 10);
          buf->buffer[2]='0'+(curve_correction % 10);



					buf->buffer[1]='z';
					buf->buffer[0]='.';
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
					buf->index=7;
					uint16_t sharp=readADC(4); //Read ADC4
					if (sharp < 1000){
						buf->buffer[7]='0';
						buf->buffer[6]='0'+(sharp/100);
						buf->buffer[5]='0'+((sharp/10) % 10);
						buf->buffer[4]='0'+(sharp % 10);
					}/*else{
						buf->buffer[7]='1';buf->buffer[4]='0';
						buf->buffer[3]='0';buf->buffer[2]='0';
					}*/

					/*buf->index=6;
					if (es_negre()==1){
						buf->buffer[6]='B';
						buf->buffer[5]='L';
						buf->buffer[4]='A';
						buf->buffer[3]='C';
						buf->buffer[2]='K';
					}else{
						buf->buffer[6]='W';
						buf->buffer[5]='H';
						buf->buffer[4]='I';
						buf->buffer[3]='T';
						buf->buffer[2]='E';
					}*/
					if (sharp <17){
						buf->buffer[3]='B';
						buf->buffer[2]='L';
					}else{
						buf->buffer[3]='W';
						buf->buffer[2]='H';
					}

					buf->buffer[1]='z';
					buf->buffer[0]='.';
				}else if(buf->buffer[0]=='v'){ //Read sharp infrared sensor */
					buf->index=5;
					uint16_t voltage=readADC(6); //Read ADC6
					if (voltage < 1000){
						buf->buffer[5]='0';
						buf->buffer[4]='0'+(voltage/100);
						buf->buffer[3]='0'+((voltage/10) % 10);
						buf->buffer[2]='0'+(voltage % 10);
					}else{
						buf->buffer[5]='1';buf->buffer[4]='0';
						buf->buffer[3]='0';buf->buffer[2]='0';
					}
					buf->buffer[1]='z';
					buf->buffer[0]='.';
				}else if(buf->buffer[0]=='d'){ //Debugging purposes
					buf->index=5;
					int inc_speed=0;
					uint16_t voltage=readADC(6); //Read ADC6
					if (voltage<770){
						inc_speed = (770-voltage)/5;
						buf->buffer[5]='+';
					}else{
						inc_speed = (voltage-770)/5;
						buf->buffer[5]='-';
					}

					buf->buffer[4]='0'+(inc_speed/100);
					buf->buffer[3]='0'+((inc_speed/10) % 10);
					buf->buffer[2]='0'+(inc_speed % 10);

					buf->buffer[1]='z';
					buf->buffer[0]='.';
				}else if(buf->buffer[0]=='p'){ //Read ping parallax */
					int ping_=ping(); //Read PB2
					buf->index=4;
					buf->buffer[4]='0'+(ping_/100);
					buf->buffer[3]='0'+((ping_/10) % 10);
					buf->buffer[2]='0'+(ping_ % 10);
					buf->buffer[1]='z';
					buf->buffer[0]='.';
				/* }else if(buf->buffer[0]=='e'){ //Read encoder counts
					//int counts=get_encoder_counts(); //Read encoder PD2
					buf->index=4;
					buf->buffer[4]='0'+(encoder_counts/100);
					buf->buffer[3]='0'+((encoder_counts/10) % 10);
					buf->buffer[2]='0'+(encoder_counts % 10);
					buf->buffer[1]='z';
					buf->buffer[0]='.'; */
				}else if(buf->buffer[0]=='a'){ //Start Rogerbot line following
					 buf->index=1;
					 buf->buffer[1]='o';
					 buf->buffer[0]='.';
					 start=1;
					 //Load Rogerbot settings from eeprom (speed, kp, kd ...)
					 load_eeprom_settings();
				}else if(buf->buffer[0]=='z'){ //Stop Rogerbot line following
					buf->index=1;
					buf->buffer[1]='o';
					buf->buffer[0]='.';
					velocitat=0;
				}else if(buf->buffer[0]=='j'){ //Proves
					buf->index=1;
					//M1_forward(velocitat);M2_forward(velocitat);
					move_robot(velocitat,velocitat,FORWARD,450);
					buf->buffer[1]='o';
					buf->buffer[0]='.';


				}else if(buf->buffer[0]=='t'){ //Send Bar test sensor reading
					buf->index=3;
					uint8_t read_sensor=PID_obtenir_errorp();
					/*if (read_sensor<0){
						buf->buffer[3]='-';
						read_sensor=-read_sensor;
					}else{
						buf->buffer[3]='0';
					}*/
					//buf->buffer[3]='0'+((read_sensor/10) % 10);
					buf->buffer[3]='0'+((read_sensor/10) % 10);
					buf->buffer[2]='0'+(read_sensor % 10);

					/*buf->buffer[2]='1';
					if (read_sensor==9) buf->buffer[2]='9';
					if (read_sensor==8) buf->buffer[2]='8';
					if (read_sensor==7) buf->buffer[2]='7';
					if (read_sensor==6) buf->buffer[2]='6';
					if (read_sensor==5) buf->buffer[2]='5';
					if (read_sensor==4) buf->buffer[2]='4';
					if (read_sensor==3) buf->buffer[2]='3';
					if (read_sensor==2) buf->buffer[2]='2';
					if (read_sensor==0) buf->buffer[2]='0';*/

					//buf->buffer[3] = read_sensor & 0xFF; // equivalent to number % 256 LOW Byte
					//buf->buffer[2] = read_sensor >> 8; // equivalent to number / 256 HIGH Byte
					buf->buffer[1]='z';
					buf->buffer[0]='.';
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
		if ((BufferWrite(&buf, u8temp)==1)||(u8temp=='.'))
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
