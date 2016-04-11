#ifndef USART_and_telemetry_H
#define USART_and_telemetry_H

/*************************************************/
/*** Buffer used for UART communications       ***/
#define BUF_SIZE 45
//type definition of buffer structure
typedef struct{
		//Array of chars
		//uint8_t buffer[BUF_SIZE];
		char buffer[BUF_SIZE];
		//array element index
		uint8_t index;
}u8buf;

/*** END buffer structure ***/
/** USART Public functions definition **/
void USART_init(void);
void USART_disable_interrupts(void); //Disable reception interrupt now we can
void USART_transmitByte(uint8_t data);//Used to send a byte via USART (usually BT)


#endif
