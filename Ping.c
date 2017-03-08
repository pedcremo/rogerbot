#include <avr/io.h>
#include <util/delay.h>
#include "Main.h"


//Local macros
#define BV(bit)			(1 << bit)
#define clearBit(byte,bit) 	(byte &= ~BV(bit))
#define toggleBit(byte,bit) 	(byte ^= BV(bit))
#define setBit(byte,bit) 	(byte |= BV(bit))

#define PING_PORT 5

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
  	// agafem sols la meitat del recorregut
  	return (pulseStart*3.2)/2/2.9;

}