/*
 * helloWorld.cpp
 *
 * Created: 10.10.2018 12:44:44
 * Author : SangvikH
 */ 

#include <avr/io.h>
#include <hostio.h>
#include <util/delay.h>

int main(void)
{
	DDRB = 0xFF;
	
    while (1) 
    {
		PORTB = 0b00000001;
		_delay_ms(500);
		PORTB = 0b00000000;
		_delay_ms(500);
		
    }
}