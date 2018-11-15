/*
 * Interrupt.cpp
 *
 * Created: 22.10.2018 10:49:26
 * Author : SangvikH
 */ 

#include <avr/io.h>
#include <hostio.h>

int main(void)
{
	//Set input/output registers
	DDRB = 0xFF;
	DDRD = 0x00;
	
	//While loop
    while (1) 
    {
		bool buttonValue = PORTD & (1 << 0);
		
		if (buttonValue == true)
		{
			PORTB |= (1 << 0);
		}
		else
		{
			PORTB &= ~(1 << 0);
		}
    }
}

