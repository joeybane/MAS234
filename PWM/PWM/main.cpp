/*
 * PWM.cpp
 *
 * Created: 22.10.2018 08:45:40
 * Author : SangvikH
 */ 

#include <avr/io.h>
#include <hostio.h>
#include <util/delay.h>
#define F_CPU 8000000

void PWM(int dutyCycle)
{
	for (int ii = 0; ii < 100; ii++)
	{
		if (ii < dutyCycle)
		{
			PORTB = 0xFF;
		}
		else
		{
			PORTB = 0x00;
		}
		_delay_us(500);
	}
}

int main(void)
{
	DDRB = 0xFF;
	
	while (1)
	{
		for (int ii = 0; ii < 100; ii++)
		{
			PWM(ii*ii/100);
		}
		for (int ii = 100; ii > 0; ii--)
		{
			PWM(ii*ii/100);
		}
	}
}