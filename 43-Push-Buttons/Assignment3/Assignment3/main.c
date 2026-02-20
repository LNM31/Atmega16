/*
 * Assignment1.c
 *
 * Created: 10/03/2025 15:21:05
 * Author : lican
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void init()
{
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7); //setup output leds
	DDRD &= ~(1 << PD5) & ~(1 << PD6); //setup input buttons
	PORTB = 0x01; // all leds are off except for the first one
}
void led_switch(uint8_t direction)
{
	if(direction == 0 && PORTB == 0x01) //left
	{
		PORTB = (1 << PB7);
	}
	else if(direction == 1 && PORTB == 0x80) //right
	{
		PORTB = (1 << PB0);
	}
	else if(direction == 0)
	{
		PORTB >>= 1;
	}
	else
	{
		PORTB <<= 1;
	}
}



int main(void)
{	
	init();
    while (1) 
    {
		_delay_ms(1);
		if((PIND & (1 << PD5)) == 0)
		{
			while((PIND & (1 << PD5)) == 0);
			led_switch(0);
		}
		else if((PIND & (1 << PD6)) == 0)
		{
			while((PIND & (1 << PD6)) == 0);
			led_switch(1);
		}
    }
}

