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
#include "./Timer-Header/timer.h"

void led_init()
{
	DDRB |= (1 << PB0); // setup led (output)
	PORTB &= ~(1 << PB0); //initially off 
}
void led_toggle()
{
	PORTB ^= (1 << PB0);
}

int main(void)
{	
	led_init();
	timer_init();
    while (1) 
    {
		if((TIFR & (1 << OCF1A)) != 0) // compare match on channel 1 has occurred
		{
			led_toggle();
			TIFR |= (1 << OCF1A);
		}
    }
}

