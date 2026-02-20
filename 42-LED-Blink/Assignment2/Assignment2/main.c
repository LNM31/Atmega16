/*
 * LED-Blinking.c
 *
 * Created: 10/03/2025 13:19:24
 * Author : lican
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void init_led()
{
	DDRB |= (1 << PB0); //setam ca output
	PORTB &= ~(1 << PB0); //initial led stins
}
void led_on() 
{
	PORTB |= (1 << PB0);
}
void led_off()
{
	PORTB &= ~(1 << PB0);
}
void delay()
{
	_delay_ms(1000);
}


int main(void)
{
    init_led();
    while (1) 
    {
		led_on();
		delay();
		led_off();
		delay();
    }
}

