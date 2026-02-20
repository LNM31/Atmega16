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

//int button_on = 0;

void init()
{
	DDRB |= (1 << PB0); //output led
	DDRD &= ~(1 << PD6); //input button
	
	PORTB &= ~(1 << PB0);// initial, led stins
}
void led_on()
{
	PORTB |= (1 << PB0);
}
void led_off()
{
	PORTB &= ~(1 << PB0);
}
void led_toggle()
{
	static int button_on = 0;
	
	if(button_on)
	{
		led_off();
		button_on = 0;
	}
	else
	{
		led_on();
		button_on = 1;
	}
}

int main(void)
{	
	init();
    while (1) 
    {
		if((PIND & (1 << PD6)) == 0) // button pressed
		{
			_delay_ms(1);
			if((PIND & (1 << PD6)) == 0) //button pressed
			{
				while((PIND & (1 << PD6)) == 0); //ends when released
				led_toggle();
			}
			
		}
		
    }
}

