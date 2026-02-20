/*
 * Assignment1.c
 *
 * Created: 10/09/2025 15:24:27
 * Author : lican
 */ 

#define F_CPU 14745600UL
#define FOSC 14745600
#define BAUD 115200
#define MYUBRR FOSC/ 16 / BAUD -1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

volatile uint8_t data;
volatile uint8_t FLAGRX;

void TXRX_init()
{
	DDRD |= (1 << PD1); // output TX
	DDRD &= ~(1 << PD0); // input RX
}

void UART_init(unsigned int ubrr)
{
	/* Set baud rate */
	UCSRC &= ~(1 << URSEL);
	UBRRH = (unsigned char) (ubrr >> 8);
	UBRRL = (unsigned char) ubrr;
	
	/* set UCSRA */
	UCSRA &= ~(1 << U2X) & ~(1 << MPCM);
	
	/* Enable receiver and transmitter */
	UCSRB |= (1 << TXEN) | (1 << RXEN);
	
	/* Enable UCSRC */
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void UART_transmit(unsigned char data)
{
	while(!(UCSRA & (1 << UDRE)));
	UDR = data; 
}

void interrupt_init()
{
	sei(); //enable ATMEGA global interrupts
	UCSRB |= (1 << RXCIE); // enable UART interrupt environment 
}
ISR(USART_RXC_vect)
{
	data = UDR;
	FLAGRX = 1;
}


void UART_send_string(char *string)
{
	for(int i=0; i < strlen(string); i++)
	{
		UART_transmit(string[i]);
	}
}
int main(void)
{
	TXRX_init();
	UART_init(MYUBRR);
	interrupt_init();
	
    while (1) 
    {
		if(FLAGRX == 1)
		{
			UART_transmit(data);
			FLAGRX = 0;
		}
    }
}



