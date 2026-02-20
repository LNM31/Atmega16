
#define F_CPU 14745600UL
#define FOSC 14745600
#define BAUD 115200
#define MYUBRR FOSC/ 16 / BAUD -1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

char text[32];
uint16_t result;

void TXRX_init()
{
	DDRD |= (1 << PD1); // output TX
	DDRD &= ~(1 << PD0); // input RX
}
void ADC_init()
{
	DDRA &= ~(1 << PA0); // set as input
	
	ADCSRA |= (1 << ADEN); // enabling the ADC module
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // forming the clock division factor
	
	ADMUX |= (1 << REFS0); // setting the voltage reference(AVCC in our case)
	// ADC0 channel is set by default from MUX[3:0] 
}

void ADC_conversion(uint16_t avcc_ref)
{
	ADCSRA |= (1 << ADSC); // setting manually the conversion
	
	while((ADCSRA & (1 << ADSC)) != 0); // waiting for the conversion to finish (ADSC = 0)
	result = (uint16_t)ADCL; // formatting the 10 bits of ADC into result (2 bits of ADCH and 8 bits of ADCL)
	result |= (uint16_t)(ADCH << 8); // formatting the 10 bits of ADC into result (2 bits of ADCH and 8 bits of ADCL)
	
	uint16_t value = result * (avcc_ref / 1023); // converting the binary value into decimal
	sprintf(text, "%d mV\n", value); // saving the value into string "text"
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
	ADC_init();
	
    while (1) 
    {
		ADC_conversion(5000);
		UART_send_string(text);
		_delay_ms(1000);
    }
}



