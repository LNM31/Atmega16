/*
 * timer.c
 *
 * Created: 10/08/2025 15:22:31
 *  Author: lican
 */ 
#define F_CPU 14745600UL
#define COMPARE_VALUE 0x7080


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void timer_init()
{
	unsigned int value = COMPARE_VALUE;
	OCR1AH = (value >> 8) & 0xFF; // setting the value of the comparator
	OCR1AL = value & 0xFF;
	
	TCCR1B |= (1 << WGM12) | (1 << CS12); // CTC mode + prescalar 256 (CS12=1 CS11=0 CS10=0)
	TCCR1A &= ~(1 << WGM11) & ~(1 << WGM10); // CTC mode
}