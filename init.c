#include "init.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>

/* Value to be placed in UBRR register. See datasheet for more */
static const uint16_t ubrr_val = 51;// (F_CPU / (16 * 19200)) - 1;

void adc_init(uint8_t channel)
{
	channel &= 0x07;
	uint8_t ref =  0x01;
	ADMUX =  (ref << 6) | channel;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | 0x07;
}

void init_timer1()
{
        OCR1A = 0xc35;
        TCCR1B |= (1 << WGM12);
        TIMSK1 |= (1 << OCIE1A);
        TCCR1B |= (1 << CS12) | (1 << CS10);
}

void init_interapt()
{
        PORTD  &= ~(1<<2) | ~(1<<3);
        DDRD |= (1<<2) | (1<<3);
        EICRA |= (1 << ISC01)|(1 << ISC11)|(1 << ISC00)|(1 << ISC10);    // The rising edge of INT0 generates an interrupt request.
        EIMSK |= (1 << INT0)| (1 << INT1);     // Turns on INT*
}

void uart_init()
{
	/* To use USART module, we need to power it on first */
	power_usart0_enable();

	/* Configure prescaler */
	UBRR0L = ubrr_val & 0xFF; /* Lower byte */
	UBRR0H = (ubrr_val >> 8) & 0xFF;   /* Higher byte */
	/* Or we could use UBRR0 (16-bit defined) instead */

	/* Set operation mode */
	/* Asynchronous mode, even parity, 2 stop bits, 8 data bits */
	UCSR0C = (1 << UPM01) | (1 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);

	/* Continue setup & enable USART in one operation */
	/* RX & TX complete, Data reg empty interrupts enabled */
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}
