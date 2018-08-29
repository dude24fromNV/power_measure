#define F_CPU 16000000UL
/* bytes in read and write buffers */
#define BUFFER_LEN (64)

#include "init.h"
#include "segm.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/eeprom.h> 

/* Value to be placed in UBRR register. See datasheet for more */
static const uint16_t ubrr_val = 51;// (F_CPU / (16 * 19200)) - 1;
/* Read and write buffers */
static uint8_t	rdbuff[BUFFER_LEN] = {'\0'},
		wrbuff[BUFFER_LEN] = {'\0'};
static uint8_t rdind = 0, wrind = 0;	/* Indices */
/* Indicates transfer & reception completion */
volatile bool txcflag = true;
volatile bool rxcflag = false;

void sleep_ms(uint16_t );
static void uart_put(char *);
static bool atomic_str_eq(char *, char *, uint8_t );

static struct segm_Port PB = {
	.DDR = &DDRB,
	.PIN = &PINB,
	.PORT = &PORTB,
};

static struct segm_Display display = {
	.SHCP = {.port = &PB, .pin = 0},
	.STCP = {.port = &PB, .pin = 1},
	.DS   = {.port = &PB, .pin = 2},
	.delay_func = &_delay_loop_1,	
	.sleep_ms_func = &sleep_ms,	
	.is_comm_anode = false		
};

uint8_t d_sec = 0; //fraction of a second
uint8_t channel= 0x00;    
uint16_t adc_volt[5];	
uint16_t adc_amper[5];
uint16_t EEMEM min_W[60];
uint16_t EEMEM day_hours_W[15][24];    
int main()
{	
        uint8_t sec = 0; 
        uint8_t min = 0; 
        uint8_t hours = 0;
        uint8_t day = 0;
        uint32_t volts = 0;
        uint32_t ampers = 0;
        uint32_t power  = 0;
        uint16_t W = 0;
        uint16_t average_adc_volt = 0;
        uint16_t average_adc_amper = 0;
        uint32_t temp = 0;
        uint8_t buffer[30];
        uint16_t zero_pos_acs712 = 511;
        uint8_t wat[]={0, 0, 0, 0};

        init_interapt();
	uart_init();
	init_timer1();
        adc_init(channel);
        segm_init(&display);  
              
	sei();

	uart_put("\n\nHello\n\n");
	
	while(1) {
		  
                for(uint8_t i =0; i<4; i++){
                        average_adc_volt += adc_volt[i];
                        average_adc_amper +=  adc_amper[i];
                };
                average_adc_volt /=5;  
                average_adc_amper /=5;

                /*conversion of current, voltage and power finding*/
                volts = average_adc_volt;
                volts *=488;
                ampers = (abs(average_adc_amper- zero_pos_acs712)*488)/66;
                power = volts * ampers;
                W = power /1000000;

                /*zeroing of the array to avoid overlapping values on the display*/
                wat[0]=0;      
                wat[1]=0;
                wat[2]=0;
                wat[3]=0;
                segm_bcd(W, wat);
                uint8_t symbols[] = {segm_sym_table[wat[3]],segm_sym_table[wat[2]], segm_sym_table[wat[1]], segm_sym_table[wat[0]]};
		segm_indicate4(&display, symbols);

                if(d_sec >4) {
                        sec++;
                        d_sec =0;        
                }         
                if(sec>59){ 
                        eeprom_write_word(&min_W[min], W); 
                        min++;
                        sec=0;
                }
                if(min>59){
                        /*average per hour and save*/
                        for (uint8_t i =0; i<60; i++)
                                temp += eeprom_read_word(&min_W[i]);
                        temp /=60;
                        eeprom_write_word(&day_hours_W[day][hours], temp); 
                        hours++;
                        min=0;
                }
                if(hours>23){
                        day++;
                        hours=0;
                }
                if (day == 15) day=0;

                if (rxcflag == true){
                        if (atomic_str_eq(rdbuff, "stat per hour", 0)) {
			        uart_put("statistics for the last hour: \n");
                                for (uint8_t i =0; i<59; i++){
                                        uint16_t str = eeprom_read_word(&min_W[i]);
                                        sprintf(buffer, "%.04hu\n", str);
			                uart_put(buffer);
                                }
                        rxcflag = false;
                        } else if (atomic_str_eq(rdbuff, "all stat", 0)) {
			        uart_put("hourly statistics for 15 days:\n");
                                for (uint8_t i =0; i<15; i++){
                                        for (uint8_t j =0; j<24; j++){
                                                uint16_t str = eeprom_read_word(&day_hours_W[i][j]);
                                                sprintf(buffer, "%.04hu\n", str);
			                        uart_put(buffer);
                                        }
                                }
                        } else {
			        uart_put("Unknown command\n");
			        uart_put(rdbuff);
                                uart_put("\n");
                                uart_put("Enter \"stat per hour\" or \"all stat\" \n");
			}
                        rxcflag = false;       
                }
        }
}

/*interrupts every 1/5 seconds*/
ISR (TIMER1_COMPA_vect)
{
        /* wait convertation and data change for chanel 0*/
        while ((ADCSRA&(1 << ADIF))== 0);
                adc_volt[d_sec] = ADCL;
	        adc_volt[d_sec]|= (ADCH & 0x03) << 8;
                channel = 0x01;
		 ADMUX &=0xF8;
                ADMUX |= channel;
	        ADCSRA |= (1 << ADSC);
         /* wait convertation and data change for chanel 1*/
        while ((ADCSRA&(1 << ADIF))== 0);
                adc_amper[d_sec] = ADCL;
	        adc_amper[d_sec]|= (ADCH & 0x03) << 8;
                channel = 0x00;
                ADMUX &=0xF8;
                ADMUX |= channel;
	        ADCSRA |= (1 << ADSC);
        
        /*increment fraction of a second*/
        d_sec++;
       
}

/*reset button for stat per hour*/
ISR (INT0_vect)
{
        for(uint8_t i=0; i<60; i++){
                eeprom_write_word(&min_W[i], 0); 
        }  
}

/*reset button for all stat*/
ISR (INT1_vect)
{
        for (uint8_t i =0; i<15; i++){
                for (uint8_t j =0; j<24; j++){
                        eeprom_write_word(&day_hours_W[i][j], 0);
                }
        }
} 

/* USART TX Complete interrupt handler */
ISR(USART_TX_vect, ISR_BLOCK)
{
	/* When data register is empty and after the shift register contents */
	/* are already transfered, this interrupt is raised */
	UCSR0B &= ~(1 << TXCIE0);
}


/* USART Data Register Empty interrupt handler */
ISR(USART_UDRE_vect, ISR_BLOCK)
{
	if (('\0' == wrbuff[wrind]) || txcflag) {
		/* If nothing to transmit - disable this interrupt and exit */
		UCSR0B &= ~(1 << UDRIE0);
		txcflag = true;
		return;
	}

	UDR0 = wrbuff[wrind++];	

	/* Really we don't need this as long as every string ends with '\0' */
	if (wrind >= BUFFER_LEN)
		wrind = 0;
}

/* USART RX Complete interrupt handler */
ISR(USART_RX_vect, ISR_BLOCK)
{
	/* Buffer will contain the last N = <buffer_len> chars read */
	rdbuff[rdind] = UDR0;

	if ('\n' == rdbuff[rdind]) {
		rdbuff[rdind] = '\0';
		rxcflag = true;
		rdind = 0;
	} else {
		rxcflag = false;
		rdind++;
	}

	if (rdind >= BUFFER_LEN)
		rdind = 0;
}

     
    

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); /* stop timer */
	/* It's often required to manually reset interrupt flag */
        /* to avoid infinite processing of it.                  */
        /* not on AVRs (unless OCB bit set)                     */
        /* 	TIFR2 &= ~TOV2;                                 */
}


void sleep_ms(uint16_t ms_val)
{
	/* Set Power-Save sleep mode */
	/* https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html */
	set_sleep_mode(SLEEP_MODE_IDLE);
	cli();		/* Disable interrupts -- as memory barrier */
	sleep_enable();	/* Set SE (sleep enable bit) */
	sei();  	/* Enable interrupts. We want to wake up, don't we? */
	TIMSK2 |= (1 << TOIE2); /* Enable Timer2 Overflow interrupt by mask */
	while (ms_val--) {
		/* Count 1 ms from TCNT2 to 0xFF (up direction) */
		TCNT2 = (uint8_t)(0xFF - (F_CPU / 128) / 1000);

		/* Enable Timer2 */
		TCCR2B =  (1 << CS22) | (1 << CS20); /* f = Fclk_io / 128, start timer */

		sleep_cpu();	/* Put MCU to sleep */

		/* This is executed after wakeup */

	}
	sleep_disable();	/* Disable sleeps for safety */		
}

static void uart_put(char *str)
{
	/* If buffer contents have not been transfered yet -- put MCU to sleep */
	while(!txcflag)
		sleep_cpu();

	/* No interrupts can occur while this block is executed */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < BUFFER_LEN; i++) {
			wrbuff[i] = str[i];
			if ('\0' == str[i])
				break;
		}
		wrind = 0;
		txcflag = false; /* programmatic transfer complete flag */
		/* Enable transmitter interrupts */
		UCSR0B |= (1 << TXCIE0) | (1 << UDRIE0);
	}
}

static bool atomic_str_eq(char *str1, char *str2, uint8_t len)
{
	if (!len)
		len = BUFFER_LEN;

	bool res = true;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < len; i++) {
			if (str1[i] != str2[i]) {
				res = false;
				break;
			}
			if ('\0' == str1[i])
				break;
		}
	}
	return res;
}


















