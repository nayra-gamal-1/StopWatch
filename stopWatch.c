/*
 * stopWatch.c
 *
 *  Created on: Sep 11, 2023
 *      Author: NayraGamal
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
unsigned char sec1,sec2,min1,min2,h1,h2;
void display()
	{

	// Display the required number on the SIX 7-segment
	PORTA =(PORTA & 0xC0) | (1<<PA5) ;
	PORTC =(PORTC & 0xF0) | (0x0F & sec1) ;
	_delay_ms(2);
	PORTA =(PORTA & 0xC0) | (1<<PA4) ;
	PORTC =(PORTC & 0xF0) | (0x0F & sec2) ;
	_delay_ms(2);
	PORTA =(PORTA & 0xC0) | (1<<PA3) ;
	PORTC =(PORTC & 0xF0) | (0x0F & min1) ;
    _delay_ms(2);
	PORTA =(PORTA & 0xC0) | (1<<PA2) ;
	PORTC =(PORTC & 0xF0) | (0x0F & min2) ;
	_delay_ms(2);
	PORTA =(PORTA & 0xC0) | (1<<PA1) ;
	PORTC =(PORTC & 0xF0) | (0x0F & h1) ;
    _delay_ms(2);
	PORTA =(PORTA & 0xC0) | (1<<PA0) ;
    PORTC =(PORTC & 0xF0) | (0x0F & h2) ;
	_delay_ms(2);


	}

void Timer1_Init(void)
{
	//USING PRESCALER 64

TCNT1=0;  //Initialize timer1 with 0 value

OCR1A=15625;  //Output Compare Register value

TIMSK |= (1<<OCIE1A); //TIMER1 COMPARE A MATCH ENABLE

TCCR1A |= (1<<FOC1A); //NO PWM (BITS ALWAYS value zero)

TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);  //SET CTC MODE

SREG |= (1<<7);  //SET I-BIT

}
ISR(TIMER1_COMPA_vect){
     ++sec1;
	if(sec1 == 10)
	{   sec1=0;
		sec2++;
	}
	if(sec2==6){
		    sec2=0;
		    min1++;
		}
	if(min1 == 10)
	{
	min1=0;
	min2++;
	}
	if(min2==6){
		min2=0;
		h1++;
	}
	if(h1 == 10)
	{
		h1=0;
		h2++;
}

}

void INT0_Init(void){
	DDRD &=~ (1<<PD2);  //INT0/PD2 AS INPUT
	PORTD |=(1<<PD2); //INTERNAL PULL-UP RESISTOR
	MCUCR |= (1<<ISC01); //TRIGGER AS FALLING EDGE INTERRUPT
	MCUCR &=~(1<<ISC00);
	GICR |= (1<<INT0);   //ENABLE EXTERNAL INTERRUPT
	SREG |= (1<<7);  //SET I-BIT
}
ISR(INT0_vect)
{
	//RESET STOP WATCH
	sec1=0;
	sec2=0;
	min1=0;
	min2=0;
	h1=0;
    h2=0;
}
void INT1_Init(void){
	DDRD &=~ (1<<PD3);  //INT1/PD3 AS INPUT
	MCUCR |= (1<<ISC11) | (1<<ISC10); //TRIGGER AS RISING EDGE INTERRUPT
	GICR |= (1<<INT1);   //ENABLE EXTERNAL INTERRUPT
	SREG |= (1<<7);  //SET I-BIT
}
ISR(INT1_vect){
 TCCR1B &=~(1<<CS11) &~(1<<CS10); //STOP WATCH IS PAUSED (STOPPED)
}
void INT2_Init(void){
	DDRB &=~ (1<<PB2);  //INT2/PB2 AS INPUT
	PORTB |=(1<<PB2); //INTERNAL PULL-UP RESISTOR
	MCUCSR &=~ (1<<ISC2); //TRIGGER AS FALLING EDGE INTERRUPT
	GICR |= (1<<INT2);   //ENABLE EXTERNAL INTERRUPT
	SREG |= (1<<7);  //SET I-BIT

}
ISR(INT2_vect){

	TCCR1B |= (1<<CS11) |  (1<<CS10); //STOP WATCH IS RESUMED
}

int main(void)
{
	DDRC |= 0x0F; 			// Configure the first four pins in PORTC as output pins.
	PORTC &= 0xF0; 		    // 7-segment display zero at the beginning.
	DDRA |= 0xC0;           //Configure the 6-pins in PORTA as output pins
	
        Timer1_Init();
	INT0_Init();
	INT1_Init();
	INT2_Init();
	display();
	while(1){
		display();
	}
}
