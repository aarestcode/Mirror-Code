/*
 * Interfaces.h
 *
 * Created: 6/14/2016 2:51:55 PM
 *  Author: Thibaud
 */ 

/*
This header file defines all the function to communicate with other hardware. 
It contains code to interface with SPI, I2C, UART and the ADC of the MCU. 
These functions are "independent" of the hardware (except for the pinsets and the SPI functions)
*/


#ifndef INTERFACES_H_
#define INTERFACES_H_

#include "Memory.h"

#ifndef OK
#define OK 0
#endif

//--------------------------------------------------
//                      GENERAL
//--------------------------------------------------
#include <avr/io.h> //General I/O
#define __DELAY_BACKWARD_COMPATIBLE__ //To use variables in delay functions
#include <util/delay.h> //Delay functions
#include <util/twi.h> // For I2C interface
#include <stdio.h> // To use printf
#include <stdbool.h>
#include <string.h>
#include <avr/interrupt.h> // Interrupt use to receive data from UART

//--------------------------------------------------
//                       CODE LED
//--------------------------------------------------
#define DDR_LED DDRD
#define PORT_LED PORTD
#define LED PORTD7

void BlinkLED(void){
	DDR_LED |= (1<<LED);
	_delay_ms(500);
	PORT_LED |= (1<<LED);
	_delay_ms(500);
	PORT_LED &= ~(1<<LED);
	
}

//--------------------------------------------------
//                 SERIAL INTERFACE 0
//--------------------------------------------------s
// PINSET
// PORTD0 = RX
// PORTD1 = TX

// PARAMETERS
uint8_t UART0_buffer[256];
uint8_t UART0_buffer_index;

// PROTOTYPES
int UART0_INIT(uint32_t USART_BAUDRATE);
int UART0_WRITE(uint8_t var);
int UART0_READ(uint8_t* var);
void UART0_FLUSH(void);

// ERROR ENUM
enum uart0{
	UART0_INIT_CODE = 21,
	UART0_WRITE_CODE,
	UART0_READ_CODE,
	UART0_FLUSH_CODE,
	
	UART0_INCORRECT_STOP,
	UART0_FRAME_LOST,
	UART0_PARITY_CHECK
	};

// FUNCTIONS
int UART0_INIT(uint32_t USART_BAUDRATE)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART0_INIT_CODE;
	
	REGISTER[memory_UART0_BAUD] = USART_BAUDRATE;
	uint16_t UBRR_VALUE = (((F_CPU / (USART_BAUDRATE * 16UL))) - 1);

	// Set the baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)(UBRR_VALUE);
	
	// Enable receiver, transmitter and RX complete interrupt
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	
	// Set frame format: 8data, 1stop bit, parity mode disabled
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
	
	// Flush the receive buffer
	UART0_FLUSH();
		
	return OK;
}
int UART0_WRITE(uint8_t var)
{	
	cli();
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART0_WRITE_CODE;
	
	// Wait for empty transmit buffer
	while ( !(UCSR0A & (1<<UDRE0)))
	
	// Start transmission
	REGISTER[memory_UART0_TX] = (REGISTER[memory_UART0_TX]<<8) | var;
	UDR0 = var;
	
	sei();
	return OK;
}
int UART0_READ(uint8_t* var)
{
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART0_READ_CODE;
	
	// Wait for incoming data
	while ( !(UCSR0A & (1<<RXC0)) );
	
	// Incorrect stop error
	if(UCSR0A & (1<<FE0)) return UART0_INCORRECT_STOP;
	// Frame lost
	if(UCSR0A & (1<<DOR0)) return UART0_FRAME_LOST;
	// Parity check error
	if(UCSR0A & (1<<UPE0)) return UART0_PARITY_CHECK;
	
	*var = UDR0;
	REGISTER[memory_UART0_RX] = (REGISTER[memory_UART0_RX]<<8) | (*var);
	
	return OK;
}
void UART0_FLUSH(void)
{
	cli();
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART0_FLUSH_CODE;
	
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
	dummy++; // Because I'm tired of the compiler warning me it's not used
	
	// Set buffer
	memset(UART0_buffer, 0, sizeof(UART0_buffer));
	UART0_buffer_index = 0;
	sei();
}
ISR(USART0_RX_vect){ // Y.W: UART receive interrupt routine.
	UART0_READ(&UART0_buffer[UART0_buffer_index]); // Y.W: put read message into buffer, then buffer is used by IsCommandWaiting. Buffer_index is automatically increased if incoming message is more than 1 byte.
	if(++UART0_buffer_index >= 256) UART0_buffer_index = 0;
	REGISTER[memory_UART0_INDEX] = UART0_buffer_index;
}

//--------------------------------------------------
//                 SERIAL INTERFACE 1 
//--------------------------------------------------
// PINSET
// PORTD2 = RX
// PORTD3 = TX

// PARAMETERS
uint8_t UART1_buffer[256];
uint8_t UART1_buffer_index;

// PROTOTYPES
int UART1_INIT(uint32_t USART_BAUDRATE);
int UART1_WRITE(uint8_t var);
int UART1_READ(uint8_t* var);
void UART1_FLUSH(void);

// ERROR ENUM
enum uart1{
	UART1_INIT_CODE = 31,
	UART1_WRITE_CODE,
	UART1_READ_CODE,
	UART1_FLUSH_CODE,
	
	UART1_INCORRECT_STOP,
	UART1_FRAME_LOST,
	UART1_PARITY_CHECK
	};

// FUNCTIONS
int UART1_INIT(uint32_t USART_BAUDRATE)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART1_INIT_CODE;
	
	REGISTER[memory_UART1_BAUD] = USART_BAUDRATE;
	uint16_t UBRR_VALUE = (((F_CPU / (USART_BAUDRATE * 16UL))) - 1);

	// Set the baud rate
	UBRR1H = (uint8_t)(UBRR_VALUE>>8);
	UBRR1L = (uint8_t)(UBRR_VALUE);
	
	// Enable receiver, transmitter and RX complete interrupt
	UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1);
	
	// Set frame format: 8data, 1stop bit, parity mode disabled
	UCSR1C = (1<<USBS1) | (3<<UCSZ10);
	
	// Flush the receive buffer
	UART1_FLUSH();
	
	return OK;
}
int UART1_WRITE(uint8_t var)
{
	cli();
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART1_WRITE_CODE;
	
	// Wait for empty transmit buffer
	while ( !(UCSR1A & (1<<UDRE1)));
	
	// Start transmission
	REGISTER[memory_UART1_TX] = (REGISTER[memory_UART1_TX]<<8) | var;
	UDR1 = var;
	
	sei();
	return OK;
}
int UART1_READ(uint8_t* var)
{
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART1_READ_CODE;
	
	// Wait for incoming data
	while ( !(UCSR1A & (1<<RXC1)));
	
	// Incorrect stop error
	if(UCSR1A & (1<<FE1)) return UART1_INCORRECT_STOP;
	// Frame lost
	if(UCSR1A & (1<<DOR1)) return UART1_FRAME_LOST;
	// Parity check error
	if(UCSR1A & (1<<UPE1)) return UART1_PARITY_CHECK;
	
	*var = UDR1;
	REGISTER[memory_UART1_RX] = (REGISTER[memory_UART1_RX]<<8) | (*var);
	
	return OK;
}
void UART1_FLUSH(void)
{
	cli();
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | UART1_FLUSH_CODE;
	
	unsigned char dummy;
	while ( UCSR1A & (1<<RXC1) ) dummy = UDR1;
	dummy++; // Because I'm tired of the compiler warning me it's not used
	
	// Set buffer
	memset(UART1_buffer, 0, sizeof(UART1_buffer));
	UART1_buffer_index = 0;
	
	sei();
}
ISR(USART1_RX_vect){
	UART1_READ(&UART1_buffer[UART1_buffer_index]);
	if(++UART1_buffer_index >= 256) UART1_buffer_index = 0;
	REGISTER[memory_UART1_INDEX] = UART1_buffer_index;
}

//--------------------------------------------------
//                   SPI INTERFACE
//--------------------------------------------------
// PINSET
#define DDR_SPI DDRB
#define MOSI_SPI PORTB5
#define SCK_SPI PORTB7

#define DDR_SS_PICO DDRB
#define PORT_SS_PICO PORTB
#define SS_PICO PORTB4 //PICO I/O EXPANDERS
#define SELECT_PICO 0

#define DDR_SS_HV DDRD
#define PORT_SS_HV PORTD
#define SS_HV PORTD5 //HV
#define SELECT_HV 1

#define DDR_SS_BIAS DDRD
#define PORT_SS_BIAS PORTD
#define SS_BIAS PORTD4 //HV BIAS
#define SELECT_BIAS 2

// ERROR ENUM
enum spi{
	SPI_INIT_CODE = 41,
	SPI_WRITE_CODE,
	
	SPI_CLOCK_OOB,
	SPI_TIMEOUT
	};

// FUNCTIONS
int SPI_INIT(uint32_t F_SPI)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SPI_INIT_CODE;
	
	/* Set MOSI, SCK and SS output, all others input (MISO) */
	DDR_SPI |= (1<<MOSI_SPI)|(1<<SCK_SPI);
	DDR_SS_PICO |= (1<<SS_PICO);
	DDR_SS_HV |= (1<<SS_HV);
	DDR_SS_BIAS |= (1<<SS_BIAS);
	
	REGISTER[memory_SPI_FREQ] = F_SPI;
	
	/* Set clock */
	uint16_t prescaler = ceil(log(F_CPU/F_SPI)/log(2)); //ceil to unsure frequency less then F_SPI
	if(prescaler==1) {SPSR |= (1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==2) {SPSR &= ~(1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==3) {SPSR |= (1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR |= (1<<SPR0);}
	else if(prescaler==4) {SPSR &= ~(1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR |= (1<<SPR0);}
	else if(prescaler==5) {SPSR |= (1<<SPI2X); SPCR |= (1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==6) {SPSR &= ~(1<<SPI2X); SPCR |= (1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==7) {SPSR &= ~(1<<SPI2X); SPCR |= (1<<SPR1); SPCR |= (1<<SPR0);}
	else return SPI_CLOCK_OOB;
	
	/* Enable SPI, Master */ // For PICO: CPOL = 0, CPHA = 0 // For HV: CPOL=0, CPHA = 1; 
	SPCR = (1<<SPE)|(1<<MSTR);;
	
	/*Put SS line high */
	PORT_SS_PICO |= (1<<SS_PICO);
	PORT_SS_HV |= (1<<SS_HV);
	PORT_SS_BIAS |= (1<<SS_BIAS);
	
	return OK;
}
int SPI_WRITE(int select, uint8_t * data, uint16_t nbytes)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SPI_WRITE_CODE;
	
	// Disable interrupts
	cli();
	
	// Begin the transmission. Adjust phase (CPHA=0 for PICO, CPHA=1 for HV). Put SS line low
	if(select==SELECT_PICO) {SPCR &= ~(1<<CPHA); PORT_SS_PICO &= ~(1<<SS_PICO);} //PICO
	if(select==SELECT_HV) {SPCR |= (1<<CPHA); PORT_SS_HV &= ~(1<<SS_HV);} //HV
	if(select==SELECT_BIAS) {SPCR |= (1<<CPHA); PORT_SS_BIAS &= ~(1<<SS_BIAS);} //BIAS
			
	for(int II =0; II < nbytes; II++)
	{	
		// Save
		REGISTER[memory_SPI_TX] = (REGISTER[memory_SPI_TX] << 8) | data[II];
		// Transfer byte
		/* Start transmission */
		SPDR = data[II];
		/* Wait for transmission complete */
		uint32_t counter = 0;
		while((!(SPSR & (1<<SPIF)))  && (counter < 1000000)) {counter ++; _delay_us(1);}
		if (counter == 1000000){
			// Enable interrupts
			sei();
			
			return SPI_TIMEOUT;
		}
	}

	// End the transmission. Put SS line high
	PORT_SS_PICO |= (1<<SS_PICO);
	PORT_SS_HV |= (1<<SS_HV);
	PORT_SS_BIAS |= (1<<SS_BIAS);

	// Enable interrupts
	sei();

	return OK;
}

//--------------------------------------------------
//                   I2C INTERFACE
//--------------------------------------------------
// PINSET
// PORTC0 = SCL
// PORTC1 = SDA

// ERROR ENUM
enum i2c{
	I2C_INIT_CODE = 51,
	I2C_WRITE_CODE,
	I2C_READ_CODE,
	
	I2C_CLOCK_OOB,
	I2C_TIMEOUT,
	I2C_START_ARB_LOST,
	I2C_START_CRITICAL,
	I2C_RESTART_ARB_LOST,
	I2C_RESTART_CRITICAL,
	I2C_ADDR_NACK,
	I2C_ADDR_ARB_LOST,
	I2C_ADDR_CRITICAL,
	I2C_DATA_NACK,
	I2C_DATA_ARB_LOST,
	I2C_DATA_CRITICAL,
	I2C_READ_CRITICAL
};

// FUNCTIONS
int I2C_INIT(uint32_t F_I2C)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | I2C_INIT_CODE;
	
	REGISTER[memory_I2C_FREQ] = F_I2C;
	
	// Set frequency of I2C
	int16_t prescaler = ((F_CPU/F_I2C)-16)/2;
	if((prescaler < 0) || (prescaler > 255)) return I2C_CLOCK_OOB;
	TWBR = prescaler;
	
	// Set max count of restart
	REGISTER[memory_I2C_MAX_ITER] = 20;
	
	return OK;
}
int I2C_WRITE(uint8_t SLA, uint8_t * data, uint16_t len)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | I2C_WRITE_CODE;
	
	// Disable interrupts
	cli();
	
	//-------------------------------------------------------------------------------
	//                          Initialization
	//-------------------------------------------------------------------------------	
	// Status to be returned
	int status = OK;

	// Number of tries
	REGISTER[memory_I2C_ITER] = 0;

	restart:
	if (++REGISTER[memory_I2C_ITER] > REGISTER[memory_I2C_MAX_ITER]) {goto quit;} //The device does not respond after MAX_ITER tries

	//-------------------------------------------------------------------------------
	//                                   Send START
	//-------------------------------------------------------------------------------
	// Send start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for transmission
	uint32_t counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000) {
		// Enable interrupts
		sei();
			 
		return I2C_TIMEOUT;
	}

	// Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior
		case TW_REP_START:
		case TW_START:
		break;
		
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_START_ARB_LOST;
		goto restart;
		
		// Error. Should never happen. Do not send stop.
		default:
		// Enable interrupts
		sei();
		return I2C_START_CRITICAL;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                Send Device Address
	//-------------------------------------------------------------------------------
	// Save address in Register
	REGISTER[memory_I2C_SLA] = (uint32_t)(REGISTER[memory_I2C_SLA] << 8) | (SLA & 0xFE);
	
	// Load SLA+W into TWDR Register...
	TWDR = SLA & 0xFE;

	//...and send
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;};

	//4. Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior. Address acknowledged
		case TW_MT_SLA_ACK:
		break;
		
		// Not acknowledged. Device busy. Restart.
		case TW_MT_SLA_NACK:
		status = I2C_ADDR_NACK;
		goto restart;
		
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_ADDR_ARB_LOST;
		goto restart;
		
		// Error.
		default:
		status = I2C_ADDR_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                      Send Data
	//-------------------------------------------------------------------------------
	for (int II=0; II<len; II++){
		// Save data in Register
		REGISTER[memory_I2C_TX] = (REGISTER[memory_I2C_TX]<<8) | data[II];
		
		// Load data into TWDR Register... (and increment)
		TWDR = data[II];
		//...and send
		TWCR = (1<<TWINT) | (1<<TWEN);

		// Wait for transmission
		counter = 0;
		while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter++; _delay_us(1);}
		if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;}

		// Check the status of the interface
		switch (TW_STATUS)
		{
			// Normal behavior. Data acknowledged
			case TW_MT_DATA_ACK:
			break;
			
			// Not acknowledged. Device busy. Restart.
			case TW_MT_DATA_NACK:
			status = I2C_DATA_NACK;
			goto restart;
			// Lost arbitration. Should never happen
			case TW_MT_ARB_LOST:
			status = I2C_DATA_ARB_LOST;
			goto restart;
			
			// Error.
			default:
			status = I2C_DATA_CRITICAL;
			goto quit;
		}
		status = 0;
	}

	//-------------------------------------------------------------------------------
	//                                       Quit
	//-------------------------------------------------------------------------------
	quit:
	//7. Transmit STOP condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	// Enable interrupts
	sei();

	return status;
}
int I2C_READ(uint8_t SLA, uint8_t * data_write, uint16_t write_len, uint8_t * data_read, uint16_t read_len)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | I2C_READ_CODE;
	
	// Disable interrupts
	cli();
	
	//-------------------------------------------------------------------------------
	//                                0. Initialization
	//-------------------------------------------------------------------------------
	// Status to be returned
	int status = OK;
	
	// Number of tries
	REGISTER[memory_I2C_ITER] = 0;

	restart:
	if (++REGISTER[memory_I2C_ITER] > REGISTER[memory_I2C_MAX_ITER]) {goto quit;} //The device does not respond after MAX_ITER tries
		
	//-------------------------------------------------------------------------------
	//                                  1. Send START
	//-------------------------------------------------------------------------------
	// Send start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for transmission
	uint32_t counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000) {
		// Enable interrupts
		sei();
		
		return I2C_TIMEOUT;
	}

	// Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior
		case TW_REP_START:
		case TW_START:
		break;
		
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_START_ARB_LOST;
		goto restart;
		
		// Error. Should never happen. Do not send stop.
		default:
		// Enable interrupts
		sei();
		return I2C_START_CRITICAL;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                         2. Send Device Address + Write (0)
	//-------------------------------------------------------------------------------
	// Save address in Register
	REGISTER[memory_I2C_SLA] = (uint32_t)(REGISTER[memory_I2C_SLA] << 8) | (SLA & 0xFE);
	
	// Load SLA+W into TWDR Register...
	TWDR = SLA & 0xFE;

	//...and send
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;};

	//4. Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior. Address acknowledged
		case TW_MT_SLA_ACK:
		break;
		
		// Not acknowledged. Device busy. Restart.
		case TW_MT_SLA_NACK:
		status = I2C_ADDR_NACK;
		goto restart;
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_ADDR_ARB_LOST;
		goto restart;
		
		// Error.
		default:
		status = I2C_ADDR_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                3. Send Write Data
	//-------------------------------------------------------------------------------
	for (int II=0; II<write_len; II++){
		// Save data in Register
		REGISTER[memory_I2C_TX] = (REGISTER[memory_I2C_TX]<<8) | data_write[II];
		
		// Load data into TWDR Register... (and increment)
		TWDR = data_write[II];
		//...and send
		TWCR = (1<<TWINT) | (1<<TWEN);

		// Wait for transmission
		counter = 0;
		while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter++; _delay_us(1);}
		if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;}

		// Check the status of the interface
		switch (TW_STATUS)
		{
			// Normal behavior. Data acknowledged
			case TW_MT_DATA_ACK:
			break;
			
			// Not acknowledged. Device busy. Restart.
			case TW_MT_DATA_NACK:
			status = I2C_DATA_NACK;
			goto restart;
			// Lost arbitration. Should never happen
			case TW_MT_ARB_LOST:
			status = I2C_DATA_ARB_LOST;
			goto restart;
			
			// Error.
			default:
			status = I2C_DATA_CRITICAL;
			goto quit;
		}
		status = 0;
	}
	

	//-------------------------------------------------------------------------------
	//                                 4. Send RESTART
	//-------------------------------------------------------------------------------
	// Send start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;}

	// Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior
		case TW_REP_START:
		case TW_START:
		break;
		
		// Lost arbitration. Should never happen
		case TW_MR_ARB_LOST:
		status = I2C_RESTART_ARB_LOST;
		goto restart;
		
		// Error. Should never happen. Do not send stop.
		default:
		status = I2C_RESTART_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                       5. Send Device Address + Read (1)
	//-------------------------------------------------------------------------------
	// Save address in Register
	REGISTER[memory_I2C_SLA] = (uint32_t)(REGISTER[memory_I2C_SLA] << 8) | (SLA | 0x01);
	
	// Load SLA+W into TWDR Register...
	TWDR = SLA | 0x01;
	
	//...and send
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;}

	//4. Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior. Address acknowledged
		case TW_MR_SLA_ACK:
		//n_ack++;
		break;
		
		// Not acknowledged. Device busy. Restart.
		case TW_MR_SLA_NACK:
		status = I2C_ADDR_NACK;
		goto restart;
		// Lost arbitration. Should never happen
		case TW_MR_ARB_LOST:
		status = I2C_ADDR_ARB_LOST;
		goto quit;
		
		// Error.
		default:
		status = I2C_ADDR_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                   6. Read
	//-------------------------------------------------------------------------------
	for (int II=0; II < read_len; II++)
	{
		if(II == read_len - 1) TWCR = (1<<TWINT) | (1<<TWEN); //Send NACK this time
		else TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		
		// Wait for transmission
		long counter = 0;
		while (!(TWCR & (1<<TWINT)) && (counter < 1000000)) {counter ++; _delay_us(1);}
		if (counter == 1000000) {status = I2C_TIMEOUT; goto quit;}
		
		switch (TW_STATUS)
		{
			// Normal behavior. Data acknowledged
			case TW_MR_DATA_ACK:
			*data_read = TWDR; //Save data and then increment
			// Save data in Register
			REGISTER[memory_I2C_RX] = (REGISTER[memory_I2C_RX]<<8) | (*data_read++);
			break;
			
			case TW_MR_DATA_NACK:
			II = read_len; // Force end of loop
			*data_read = TWDR; //Save data and then increment
			// Save data in Register
			REGISTER[memory_I2C_RX] = (REGISTER[memory_I2C_RX]<<8) | (*data_read++);
			goto quit;
						
			// Error.
			default:
			status = I2C_READ_CRITICAL;
			goto quit;
		}
		status = 0;
	}
	//-------------------------------------------------------------------------------
	//                                   Q. Quit
	//-------------------------------------------------------------------------------
	quit:

	//7. Transmit STOP condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

	// Enable interrupts
	sei();

	return status;
}

//--------------------------------------------------
//                       ADC
//--------------------------------------------------
// PINSET
#define DDR_SENSORS DDRA
#define PICOMOTOR_VOLTAGE PORTA0
#define HV_GROUND PORTA1
#define HV_VOLTAGE PORTA2
#define BUS_CURR PORTA3
#define BUS_VOLT PORTA4
#define SEP_VOLT PORTA5

// ERROR ENUM
enum adc{
	ADC_INIT_CODE = 71,
	ADC_READ_CODE,
	
	ADC_CLOCK_LOW,
	ADC_CLOCK_HIGH,
	ADC_CLOCK_OOB,
	ADC_TIMEOUT
	};

// FUNCTIONS
int ADC_INIT(uint32_t F_ADC)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | ADC_INIT_CODE;
	
	// CLOCK RATE (between 5kHz and 200kHz for good behavior)
	REGISTER[memory_ADC_FREQ] = F_ADC;
	if (F_ADC < 5000) return ADC_CLOCK_LOW;
	if (F_ADC > 200000) return ADC_CLOCK_HIGH;
	
	// Selection Register (AREF as voltage)
	ADMUX = 0;
	
	// Clock prescaler
	int prescaler = ceil(log(F_CPU/F_ADC)/log(2)); //ceil to unsure frequency less then F_ADC
	if(prescaler>7 || prescaler<0) return ADC_CLOCK_OOB;
	
	// Control and status register A - Enable ADC, No trigger, No Interrupt + set clock
	ADCSRA = (1<<ADEN) | prescaler;
	
	// Disable digital input (Reduce power consumption)
	DIDR0 = (1<<PICOMOTOR_VOLTAGE) | (1<<HV_GROUND) | (1<<HV_VOLTAGE) | (1<<BUS_CURR) | (1<<BUS_VOLT) | (1<<SEP_VOLT);
	
	return OK;
}
int ADC_READ(uint8_t line, uint16_t* data)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | ADC_READ_CODE;
	
	// Disable interrupts
	cli();
	
	// Input channel selection
	ADMUX |= line;
	
	// Start conversion
	ADCSRA |= (1<<ADSC);
	
	// Wait for conversion to be done
	uint32_t counter = 0;
	while((ADCSRA & (1<<ADIF)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000){
		// Enable interrupts
		sei();
		
		return ADC_TIMEOUT;
	}
	
	// Extract data
	*data = (uint16_t)ADCL;
	*data |= (ADCH<<8);	
	
	REGISTER[memory_ADC_RX] = (REGISTER[memory_ADC_RX] << 16) | (*data);
	
	// Reset channel selection
	ADMUX &= ~(line);
	
	// Enable interrupts
	sei();
	
	return OK;
}


#endif /* INTERFACES_H_ */