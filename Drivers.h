/*
 * Drivers.h
 *
 * Created: 6/14/2016 3:50:18 PM
 *  Author: Thibaud
 */ 


#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "Interfaces.h"
#include "Memory.h"
#include <stdbool.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#ifndef OK
#define OK 0
#endif

/*--------------------------------------------------
              COMMUNICATION (XBEE & USB)
--------------------------------------------------*/
// PARAMETERS
#define MessageCommandN 1 //Length of command in byte
#define MessageDataN 4 // Length of data
#define MessageChecksumN 1 // length of checksum
#define MessageN MessageCommandN+MessageDataN+MessageChecksumN // Command(1) + Data(4) + Checksum(1)
unsigned char Message[MessageN]; //Vector of received bytes
unsigned char Feedback[MessageN]; //Vector of transmitted bytes

// ERROR ENUM
enum communication{
	COMMUNICATION_READ_PORT = 101,
	COMMUNICATION_WRITE_PORT
};

// FUNCTIONS
int COMMUNICATION_INIT(long timeout_ms)
{
	//TODO XBEE_INIT
	
	// Initialize register
	REGISTER[memory_MESSAGE_COUNT0] = 0;
	REGISTER[memory_FEEDBACK_COUNT0] = 0;
	REGISTER[memory_MESSAGE_COUNT1] = 0;
	REGISTER[memory_FEEDBACK_COUNT1] = 0;
	
	REGISTER[memory_COMMUNICATION_TIMEOUT] = timeout_ms;
	
	return OK;
}
int IsCommandWaiting(void)
{
	if(USART0_FLAG()) return 1; // USB (priority)
	if(USART1_FLAG()) return 2; // XBee
	return 0;
}
int LoadMessage(int port, uint8_t * buffer, int len, long timeout_ms)
{
	int error;
	
	//The message byte
	char temp;
	
	if(port==1){			// USB
		//Loop over the number of bytes to be received
		for (int II = 0; II < len; II++)
		{
			error = USART0_READ(&temp, timeout_ms);
			if(error) {USART0_FLUSH(); return error;}

			buffer[II] = temp;
		}
		
		// Flush the rest to clear the buffers
		USART0_FLUSH();
	}
	else if (port==2){		// XBee
		//Loop over the number of bytes to be received
		for (int II = 0; II < len; II++)
		{
			error = USART1_READ(&temp, timeout_ms);
			if(error) {USART1_FLUSH(); return error;}

			buffer[II] = temp;
		}
		
		// Flush the rest to clear the buffers
		USART1_FLUSH();
	}
	else return COMMUNICATION_READ_PORT;
	REGISTER[memory_MESSAGE_COUNT0 + port -1] ++;
	
	return OK;
}
int SaveCommand(int port)
{
	return LoadMessage(port, Message, MessageN, REGISTER[memory_COMMUNICATION_TIMEOUT]);
}
int SendFeedback(int port, int address, long data)
{
	int error;
	int II;
	unsigned int sum = 0;

	// Address
	for (II = 0; II < MessageCommandN; II++)
	{
		sum += (address >> 8*(MessageCommandN-II-1)) & 0xff;
		Feedback[II] = (unsigned char)(address >> 8*(MessageCommandN-II-1));
	}
	
	// Data
	for (II = 0; II < MessageDataN; II++)
	{
		sum += (data >> 8*(MessageDataN-II-1)) & 0xff;
		Feedback[MessageCommandN + II] = (unsigned char)(data >> 8*(MessageDataN-II-1));
	}

	// Checksum
	if(MessageChecksumN)
	{
		unsigned int mask = (1<<8*MessageChecksumN) - 1;
		unsigned int checksum = mask - (sum & mask);
		
		for (II = 0; II < MessageChecksumN; II++)
		{
			Feedback[MessageCommandN + MessageDataN + II] = (unsigned char)(checksum >> 8*(MessageChecksumN-II-1));
		}
	}

	
	// Send
	if (port==1){
		for(II = 0; II < MessageN; II++)
		{
			error = USART0_WRITE(Feedback[II]);
			if(error) return error;
		}
	}
	else if (port==2){
		for(II = 0; II < MessageN; II++)
		{
			error = USART1_WRITE(Feedback[II]);
			if(error) return error;
		}
	}
	else return COMMUNICATION_WRITE_PORT;
	REGISTER[memory_MESSAGE_COUNT0 + port - 1] ++;

	return OK;
}

/*--------------------------------------------------
                    POWER/HV BOARD
--------------------------------------------------*/
// AD5641
// PINSET
#define DDR_SV DDRB
#define PORT_SV PORTB
#define FIVE_V_E PORTB1
#define TWELVE_V_E PORTB0
#define TWO_EIGHT_V_E PORTB2

#define DDR_CL_E DDRC
#define PORT_CL_E PORTC
#define CL1_E PORTC4
#define CL2_E PORTC5
#define CL3_E PORTC6

#define DDR_CL_F1 DDRA
#define PIN_CL_F1 PINA
#define CL2_F PORTA6
#define CL3_F PORTA7

#define DDR_CL_F2 DDRD
#define PIN_CL_F2 PIND
#define CL1_F PORTD6

#define TWO_FIVE_V 775

//PROTOTYPE
int POWER_INIT(void);
int ActivateHV(void);
int DeactivateHV(void);
int ActivatePICOV(bool withEncoders);
int SetVoltage(uint16_t voltage);
int SetBias(uint16_t voltage);
int EnableSV(int port, bool state);
int EnableCL(int port, bool state);
int MeasureV(int port, int* val);
bool IsCLFault(int port);

// ERROR ENUM
enum power_driver{
	VAR_FEEDBACK_OOB = 121,
	GROUND_FEEDBACK_OOB,
	PICOMOTOR_FEEDBACK_OOB,
	BUSV_OOB,
	BUSI_OOB,
	SEP_VOL_OOB,
	CL1_FAULT,
	CL2_FAULT,
	CL3_FAULT,
	CL_INDEX_OOB
	};

// FUNCTIONS
int POWER_INIT(void){	
	// Set tolerance on ADC feedback
	REGISTER[memory_HV_TOL_V] = 31; // 0.1V tolerance over 3.3V max

	// Set Supply voltage enable as output
	DDR_SV |= ((1<<FIVE_V_E) | (1<<TWELVE_V_E) | (1<<TWO_EIGHT_V_E));
	PORT_SV &= ~((1<<FIVE_V_E) | (1<<TWELVE_V_E) | (1<<TWO_EIGHT_V_E));
	
	// Disable Supply Voltages
	int error;
	error = EnableSV(TWELVE_V_E,false);
	if(error) return error;
	error = EnableSV(FIVE_V_E,false);
	if(error) return error;
	error = EnableSV(TWO_EIGHT_V_E,false);
	if(error) return error;
	
	// Disable Current Limiters
	error = EnableCL(CL1_E,false);
	if(error) return error;
	error = EnableCL(CL2_E,false);
	if(error) return error;
	error = EnableCL(CL3_E,false);
	if(error) return error;
	
	// Set Current Limiters Enable as output
	DDR_CL_E |= ((1<<CL1_E) | (1<<CL2_E) | (1<<CL3_E));
	PORT_CL_E &= ~((1<<CL1_E) | (1<<CL2_E) | (1<<CL3_E));
	
	// Set Current Limiters Fault as inputs
	DDR_CL_F1 &= ~((1<<CL2_F) | (1<<CL3_F));
	DDR_CL_F2 &= ~(1<<CL1_F);
	
	return OK;
}
int ActivateHV(void)
{
	int error;
	
	// 1) Enable 5V
	error = EnableSV(FIVE_V_E,true);
	if(error) return error;
	
	// 2) Command variable HV to 0V
	error = SetVoltage(10000);
	if(error) return error;	
	
	// 3) Command ground to 0V
	error = SetBias(0x3fff);
	if(error) return error;
	
	// 4) Enable 12V
	error = EnableSV(TWELVE_V_E,true);
	if(error) return error;
	
	// 5) Enable CL3
	error = EnableCL(CL3_E,true);
	if(error) return error;
	
	// 6) Check HV_VOLTAGE at 2.5V
	/*_delay_ms(1500);
	int val;
	error = MeasureV(PICOMOTOR_VOLTAGE,&val);
	if(error) return error;
	if(abs(val-TWO_FIVE_V) > REGISTER[memory_HV_TOL_V]) {
		// Disable CL3
		error = EnableCL(CL3_E,false);
		if(error) return error;
		
		return VAR_FEEDBACK_OOB;
	}*/
		
	// 7) Check CL3
	/*if(IsCLFault(3)){
		// Disable CL3
		error = EnableCL(CL3_E,false);
		if(error) return error;
		
		return CL3_FAULT;
	}*/
	
	// 8) Enable CL2
	error = EnableCL(CL2_E,true);
	if(error) return error;
	
	// 9) Check HV_GROUND at 2.5V
	/*_delay_ms(1500);
	error = MeasureV(PICOMOTOR_VOLTAGE,&val);
	if(error) return error;
	if(abs(val-TWO_FIVE_V) > REGISTER[memory_HV_TOL_V]) {
		// Disable CL2
		error = EnableCL(CL2_E,false);
		if(error) return error;
		
		return GROUND_FEEDBACK_OOB;
	}*/
	
	// 10) Check CL2 //TODO
	/*if(IsCLFault(2)){
		// Disable CL2
		error = EnableCL(CL2_E,false);
		if(error) return error;
		
		return CL2_FAULT;
	}*/
	
	return OK;
}
int DeactivateHV(void){
	int error;
	
	// 1) Command variable HV to BIAS
	error = SetVoltage(REGISTER[memory_HV_BIAS]);
	if(error) return error;
	_delay_ms(REGISTER[memory_HV_TIMER]);
	
	// 2) Disable 12V
	error = EnableSV(TWELVE_V_E,false);
	if(error) return error;
	
	// 3) Disable CL3
	error = EnableCL(CL3_E,false);
	if(error) return error;
	
	// 4) Disable CL2
	error = EnableCL(CL2_E,false);
	if(error) return error;
	
	// 5) Disable 5V
	error = EnableSV(FIVE_V_E,false);
	if(error) return error;
	
	return OK;
	
}
int ActivatePICOV(bool withEncoders){
	int error;
	
	// 1) Enable 12V
	error = EnableSV(TWELVE_V_E,true);
	if(error) return error;
	
	// 2) Enable CL1
	error = EnableCL(CL1_E,true);
	if(error) return error;
	
	// 3) Check PICOMOTOR_VOLTAGE at 2.5V !!! TAKES ABOUT 1.5 sec TO STABILIZE !!!
	_delay_ms(1500);
	int val;
	error = MeasureV(PICOMOTOR_VOLTAGE,&val);
	if(error) return error;
	if(abs(val-TWO_FIVE_V) > REGISTER[memory_HV_TOL_V]) {
		// Disable CL1
		error = EnableCL(CL1_E,false);
		if(error) return error;
		
		return PICOMOTOR_FEEDBACK_OOB;
	}
		
	// 4) Check CL1
	/*if(IsCLFault(1)){
		// Disable CL1
		error = EnableCL(CL1_E,false);
		if(error) return error;
		
		return CL1_FAULT;
	}*/

	if(withEncoders){
		// 5) Enable 2.8V
		error = EnableSV(TWO_EIGHT_V_E,true);
		if(error) return error;
	}
	
	return OK;
}
int DeactivatePICOV(void){
	int error;
	
	// 1) Disable 12V
	error = EnableSV(TWELVE_V_E,false);
	if(error) return error;
	
	// 2) Disable CL1
	error = EnableCL(CL1_E,false);
	if(error) return error;
	
	// 5) Disable 2.8V
	error = EnableSV(TWO_EIGHT_V_E,false);
	if(error) return error;
	
	return OK;
}
int SetVoltage(uint16_t voltage)
{
	int error = SPI_WRITE(SELECT_HV,(uint8_t [2]){voltage>>8, voltage},2);
	if(error) return error;
	
	REGISTER[memory_HV] = voltage;
	return OK;
}
int SetBias(uint16_t voltage){
	int error = SPI_WRITE(SELECT_BIAS,(uint8_t [2]){voltage>>8, voltage},2);
	if(error) return error;
	
	REGISTER[memory_GND] = voltage;
	return OK;
}
int EnableSV(int port, bool state)
{
	// PORT = FIVE_V_E or TWELVE_V_E or TWO_EIGHT_V_E
	// State = true means enable // State = false means disable
	if(state) PORT_SV |= (1<<port);
	else PORT_SV &= ~(1<<port);
	
	return OK;
}
int EnableCL(int port, bool state)
{
	// PORT = CL1_E or CL2_E or CL3_E
	// State = true means enable // State = false means disable
	if(state) PORT_CL_E &= ~(1<<port);
	else PORT_CL_E |= (1<<port);
	
	return OK;
}
int MeasureV(int port, int* val)
{
	// port = HV_VOLTAGE or HV_GROUND or PICOMOTOR_VOLTAGE or BUS_CURR or BUS_VOLT or SEP_VOLT
	// OUTPUT val = value to be returned
	int error;
	
	error = ADC_READ(port,val);
	if(error) return error;
	
	REGISTER[memory_PICOMOTOR_V_FB + port] = *val;
	return OK; 
}
bool IsCLFault(int CL_index)
{
	// CL_index = 1, 2 or 3
	if (CL_index==1) return (PIN_CL_F2 & (1<<CL1_F));
	if (CL_index==2) return (PIN_CL_F1 & (1<<CL2_F));
	if (CL_index==3) return (PIN_CL_F1 & (1<<CL3_F));
	else return CL_INDEX_OOB;
}

/*--------------------------------------------------
                   SEPARATION DEVICE
--------------------------------------------------*/
// PINSET
#define DDR_SD_TRIG DDRB
#define PORT_SD_TRIG PORTB
#define SEP_DEV_TRIG PORTB3

#define DDR_SD_DET DDRC
#define PIN_SD_DET PINC
#define SEP_DEV_DET PORTC7

// ERROR ENUM
enum seperation_device{
	SEPARATION_DEV_TIMEOUT = 161
};

bool IsMirrorConstrained(void);

// FUNCTIONS
int SEP_DEV_INIT(void)
{
	// Set pin as output
	DDR_SD_TRIG |= (1<<SEP_DEV_TRIG);
	
	// Set pin to 0
	PORT_SD_TRIG &= ~(1<<SEP_DEV_TRIG);
	
	// Set detection as input
	DDR_SD_DET &= ~(1<<SEP_DEV_DET);
	
	return OK;
} 
int ReleaseMirror(long timeout_ms)
{	
	// Set pin to 1
	PORT_SD_TRIG |= (1<<SEP_DEV_TRIG);
	
	// Wait for incoming data
	uint32_t counter = 0;
	while ( IsMirrorConstrained() && (counter < 1000*timeout_ms)){
		_delay_us(1);
		++counter;
	}
	
	// Set pin to 0
	PORT_SD_TRIG &= ~(1<<SEP_DEV_TRIG);
	
	// Return
	if(counter == 1000*timeout_ms) return SEPARATION_DEV_TIMEOUT;
	return OK;
}
bool IsMirrorConstrained(void){
	// TODO
	return 1;// (PIN_SD_DET & (1<<SEP_DEV_DET));
}

/*--------------------------------------------------
                  PICOMOTORS/ENCODERS
--------------------------------------------------*/
// MCP23S17
const int IOEaddr = 0x40; // Address of the IO Expander (LSB = 0 for WRITE operations)
const int IOEport[3] = {0x12,0x13,0x13}; // Address of the port to which each picomotor is connected (pico1, pico2, pico3)
const int IOEpin[12] = {1,0,3,2,1,0,3,2,5,4,7,6}; // Pin of each port to which the switch is connected (pico1_FW_HIGH, pico1_FW_LOW, pico1_BW_HIGH, pico1_BW_LOW, ...)

// ENCODER PINSET
#define DDR_ENCODER0 DDRC
#define PIN_ENCODER0 PINC
#define ENCODER0A PINC2
#define ENCODER0B PINC3

#define DDR_ENCODER1 DDRD
#define PIN_ENCODER1 PIND
#define ENCODER1A PIND6
#define ENCODER1B PIND7

#define DDR_ENCODER2 DDRD
#define PIN_ENCODER2 PIND
#define ENCODER2A PIND4
#define ENCODER2B PIND5

// ENCODER STATES
enum EncoderState {state00,state10,state11,state01}; 

// ERROR ENUM
enum picomotor_driver{
	ENCODER_STATE_CRITICAL = 141
	};

// FUCTIONS
int PICOMOTORS_INIT(void)
{
	int error;
	
	// Set Encoders as inputs
	//DDR_ENCODER0 &= ~((1<<ENCODER0A) | (1<<ENCODER0B));
	//DDR_ENCODER1 &= ~((1<<ENCODER1A) | (1<<ENCODER1B));
	//DDR_ENCODER2 &= ~((1<<ENCODER2A) | (1<<ENCODER2B));
	
	// Set CONFIGURATION bits to 0
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x0A, 0x00},3);
	if (error) return error;
	
	// Set PortA pins as outputs (via IODIRA)
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x00, 0x00},3);
	if (error) return error;
	
	// Set PortB pins as outputs (via IODIRB)
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x01, 0x00},3);
	if (error) return error;
	
	// Deactivate PortA pull-up resistors
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x0C, 0x00},3);
	if (error) return error;
	
	// Deactivate PortB pull-up resistors
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x0D, 0x00},3);
	if (error) return error;
	
	// Set PortA pins to 0
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x12, 0x00},3);
	if (error) return error;

	// Set PortB pins to 0
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x13, 0x00},3);
	if (error) return error;

	return OK;
}
int MovePicomotor(int index, signed int ticks)
{
	// INFO: @4MHz SPI clock, full message takes 37us to be sent
	//ticks < 0 <=> BACKWARD
	//ticks > 0 <=> FORWARD
	int error = 0;
	int II = 0;
	
	if(ticks>0)
	{
		//MOVE FORWARD
		for (II=0; II<ticks; II++){
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index]},3);
			if (error) goto end;
			_delay_us(326); // The delay was shorten by 2 full message durations. The next message is happening 37us earlier than Nicolas' code
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;
			_delay_us(10); // Unchanged. Message delay accounted in the previous pause.
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index+1]},3);
			if (error) goto end;
			_delay_us(63); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end; 
			_delay_us(2463); // Accounted for message delay
		}
		
	}
	else if(ticks<0)
	{
		//BACKWARD
		for (II=0; II>ticks; II--){
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index+2]},3);
			if (error) goto end;
			_delay_us(63); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;
			_delay_us(63); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index+3]},3);
			if (error) goto end;
			_delay_us(363); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;
			_delay_us(2363); // Accounted for message delay
		}
	}
	
	end:
	// Update memory vector
	REGISTER[memory_PICO0_TICKS + index] += II;

	return error;
}
int GetEncoderState(int index, int* state)
{
	// INPUT  index = 0 or 1 or 2 depending on the encoder
	// OUTPUT state = state00 or state10 or state11 or state01 (depending of state of channel A and B resp.)
	
	if(index==0){
		char PIN = PIN_ENCODER0;
		if ((PIN & (1<<ENCODER0A)) && (PIN & (1<<ENCODER0B))) {REGISTER[memory_ENCODER0_STATE] = state11; *state = state11; return OK;}
		if ((PIN & (1<<ENCODER0A)) && !(PIN & (1<<ENCODER0B))) {REGISTER[memory_ENCODER0_STATE] = state10; *state = state10; return OK;}
		if (!(PIN & (1<<ENCODER0A)) && (PIN & (1<<ENCODER0B))) {REGISTER[memory_ENCODER0_STATE] = state01; *state = state01; return OK;}
		if (!(PIN & (1<<ENCODER0A)) && !(PIN & (1<<ENCODER0B))) {REGISTER[memory_ENCODER0_STATE] = state00; *state = state00; return OK;}
	}
	if(index==1){
		char PIN = PIN_ENCODER1;
		if ((PIN & (1<<ENCODER1A)) && (PIN & (1<<ENCODER1B))) {REGISTER[memory_ENCODER1_STATE] = state11; *state = state11; return OK;}
		if ((PIN & (1<<ENCODER1A)) && !(PIN & (1<<ENCODER1B))) {REGISTER[memory_ENCODER1_STATE] = state10; *state = state10; return OK;}
		if (!(PIN & (1<<ENCODER1A)) && (PIN & (1<<ENCODER1B))) {REGISTER[memory_ENCODER1_STATE] = state01; *state = state01; return OK;}
		if (!(PIN & (1<<ENCODER1A)) && !(PIN & (1<<ENCODER1B))) {REGISTER[memory_ENCODER1_STATE] = state00; *state = state00; return OK;}
	}
	if(index==2){
		char PIN = PIN_ENCODER2;
		if ((PIN & (1<<ENCODER2A)) && (PIN & (1<<ENCODER2B))) {REGISTER[memory_ENCODER2_STATE] = state11; *state = state11; return OK;}
		if ((PIN & (1<<ENCODER2A)) && !(PIN & (1<<ENCODER2B))) {REGISTER[memory_ENCODER2_STATE] = state10; *state = state10; return OK;}
		if (!(PIN & (1<<ENCODER2A)) && (PIN & (1<<ENCODER2B))) {REGISTER[memory_ENCODER2_STATE] = state01; *state = state01; return OK;}
		if (!(PIN & (1<<ENCODER2A)) && !(PIN & (1<<ENCODER2B))) {REGISTER[memory_ENCODER2_STATE] = state00; *state = state00; return OK;}
	}
	
	return ENCODER_STATE_CRITICAL;
}

//---------------------------------------------------------------------------------------
//                                       MULTIPLEXER
//---------------------------------------------------------------------------------------
// ADRESSES OF I/O EXPANDERS
const char MPaddr[2] = {0x98,0x84}; // Multiplexer I2C addresses (connected to SCL, SDA, V+). LSB is irrelevant (7-bit address in bit 7 to bit 1)
	
// ADRESSES OF SWITCHES
const bool MPIC[42] =   {   1,   1,   0,   0,   0,   0,   1,   0,   0,   1,   1,   0,   1,   1,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   0,   1,   0,   1,   0,   1,   0,   1,   1}; // I/O expander
const char MPport[42] = {0x3F,0x3E,0x28,0x3E,0x39,0x38,0x34,0x2D,0x2F,0x39,0x3D,0x3C,0x30,0x2D,0x24,0x3A,0x2A,0x29,0x30,0x3D,0x2F,0x26,0x31,0x33,0x38,0x2E,0x2B,0x3A,0x25,0x33,0x3B,0x37,0x2C,0x31,0x3B,0x27,0x2E,0x3F,0x2C,0x32,0x3C,0x32};
	
// FUNCTIONS
int MULTIPLEXER_INIT(int i)
{
	// Integer to return
	int status;

			// Set mode to normal
			status = I2C_WRITE(MPaddr[i],(uint8_t [2]){0x04, 0x01},2); //Send message
			if(status) return status;
			
			// Set global current to 10.5mA for 0x06, 12mA for 0x07
			status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x02, 0x06},2); //Send message
			if(status) return status;
			
			// Set all pins to LED segment driver configuration (LED = switch)
			for (char cmd = 0x09; cmd <= 0x0F; cmd++)
			{
				status = I2C_WRITE(MPaddr[i], (uint8_t [2]){cmd, 0},2); //Send message
				if(status) return status;
			}
			
			// Disable nonexistent ports on smaller multiplexer
			if (i%2==1)
			{
				status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x09, 0x55},2); //Send message
				if(status) return status;
				status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x0A, 0x55},2); //Send message
				if(status) return status;
			}
			
			// Set all output values to 0
			for (char cmd = 0x44; cmd <= 0x5c; cmd += 8)
			{
				status = I2C_WRITE(MPaddr[i], (uint8_t [2]){cmd, 0},2); //Send message
				if(status) return status;
			}
	
	return OK;
}
int ChannelOn(int ch)
{
	int status = I2C_WRITE(MPaddr[MPIC[ch]], (uint8_t [2]){MPport[ch], 1},2); //Send message
	if(!status) REGISTER[memory_MUX_ACTIVE_CH] = ch;
	return status;
}
int ChannelOff(int ch)
{
	int status = I2C_WRITE(MPaddr[MPIC[ch]], (uint8_t [2]){MPport[ch], 0},2); //Send message
	if(!status) REGISTER[memory_MUX_ACTIVE_CH] = -1;
	return status;
}

/*--------------------------------------------------
                   THERMO-SENSORS
--------------------------------------------------*/
// ADDRESSES OF SENSORS
const uint8_t MCP9801addr[1] = {0x9E}; // const uint8_t MCP9801addr[2] = {0x9E, 0x92}; 
const uint8_t PCT2075addr[0] = {}; // const uint8_t PCT2075addr[2] = {0x9E, 0x90}; 
const uint8_t TMP006addr[0] = {}; //const uint8_t TMP006addr[3] = {0x80, 0x82, 0x8A};

	
// PARAMETERS
const float S0[1] = {1.0}; // TODO: Calibrate S0

// ENUM
enum temp_sensors{
	SENSOR_INDEX_OOB = 171,
	};

// FUNCTIONS
int TEMP_SENSORS_INIT(void){
	int status;
	
	// MCP9801 (12-bit precision)
	for(int II=0; II < (sizeof(MCP9801addr)/sizeof(uint8_t)); II++){
		status = I2C_WRITE(MCP9801addr[II], (uint8_t [2]){0x01, 0b01100000}, 2);
		if(status) return status;
	}
	
	// PCT2075
	for(int II=0; II < (sizeof(PCT2075addr)/sizeof(uint8_t)); II++){
		status = I2C_WRITE(PCT2075addr[II], (uint8_t [2]){0x01, 0b00000000}, 2); // Normal reading mode
		if(status) return status;
		status = I2C_WRITE(PCT2075addr[II], (uint8_t [2]){0x04, 0b00000001}, 2); // Period to measure temperature = 100 ms
		if(status) return status;
	}
	
	// TMP006
	for(int II=0; II < (sizeof(TMP006addr)/sizeof(uint8_t)); II++){
		status = I2C_WRITE(TMP006addr[II], (uint8_t [3]){0x02, 0x74, 0}, 3);
		if(status) return status;
	}
	
	
	return OK;
}
int GetTemperatureMCP9801(int sensor_index, int16_t * temperature_128){	
	
	// Check index
	if(sensor_index >= (sizeof(MCP9801addr)/sizeof(uint8_t))) return SENSOR_INDEX_OOB;
	
	uint8_t read_data[2];
	
	int status = I2C_READ(MCP9801addr[sensor_index], (uint8_t [1]){0}, 1, read_data, 2);
	if(status) return status;
	
	uint16_t sign = (read_data[0] & 0x80) && (0x80); // 0 = +, 1 == -
	uint16_t temp_128_abs = (((read_data[0] &  0x7F)<<8) + read_data[1]) >> 1;
	
	REGISTER[memory_TEMP_MCP9801_1+sensor_index] = ((int16_t)(1-2*sign))*temp_128_abs;
	*temperature_128 = ((int16_t)(1-2*sign))*temp_128_abs;
	
	return OK;
}
int GetTemperaturePCT2075(int sensor_index, int16_t * temperature_128){	
	
	// Check index
	if(sensor_index >= (sizeof(PCT2075addr)/sizeof(uint8_t))) return SENSOR_INDEX_OOB;
	
	uint8_t read_data[2];
	
	int status = I2C_READ(MCP9801addr[sensor_index], (uint8_t [1]){0}, 1, read_data, 2);
	if(status) return status;
	
	int16_t temp_256 = (read_data[0] << 8) + read_data[1];
	
	REGISTER[memory_TEMP_PCT2075_1+sensor_index] = temp_256/2;
	*temperature_128 = temp_256/2;
	
	return OK;
}
int GetTemperatureTMP006(int sensor_index, int16_t * temperature_128){
	// Check index
	if(sensor_index >= (sizeof(TMP006addr)/sizeof(uint8_t))) return SENSOR_INDEX_OOB;
	
	uint8_t read_data[2];
	
	// Extract T_DIE
	int status = I2C_READ(TMP006addr[sensor_index], (uint8_t [1]){0x01}, 1, read_data, 2);
	if(status) return status;
	
	float T_DIE = (float)((read_data[0]<<8) + read_data[1])/128;
	
	// EXTRACT V_SENSOR
	status = I2C_READ(TMP006addr[sensor_index], (uint8_t [1]){0x00}, 1, read_data, 2);
	if(status) return status;
	
	float V_SENSOR = (float)((read_data[0]<<8) + read_data[1]);
	
	// Temperature calculation
	float T_REF = 298.15;
	float S = S0[sensor_index]*( 1 + 0.00175*( T_DIE - T_REF ) - 0.00001678*pow(T_DIE - T_REF,2));
	float V_OS = -0.0000294 - 0.00000057*(T_DIE - T_REF) + 0.00000000463*pow(T_DIE - T_REF,2);
	float f = (V_SENSOR - V_OS) + 13.4*pow(V_SENSOR - V_OS,2);
	float T_OBJ = pow(pow(T_DIE,4) + (f/S),0.25);
	
	REGISTER[memory_TEMP_TMP006_1+sensor_index] = T_OBJ*128;
	*temperature_128 = T_OBJ*128;
	
	return OK;
}

/*--------------------------------------------------
                     EXT EEPROM
--------------------------------------------------*/
// ADDRESSES OF EEPROM
const char EXT_EEPROM_ADDR[1] = {0xA0};

// PARAMETERS
#define EXT_EEPROM_MAX_ADDR 16383 //maximum number of addresses
#define EXT_EEPROM_PAGE_SIZE 64 //64 bytes per page

// ENUM
enum ext_eeprom{
	EXT_EEPROM_WRONG_ADDR = 181,
	EXT_EEPROM_OVERFLOW
	};

// FUNCTIONS
int ReadCodeinEEPROM(uint32_t eeprom_SLA_index, uint16_t eeprom_address, uint8_t * byte){
	
	// CHECK THE ADDRESS IS CORRECT
	if(eeprom_SLA_index + (uint32_t)1 > (uint32_t)(sizeof(EXT_EEPROM_ADDR)/sizeof(char))) return EXT_EEPROM_WRONG_ADDR;
	
	eeprom_address += 2; // Skip length bytes
	// CHECK THAT THE EEPROM IS LARGE ENOUGH
	if (eeprom_address > EXT_EEPROM_MAX_ADDR) return EXT_EEPROM_OVERFLOW;
	
	uint8_t byte_addr[2] = {eeprom_address>>8,eeprom_address};
	uint8_t read_bytes[1];
	int status = I2C_READ(EXT_EEPROM_ADDR[eeprom_SLA_index], byte_addr, 2, read_bytes, 1);
	if(status) return status;
	
	
	REGISTER[memory_EEPROM_CODE_BYTE] = (REGISTER[memory_EEPROM_CODE_BYTE]<<8) | (uint32_t)read_bytes[0];
	
	*byte = read_bytes[0];
	
	return OK;
}
int GetSizeofCode(uint32_t eeprom_SLA_index, int * len){
	
	// CHECK THE ADDRESS IS CORRECT
	if(eeprom_SLA_index + (uint32_t)1 > (uint32_t)(sizeof(EXT_EEPROM_ADDR)/sizeof(char))) return EXT_EEPROM_WRONG_ADDR;
	
	uint16_t eeprom_page_address = 0; //Make sure we start at beginning of page
	
	// CHECK THAT THE EEPROM IS LARGE ENOUGH
	if (eeprom_page_address + 1> EXT_EEPROM_MAX_ADDR) return EXT_EEPROM_OVERFLOW;
	
	uint8_t byte_addr[2] = {eeprom_page_address>>8,eeprom_page_address};
	uint8_t read_bytes[2];
	int status = I2C_READ(EXT_EEPROM_ADDR[eeprom_SLA_index], byte_addr, 2, read_bytes, 2);
	if(status) return status;

	REGISTER[memory_EEPROM_CODE_LENGTH] = (read_bytes[0]<<8) + read_bytes[1];
	*len = (read_bytes[0]<<8) + read_bytes[1];
	
	return OK;
}
int WriteinEEPROM(uint32_t eeprom_SLA_index, uint8_t * buffer, uint16_t len){
	
	// CHECK THE ADDRESS IS CORRECT
	if(eeprom_SLA_index + (uint32_t)1 > (uint32_t)(sizeof(EXT_EEPROM_ADDR)/sizeof(char))) return EXT_EEPROM_WRONG_ADDR;
	
	int status;
	uint16_t eeprom_page_address = 0;
	
	// CHECK THAT THE EEPROM IS LARGE ENOUGH
	if (eeprom_page_address + len - 1 > EXT_EEPROM_MAX_ADDR) return EXT_EEPROM_OVERFLOW;
	
	// WRITE CODE
	int page_num = ceil((double)len/EXT_EEPROM_PAGE_SIZE);
	uint16_t page = eeprom_page_address & 0xFFC0; //Make sure we start at beginning of page
	int word = 0;
	
	for(int II = 0; II < page_num; II++){
		// FILL THE PAGE
		uint8_t page_bytes[2+EXT_EEPROM_PAGE_SIZE];
		page_bytes[0] = page >> 8;
		page_bytes[1] = page;
		
		for (int III = 0; III < EXT_EEPROM_PAGE_SIZE; III++){
			if(word < (int)len + 2) page_bytes[2+III] = buffer[word++];
			else page_bytes[2+III] = 0xFF;
		}
		
		// SEND PAGE
		status = I2C_WRITE(EXT_EEPROM_ADDR[eeprom_SLA_index], page_bytes, 2+EXT_EEPROM_PAGE_SIZE);
		if(status) return status;
		_delay_ms(5); //delay for EEPROM to write the page
		
		// INCREMENT PAGE
		page += EXT_EEPROM_PAGE_SIZE;
	}
	
	return OK;
}

/*--------------------------------------------------
                   WATCHDOG TIMER
--------------------------------------------------*/
int WATCHDOG_INIT(void){
	cli(); // Disable interrupts
	
	// Start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE); 
	
	// Set configuration to "Interrupt + System Reset", Timer to 8s
	WDTCSR = (1<<WDIE) |(1<<WDE) | (1<<WDP3) | (1<<WDP0);
	
	sei(); // Enable interrupts
	return OK;
}
void resetWatchdogTimer(void){
	wdt_reset();
}
void DisableWatchdogTimer(void){
	wdt_disable();
}

#endif /* DRIVERS_H_ */