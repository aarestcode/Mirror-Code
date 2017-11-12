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
#include <avr/pgmspace.h>

#ifndef OK
#define OK 0
#endif

//---------------------------------------------------------------------------------------
//					             COMMUNICATION (XBEE & USB)
//---------------------------------------------------------------------------------------
// PARAMETERS
#define MessageCommandN 1 //Length of command in byte
#define MessageDataN 4 // Length of data
#define MessageChecksumN 1 // length of checksum
#define MessageN MessageCommandN+MessageDataN+MessageChecksumN // Command(1) + Data(4) + Checksum(1)
unsigned char Message[MessageN]; //Vector of received bytes
unsigned char Feedback[MessageN]; //Vector of transmitted bytes

// ERROR ENUM
enum communication{
	COMMUNICATION_INIT_CODE = 101,
	COMMUNICATION_ISCOMMANDWAITING_CODE,
	COMMUNICATION_LOADDATA_CODE,
	COMMUNICATION_SENDFEEDBACK_CODE,
	COMMUNICATION_CHECKMESSAGE_CODE,
	
	COMMUNICATION_READ_PORT,
	COMMUNICATION_WRITE_PORT,
	COMMUNICATION_CHECK_CHECKSUM
};

// FUNCTIONS
int COMMUNICATION_INIT(long timeout_ms)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | COMMUNICATION_INIT_CODE;
	
	//TODO XBEE_INIT
		
	// Initialize register
	REGISTER[memory_MESSAGE_COUNT0] = 0;
	REGISTER[memory_FEEDBACK_COUNT0] = 0;
	REGISTER[memory_MESSAGE_COUNT1] = 0;
	REGISTER[memory_FEEDBACK_COUNT1] = 0;
	
	// Timeout to read a known length message
	REGISTER[memory_COMMUNICATION_TIMEOUT] = timeout_ms;
	
	return OK;
}
int IsCommandWaiting(void)
{
	cli();
	
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | COMMUNICATION_ISCOMMANDWAITING_CODE;
	
	if(UART0_buffer_index >= (MessageN)){ // USB (priority)
		memcpy(Message, &UART0_buffer[UART0_buffer_index - (MessageN)], MessageN);
		UART0_FLUSH();
		sei();
		REGISTER[memory_MESSAGE_COUNT0] ++;
		return 1; 
	}
	if(UART1_buffer_index >= MessageN){ // XBee
		memcpy(Message, &UART1_buffer[UART1_buffer_index - (MessageN)], MessageN);
		UART1_FLUSH();
		sei();
		REGISTER[memory_MESSAGE_COUNT1] ++;
		return 2;
	} 
	
	sei();
	
	return OK;
}
int SendFeedback(int port, uint8_t address, int32_t data)
{
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | COMMUNICATION_SENDFEEDBACK_CODE;
	
	int error;
	int II;
	uint32_t sum = 0;

	// Address
	for (II = 0; II < MessageCommandN; II++)
	{
		sum += (address >> 8*(MessageCommandN-II-1)) & 0xff;
		Feedback[II] = (uint8_t)(address >> 8*(MessageCommandN-II-1));
	}
	
	// Data
	for (II = 0; II < MessageDataN; II++)
	{
		sum += (data >> 8*(MessageDataN-II-1)) & 0xff;
		Feedback[MessageCommandN + II] = (uint8_t)(data >> 8*(MessageDataN-II-1));
	}

	// Checksum
	if(MessageChecksumN)
	{
		uint32_t mask = (1<<8*MessageChecksumN) - 1;
		uint32_t checksum = mask - (sum & mask);
		
		for (II = 0; II < MessageChecksumN; II++)
		{
			Feedback[MessageCommandN + MessageDataN + II] = (uint8_t)(checksum >> 8*(MessageChecksumN-II-1));
		}
	}

	
	// Send
	if (port==1){
		for(II = 0; II < MessageN; II++){
			error = UART0_WRITE(Feedback[II]);
			if(error) return error;
		}
		REGISTER[memory_FEEDBACK_COUNT0]++;
	}
	else if (port==2){
		for(II = 0; II < MessageN; II++){
			error = UART1_WRITE(Feedback[II]);
			if(error) return error;
		}
		REGISTER[memory_FEEDBACK_COUNT1]++;
	}
	else return COMMUNICATION_WRITE_PORT;

	return OK;
}
int CheckMessage(int port, uint8_t * command, int32_t * data, uint16_t * checksum)
{
	//REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | COMMUNICATION_CHECKMESSAGE_CODE;
	
	// Checksum
	if(MessageChecksumN)
	{
		uint32_t sum = 0;
		for(int II = 0; II < (MessageN)-(MessageChecksumN); II++)
		sum += Message[II];
		
		uint32_t check = 0;
		for (int II = 0; II < (MessageChecksumN); II++)
		check |= (Message[(MessageCommandN)+(MessageDataN)+II] << 8*((MessageChecksumN)-II-1));
		
		sum += check;
		
		uint32_t mask = (1 << 8*(MessageChecksumN)) - 1;
		*checksum = sum & mask;
		
		if(*checksum != mask)
		{
			if (port == 1) UART0_FLUSH();
			if (port == 2) UART1_FLUSH();
			
			return COMMUNICATION_CHECK_CHECKSUM;
		}
	}
	
	// Command
	*command = 0;
	for (int II = 0; II < (MessageCommandN); II++)
	*command |= (Message[II] << 8*((MessageCommandN)-II-1));
	
	// Data
	*data = 0;
	for (int II = 0; II < (MessageDataN); II++)
	*data |= ((int32_t)Message[(MessageCommandN)+II] << 8*((MessageDataN)-II-1));
	
	return OK;
}
int LoadData(int port, uint8_t * buffer, uint16_t numbytes)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | COMMUNICATION_LOADDATA_CODE;
	cli(); // Could not make use of interrupt for unknown reason. So I;m doing it the old way.
	
	if (port == 1){
		for (int II = 0; II < numbytes; II++) UART0_READ(&buffer[II]);
		UART0_FLUSH();
		sei();
	}
	else if (port == 2){
		for (int II = 0; II < numbytes; II++) UART1_READ(&buffer[II]);
		UART1_FLUSH();
		sei();
	}
	else {
		sei();
		return COMMUNICATION_READ_PORT;
	}
	
	return OK;
}


//---------------------------------------------------------------------------------------
//					                    POWER/HV BOARD
//---------------------------------------------------------------------------------------
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
int ActivateHV(uint8_t measure_count);
int DeactivateHV(void);
int ActivatePICOV(bool withEncoders);
int SetVoltage(uint16_t voltage);
int SetBias(uint16_t voltage);
int EnableSV(int port, bool state);
int EnableCL(int port, bool state);
int MeasureV(int port, uint16_t* val);
bool IsCLFault(int port);

// ERROR ENUM
enum power_driver{
	POWER_INIT_CODE = 111,
	POWER_ACTIVATEHV_CODE,
	POWER_DEACTIVATEHV_CODE,
	POWER_ACTIVATEPICO_CODE,
	POWER_DEACTIVATEPICO_CODE,
	POWER_SETVOLTAGE_CODE,
	POWER_SETBIAS_CODE,
	POWER_ENABLESV_CODE,
	POWER_ENABLECL_CODE,
	POWER_MEASUREV_CODE,
	POWER_ISCLFAULT_CODE,
	
	VAR_FEEDBACK_OOB,
	BIAS_FEEDBACK_OOB,
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
int POWER_INIT(void)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_INIT_CODE;
		
	// Set tolerance on ADC feedback
	REGISTER[memory_HV_TOL_V] = 50; // 0.15V tolerance over 3.3V max

	// Set Supply voltage enable as output
	DDR_SV |= ((1<<FIVE_V_E) | (1<<TWELVE_V_E) | (1<<TWO_EIGHT_V_E));
	
	// Disable Supply Voltages
	int error;
	error = EnableSV(TWELVE_V_E,false);
	if(error) return error;
	error = EnableSV(FIVE_V_E,false);
	if(error) return error;
	error = EnableSV(TWO_EIGHT_V_E,false);
	if(error) return error;
	
	// Set Current Limiters Enable as output
	DDR_CL_E |= ((1<<CL1_E) | (1<<CL2_E) | (1<<CL3_E));
	
	// Disable Current Limiters
	error = EnableCL(CL1_E,false);
	if(error) return error;
	error = EnableCL(CL2_E,false);
	if(error) return error;
	error = EnableCL(CL3_E,false);
	if(error) return error;
		
	// Set Current Limiters Fault as inputs
	DDR_CL_F1 &= ~((1<<CL2_F) | (1<<CL3_F));
	DDR_CL_F2 &= ~(1<<CL1_F);
	
	return OK;
}
int ActivateHV(uint8_t measure_count)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_ACTIVATEHV_CODE;
	
	int error;
	
	// 1) Enable 5V
	error = EnableSV(FIVE_V_E,true);
	if(error) return error;
	
	// 2) Command variable HV to 0V
	error = SetVoltage(0x3f00);
	if(error) return error;	
	
	// 3) Command ground to 0V
	error = SetBias(0x3f00);
	if(error) return error;
	
	// 4) Enable 12V
	error = EnableSV(TWELVE_V_E,true);
	if(error) return error;
	
	// 5) Enable CL3
	error = EnableCL(CL3_E,true);
	if(error) return error;
	
	// 6) Check HV_VOLTAGE at 2.5V
	_delay_ms(1500);
	uint16_t val;
	error = MeasureV(HV_VOLTAGE,&val);
	if(error) return error;
	uint8_t counter = 0;
	while ( (abs(val-TWO_FIVE_V) > REGISTER[memory_HV_TOL_V]) && (counter < measure_count) ) {
		error = MeasureV(HV_VOLTAGE,&val);
		if(error) return error;
		counter++;
	}
	if ( counter == measure_count ){
		// Disable CL3
		error = EnableCL(CL3_E,false);
		if(error) return error;
		
		return VAR_FEEDBACK_OOB;
	}
		
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
	_delay_ms(1500);
	error = MeasureV(HV_GROUND,&val);
	if(error) return error;
	counter = 0;
	while ( (abs(val-TWO_FIVE_V) > REGISTER[memory_HV_TOL_V]) && ( counter < measure_count )) {
		error = MeasureV(HV_GROUND,&val);
		if(error) return error;
		counter++;
	}
	if ( counter == measure_count ){
		// Disable CL2
		error = EnableCL(CL2_E,false);
		if(error) return error;
		
		return BIAS_FEEDBACK_OOB;
	}
	
	// 10) Check CL2 //TODO
	/*if(IsCLFault(2)){
		// Disable CL2
		error = EnableCL(CL2_E,false);
		if(error) return error;
		
		return CL2_FAULT;
	}*/
	
	return OK;
}
int DeactivateHV(void)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_DEACTIVATEHV_CODE;
	
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
int ActivatePICOV(bool withEncoders)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_ACTIVATEPICO_CODE;
	
	int error;
	
	// 1) Enable 12V
	error = EnableSV(TWELVE_V_E,true);
	if(error) return error;
	
	// 2) Enable CL1
	error = EnableCL(CL1_E,true);
	if(error) return error;
	
	// 3) Check PICOMOTOR_VOLTAGE at 2.5V !!! TAKES ABOUT 1.5 sec TO STABILIZE !!!
	_delay_ms(1500);
	uint16_t val;
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
int DeactivatePICOV(void)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_DEACTIVATEPICO_CODE;
	
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
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_SETVOLTAGE_CODE;
	
	int error = SPI_WRITE(SELECT_HV,(uint8_t [2]){voltage>>8, voltage},2);
	if(error) return error;
	
	REGISTER[memory_HV] = voltage;
	return OK;
}
int SetBias(uint16_t voltage)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_SETBIAS_CODE;
	
	int error = SPI_WRITE(SELECT_BIAS,(uint8_t [2]){voltage>>8, voltage},2);
	if(error) return error;
	
	REGISTER[memory_GND] = voltage;
	return OK;
}
int EnableSV(int port, bool state)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_ENABLESV_CODE;
	
	// PORT = FIVE_V_E or TWELVE_V_E or TWO_EIGHT_V_E
	// State = true means enable // State = false means disable
	if(state) PORT_SV |= (1<<port);
	else PORT_SV &= ~(1<<port);
	
	return OK;
}
int EnableCL(int port, bool state)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_ENABLECL_CODE;
	
	// PORT = CL1_E or CL2_E or CL3_E
	// State = true means enable // State = false means disable
	if(state) PORT_CL_E &= ~(1<<port);
	else PORT_CL_E |= (1<<port);
	
	return OK;
}
int MeasureV(int port, uint16_t* val)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_MEASUREV_CODE;
	
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
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | POWER_ISCLFAULT_CODE;
	
	// CL_index = 1, 2 or 3
	if (CL_index==1) return (PIN_CL_F2 & (1<<CL1_F));
	if (CL_index==2) return (PIN_CL_F1 & (1<<CL2_F));
	if (CL_index==3) return (PIN_CL_F1 & (1<<CL3_F));
	else return CL_INDEX_OOB;
}

//---------------------------------------------------------------------------------------
//					                   SEPARATION DEVICE
//---------------------------------------------------------------------------------------
// PINSET
#define DDR_SD_TRIG DDRB
#define PORT_SD_TRIG PORTB
#define SEP_DEV_TRIG PORTB3

#define DDR_SD_DET DDRC
#define PIN_SD_DET PINC
#define SEP_DEV_DET PORTC7

// ERROR ENUM
enum seperation_device{
	SEP_DEV_INIT_CODE = 141,
	SEP_DEV_RELEASE_CODE,
	SEP_DEV_ISCONSTRAINED_CODE,
	
	SEPARATION_DEV_TIMEOUT
};

bool IsMirrorConstrained(void);

// FUNCTIONS
int SEP_DEV_INIT(void)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SEP_DEV_INIT_CODE;
	
	// Set pin as output
	DDR_SD_TRIG |= (1<<SEP_DEV_TRIG);
	
	// Set pin to 0
	PORT_SD_TRIG &= ~(1<<SEP_DEV_TRIG);
	
	// Set detection as input
	DDR_SD_DET &= ~(1<<SEP_DEV_DET);
	
	return OK;
} 
int ReleaseMirror(uint32_t timeout_ms)
{	
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SEP_DEV_RELEASE_CODE;
	
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
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SEP_DEV_ISCONSTRAINED_CODE;
	
	// TODO: IsMirrorConstrained
	return 1;// (PIN_SD_DET & (1<<SEP_DEV_DET));
}

//---------------------------------------------------------------------------------------
//						             PICOMOTORS/ENCODERS
//---------------------------------------------------------------------------------------
// MCP23S17
uint8_t IOEaddr = 0x40; // Address of the IO Expander (LSB = 0 for WRITE operations)
uint8_t IOEport[3] = {0x12,0x13,0x13}; // Address of the port to which each picomotor is connected (pico1, pico2, pico3)
uint8_t IOEpin[12] = {1,0,3,2,1,0,3,2,5,4,7,6}; // Pin of each port to which the switch is connected (pico1_FW_HIGH, pico1_FW_LOW, pico1_BW_HIGH, pico1_BW_LOW, ...)

// ADG715
uint8_t ENCODERSWITCHaddr = 0x90;

// ENCODER PINSET
#define DDR_ENCODER DDRC
#define PIN_ENCODER PINC
#define ENCODERA PINC2
#define ENCODERB PINC3

// ENCODER STATES
enum EncoderState {state00,state10,state11,state01}; 

// ERROR ENUM
enum picomotor_driver{
	PICOMOTORS_INIT_CODE = 151,
	PICOMOTORS_MOVE_CODE,
	PICOMOTORS_GETENCODER_CODE,
	
	ENCODER_STATE_CRITICAL
	};

// FUCTIONS
int PICOMOTORS_INIT(void)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTORS_INIT_CODE;
	
	int error;
	
	// Set Encoders as inputs
	DDR_ENCODER &= ~((1<<ENCODERA) | (1<<ENCODERB));
	
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
	
	// Encoder
	error = I2C_WRITE(ENCODERSWITCHaddr, (uint8_t [1]){0}, 1); // Open all switches
	if (error) return error;

	return OK;
}
int MovePicomotor(int index, int32_t ticks)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTORS_MOVE_CODE;
	
	// INFO: @4MHz SPI clock, full message takes 37us to be sent
	//ticks < 0 <=> BACKWARD
	//ticks > 0 <=> FORWARD
	int error = 0;
	int32_t II = 0;
	
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
int GetEncoderState(int index, uint8_t* state)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | PICOMOTORS_GETENCODER_CODE;
	
	// INPUT  index = 0 or 1 or 2 depending on the encoder
	// OUTPUT state = state00 or state10 or state11 or state01 (depending of state of channel A and B resp.)
	
	if( REGISTER[memory_ENCODER_SELECT] != index ){
		//TODO: Select channel
		int error = I2C_WRITE(ENCODERSWITCHaddr, (uint8_t [1]){3 << (2*index)}, 1);
		if (error) return error;
		REGISTER[memory_ENCODER_SELECT] = index;
	}
		
	if ((PIN_ENCODER & (1<<ENCODERA)) && (PIN_ENCODER & (1<<ENCODERB))) {REGISTER[memory_ENCODER0_STATE + index] = REGISTER[memory_ENCODER0_STATE + index] << 4 | state11; *state = state11; return OK;}
	if ((PIN_ENCODER & (1<<ENCODERA)) && !(PIN_ENCODER & (1<<ENCODERB))) {REGISTER[memory_ENCODER0_STATE + index] = REGISTER[memory_ENCODER0_STATE + index] << 4 | state10; *state = state10; return OK;}
	if (!(PIN_ENCODER & (1<<ENCODERA)) && (PIN_ENCODER & (1<<ENCODERB))) {REGISTER[memory_ENCODER0_STATE + index] = REGISTER[memory_ENCODER0_STATE + index] << 4 | state01; *state = state01; return OK;}
	if (!(PIN_ENCODER & (1<<ENCODERA)) && !(PIN_ENCODER & (1<<ENCODERB))) {REGISTER[memory_ENCODER0_STATE + index] = REGISTER[memory_ENCODER0_STATE + index] << 4 | state00; *state = state00; return OK;}
	
	return ENCODER_STATE_CRITICAL;
}

//---------------------------------------------------------------------------------------
//                                       MULTIPLEXER
//---------------------------------------------------------------------------------------
// ADRESSES OF I/O EXPANDERS
uint8_t MPaddr[2] = {0x98,0x84}; // Multiplexer I2C addresses (connected to SCL, SDA, V+). LSB is irrelevant (7-bit address in bit 7 to bit 1)
	
// ADRESSES OF SWITCHES
const bool MPIC[42] =   {   1,   1,   0,   0,   0,   0,   1,   0,   0,   1,   1,   0,   1,   1,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   0,   1,   0,   1,   0,   1,   0,   1,   1}; // I/O expander
uint8_t MPport[42] = {0x3F,0x3E,0x28,0x3E,0x39,0x38,0x34,0x2D,0x2F,0x39,0x3D,0x3C,0x30,0x2D,0x24,0x3A,0x2A,0x29,0x30,0x3D,0x2F,0x26,0x31,0x33,0x38,0x2E,0x2B,0x3A,0x25,0x33,0x3B,0x37,0x2C,0x31,0x3B,0x27,0x2E,0x3F,0x2C,0x32,0x3C,0x32};
	
// ERROR ENUM
enum mux_driver{
	MULTIPLEXER_INIT_CODE = 161,
	MULTIPLEXER_CHANNELON_CODE,
	MULTIPLEXER_CHANNELOFF_CODE,
};
	
// FUNCTIONS
int MULTIPLEXER_INIT(uint8_t i)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | MULTIPLEXER_INIT_CODE;
	
	// Integer to return
	int status;

	// Set mode to normal
	status = I2C_WRITE(MPaddr[i],(uint8_t [2]){0x04, 0x01},2); //Send message
	if(status) return status;
	
	// Set global current to 10.5mA for 0x06, 12mA for 0x07
	status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x02, 0x06},2); //Send message
	if(status) return status;
	
	// Set all pins to LED segment driver configuration (LED = switch)
	for (uint8_t cmd = 0x09; cmd <= 0x0F; cmd++)
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
	for (uint8_t cmd = 0x44; cmd <= 0x5c; cmd += 8)
	{
		status = I2C_WRITE(MPaddr[i], (uint8_t [2]){cmd, 0},2); //Send message
		if(status) return status;
	}
	return OK;
}
int ChannelOn(uint8_t ch)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | (uint32_t)MULTIPLEXER_CHANNELON_CODE;
	
	int status = I2C_WRITE(MPaddr[MPIC[ch]], (uint8_t [2]){MPport[ch], 1},2); //Send message
	if(!status) REGISTER[memory_MUX_ACTIVE_CH] = ch;
	return status;
}
int ChannelOff(uint8_t ch)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | (uint32_t)MULTIPLEXER_CHANNELOFF_CODE;
	
	int status = I2C_WRITE(MPaddr[MPIC[ch]], (uint8_t [2]){MPport[ch], 0},2); //Send message
	if(!status) REGISTER[memory_MUX_ACTIVE_CH] = -1;
	return status;
}

//---------------------------------------------------------------------------------------
//					                   THERMO-SENSORS
//---------------------------------------------------------------------------------------
// ADDRESSES OF SENSORS
const uint8_t MCP9801addr[1] = {0x9E}; // const uint8_t MCP9801addr[2] = {0x9E, 0x92}; 
const uint8_t PCT2075addr[0] = {}; // const uint8_t PCT2075addr[2] = {0x9E, 0x90}; 
const uint8_t TMP006addr[3] = {0x80, 0x82, 0x8A};

	
// PARAMETERS
const double S0[3] = {0.00000000000006, 0.00000000000006, 0.00000000000006}; // TODO: Calibrate S0

// ENUM
enum temp_sensors{
	TEMP_SENSORS_INIT_CODE = 171,
	TEMP_SENSORS_GETMCP9801_CODE,
	TEMP_SENSORS_GETPCT2075_CODE,
	TEMP_SENSORS_GETTMP006_CODE,
	
	TEMP_SENSORS_INDEX_OOB,
};

// FUNCTIONS
int TEMP_SENSORS_INIT(uint8_t index)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | TEMP_SENSORS_INIT_CODE;
	
	// index = 0 -> initialize all
	// index > 0 -> initialize 1 thermo sensor only (index depends on number of sensors)
	int status;
	int error = 0;
	int start, end;
	
	int MCP9801addr_len = sizeof(MCP9801addr)/sizeof(uint8_t);
	int PCT2075addr_len = sizeof(PCT2075addr)/sizeof(uint8_t);
	int TMP006addr_len = sizeof(TMP006addr)/sizeof(uint8_t);
	
	// MCP9801 (12-bit precision)
	if ( index == 0 ) {start = 0; end = MCP9801addr_len;}
	else if ( (index - 1) < MCP9801addr_len ) {start = index - 1; end = index;}
	else {start = 0; end = 0;}
	for(int II = start; II < end ; II++){
		status = I2C_WRITE(MCP9801addr[II], (uint8_t [2]){0x01, 0b01100000}, 2);
		if ( status ) error = status;
	}
	
	// PCT2075
	if ( index == 0 ) {start = 0; end = PCT2075addr_len;}
	else if ( (index > MCP9801addr_len) && ((index - MCP9801addr_len - 1) < PCT2075addr_len) ) {start = index - MCP9801addr_len - 1; end = index - MCP9801addr_len;}
	else {start = 0; end = 0;}
	for(int II = start; II < end; II++){
		status = I2C_WRITE(PCT2075addr[II], (uint8_t [2]){0x01, 0b00000000}, 2); // Normal reading mode
		if ( status ) error = status;
		status = I2C_WRITE(PCT2075addr[II], (uint8_t [2]){0x04, 0b00000001}, 2); // Period to measure temperature = 100 ms
		if ( status ) error = status;
	}
	
	// TMP006
	if ( index == 0 ) {start = 0; end = TMP006addr_len;}
	else if ( (index > (MCP9801addr_len + PCT2075addr_len)) && ((index - MCP9801addr_len - PCT2075addr_len - 1) < TMP006addr_len) ) {start = index - MCP9801addr_len - PCT2075addr_len - 1; end = index - MCP9801addr_len - PCT2075addr_len;}
	else {start = 0; end = 0;}
	for(int II = start; II < end; II++){
		status = I2C_WRITE(TMP006addr[II], (uint8_t [3]){0x02, 0x74, 0}, 3);
		if ( status ) error = status;
	}
	
	return error;
}
int GetTemperatureMCP9801(int sensor_index, int16_t * temperature_128)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | TEMP_SENSORS_GETMCP9801_CODE;	
	
	// Check index
	if(sensor_index >= (sizeof(MCP9801addr)/sizeof(uint8_t))) return TEMP_SENSORS_INDEX_OOB;
	
	uint8_t read_data[2];
	
	int status = I2C_READ(MCP9801addr[sensor_index], (uint8_t [1]){0}, 1, read_data, 2);
	if(status) return status;
	
	uint16_t sign = (read_data[0] & 0x80) && (0x80); // 0 = +, 1 == -
	uint16_t temp_128_abs = (((read_data[0] &  0x7F)<<8) + read_data[1]) >> 1;
	
	REGISTER[memory_TEMP_MCP9801_1+sensor_index] = ((int16_t)(1-2*sign))*temp_128_abs;
	*temperature_128 = ((int16_t)(1-2*sign))*temp_128_abs;
	
	return OK;
}
int GetTemperaturePCT2075(int sensor_index, int16_t * temperature_128)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | TEMP_SENSORS_GETPCT2075_CODE;	
	
	// Check index
	if(sensor_index >= (sizeof(PCT2075addr)/sizeof(uint8_t))) return TEMP_SENSORS_INDEX_OOB;
	
	uint8_t read_data[2];
	
	int status = I2C_READ(PCT2075addr[sensor_index], (uint8_t [1]){0}, 1, read_data, 2);
	if(status) return status;
	
	int16_t temp_256 = (read_data[0] << 8) + read_data[1];
	
	REGISTER[memory_TEMP_PCT2075_1+sensor_index] = temp_256/2;
	*temperature_128 = temp_256/2;
	
	return OK;
}
int GetTemperatureTMP006(int sensor_index, int16_t * temperature_128)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | TEMP_SENSORS_GETTMP006_CODE;
	
	// Check index
	if(sensor_index >= (sizeof(TMP006addr)/sizeof(uint8_t))) return TEMP_SENSORS_INDEX_OOB;
	
	uint8_t read_data[2];
	
	// Extract T_DIE
	int status = I2C_READ(TMP006addr[sensor_index], (uint8_t [1]){0x01}, 1, read_data, 2);
	if(status) return status;
	
	int16_t data = (read_data[0]<<8) + read_data[1];
	double T_DIE = data / 128.0 + 273.15; // in Kelvin
	
	// EXTRACT V_SENSOR
	status = I2C_READ(TMP006addr[sensor_index], (uint8_t [1]){0x00}, 1, read_data, 2);
	if(status) return status;
	
	data = (read_data[0]<<8) + read_data[1];
	double V_SENSOR = data * 0.00000015625; // in Volt
	
	// Temperature calculation
	double T_REF = 298.15; // in Kelvin
	double S = S0[sensor_index]*( 1 + 0.00175*( T_DIE - T_REF ) - 0.00001678*pow(T_DIE - T_REF,2));
	double V_OS = -0.0000294 - 0.00000057*(T_DIE - T_REF) + 0.00000000463*pow(T_DIE - T_REF,2);
	double f = (V_SENSOR - V_OS) + 13.4*pow(V_SENSOR - V_OS,2);
	double T_OBJ = pow(pow(T_DIE,4) + (f/S),0.25) - 273.15; // in Celsius
	
	REGISTER[memory_TEMP_TMP006_1+sensor_index] = (int32_t)(T_OBJ*128);
	*temperature_128 = T_OBJ*128;
	
	return OK;
}

//---------------------------------------------------------------------------------------
//					                     EXT EEPROM
//---------------------------------------------------------------------------------------
// ADDRESSES OF EEPROM
const char EXT_EEPROM_ADDR = 0xA0;

// PARAMETERS
#define EXT_EEPROM_PAGESIZE 256 //64 bytes per page
typedef enum {
	code0,
	code1,
	code2,
	code3
}CodeID_t;

// ENUM
enum ext_eeprom{
	EXT_EEPROM_READ = 181,
	EXT_EEPROM_SIZE,
	EXT_EEPROM_WRITE_INFO,
	EXT_EEPROM_WRITE_PAGE
};

int ReadDWordFromEEPROM(CodeID_t CodeID, uint16_t addr, uint32_t * dword){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | EXT_EEPROM_READ;

	uint8_t byte_addr[2] = {(addr >> 8) + 1, addr & 0xFF};
	uint8_t bytes[4] = {};
	int status = I2C_READ(EXT_EEPROM_ADDR + (CodeID<<1), byte_addr, 2, bytes, 4);
	if(status) return status;
	
	*dword = (((uint32_t)bytes[3]) << 24) + (((uint32_t)bytes[2]) << 16) + (((uint32_t)bytes[1]) << 8) + ((uint32_t)bytes[0]);
	REGISTER[memory_EEPROM_CODE_DWORD] = (uint32_t)*dword;
		
	return OK;
}
int GetSizeofCode(CodeID_t CodeID, uint8_t * Npages){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | EXT_EEPROM_SIZE;
	
	uint8_t byte_addr[2] = {0,0};
	uint8_t read_bytes[1];
	int status = I2C_READ(EXT_EEPROM_ADDR + (CodeID<<1), byte_addr, 2, read_bytes, 1);
	if(status) return status;

	*Npages = read_bytes[0];
	REGISTER[memory_EEPROM_CODE_LENGTH] = (uint32_t)(*Npages);
	
	return OK;
}
int WriteCodeInfoinEEPROM(CodeID_t CodeID, uint8_t Npages){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | EXT_EEPROM_WRITE_INFO;
	
	uint8_t data[2 + 1];
	data[0] = 0; // First page = code data
	data[1] = 0;
	data[2] = Npages;
	
	int status = I2C_WRITE(EXT_EEPROM_ADDR + (CodeID<<1), data, 2 + 1);
	if(status) return status;
	
	return OK;
}
int WritePageInEEPROM(CodeID_t CodeID, uint8_t page, uint8_t buffer[EXT_EEPROM_PAGESIZE]){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | EXT_EEPROM_WRITE_PAGE;
	
	uint8_t data[2 + EXT_EEPROM_PAGESIZE];
	data[0] = page + 1; // First page = code data
	data[1] = 0;
	memcpy(&(data[2]), buffer, EXT_EEPROM_PAGESIZE);
	int status = I2C_WRITE(EXT_EEPROM_ADDR + (CodeID<<1), data, 2 + EXT_EEPROM_PAGESIZE);
	if(status) return status;
	
	return OK;
}

//---------------------------------------------------------------------------------------
//				                      WATCHDOG TIMER
//---------------------------------------------------------------------------------------
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