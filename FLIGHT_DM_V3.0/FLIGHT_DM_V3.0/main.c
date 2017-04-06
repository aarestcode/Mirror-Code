/*
 * FLIGHT_RM_V1.c
 *
 * Created: 6/14/2016 2:48:57 PM
 * Author : Thibaud
 */ 

#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)

#include "Memory.h"
#include "Interfaces.h"
#include "Drivers.h"
#include "Algorithms.h"

int ParseCommand(int port)
{
	/*--------------------------------------------------
                       MESSAGE CHECK
	--------------------------------------------------*/
	int II;
	
	// Checksum
	if(MessageChecksumN)
	{
		unsigned int sum = 0;
		for(II = 0; II < MessageN-MessageChecksumN; II++) sum += Message[II];
		
		unsigned int checksum = 0;
		for (II = 0; II < MessageChecksumN; II++) checksum |= (Message[MessageCommandN+MessageDataN+II] << 8*(MessageChecksumN-II-1));
		
		sum += checksum;
		
		unsigned int mask = (1 << 8*MessageChecksumN) - 1;
		checksum = sum & mask;
		
		if(checksum != mask)
		{
			int error = SendFeedback(port,'C',checksum);
			if(error) return error;
		}

	}
	
	// Command
	unsigned int command = 0;
	for (II = 0; II < MessageCommandN; II++) command |= (Message[II] << 8*(MessageCommandN-II-1));
	
	// Data
	signed long data = 0;
	for (II = 0; II < MessageDataN; II++) data |= ((int32_t)Message[MessageCommandN+II] << 8*(MessageDataN-II-1));


	/*--------------------------------------------------
                           PRIVATE
	--------------------------------------------------*/
	// command = 0
	if (command	== 0){
		int error = SendFeedback(port,0,0xAA12e570); //Send back AAReST written in Hex
		if(error) return error;
	}

	/*--------------------------------------------------
                       REGISTER WRITE
	--------------------------------------------------*/
	// command = 1-149
	else if (command < 150)
	{	
		REGISTER[command] = data;
		int error = SendFeedback(port,command,0);
		if(error) return error;	
	}

	/*--------------------------------------------------
                       REGISTER READ
	--------------------------------------------------*/
	// command = 150
	else if(command == 150){
		int error = SendFeedback(port,data,REGISTER[data]);
		if(error) return error;
	}
	
	/*--------------------------------------------------
                          ACTIONS
	--------------------------------------------------*/
	// command = 151-239
	
	// RE-INITIALIZE USART0
	else if (command==151){
		int status = USART0_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE USART1
	else if (command==152){
		int status = USART1_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE SPI
	else if (command==153){
		int status = SPI_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE I2C
	else if (command==154){
		int status = I2C_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE ADC
	else if (command==155){
		int status = ADC_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE COMMUNICATIONS
	else if (command==156){
		int status = COMMUNICATION_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE POWER
	else if (command==160){
		int status = POWER_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ACTIVATE ELECTRODE HV
	else if (command==161){
		int status = ActivateHV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DEACTIVATE ELECTRODE HV
	else if (command==162){
		int status = DeactivateHV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ACTIVATE PICOMOTOR HV
	else if (command==163){
		int status = ActivatePICOV(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DEACTIVATE PICOMOTOR HV
	else if (command==164){
		int status = DeactivatePICOV();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// CHANGE VARIABLE HV
	else if (command==165){
		int status = SetVoltage(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// CHANGE BIAS HV
	else if (command==166){
		int status = SetBias(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ENABLE SUPPLY VOLTAGE
	else if (command==167){
		int status = EnableSV(data,true);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DISABLE SUPPLY VOLTAGE
	else if (command==168){
		int status = EnableSV(data,false);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ENABLE CURRENT LIMITER
	else if (command==169){
		int status = EnableCL(data,true);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DISABLE CURRENT LIMITER
	else if (command==170){
		int status = EnableCL(data,false);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// MEASURE FB VOLTAGE
	else if (command==171){
		int val;
		int status = MeasureV(data,&val);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// CURRENT LIMITER FAULT
	else if (command==172){
		int status = IsCLFault(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE SEPERATION DEVICE
	else if (command==175){
		int status = SEP_DEV_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RELEASE SEPERATION DEVICE
	else if (command==176){
		int status = ReleaseMirror(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// SEPERATION DEVICE OFF
	else if (command==177){
		int status = IsMirrorConstrained();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE PICOMOTORS DRIVER
	else if (command==179){
		int status = PICOMOTORS_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE PICOMOTORS ESTIMATION ALGORITHM
	else if (command==180){
		int status = PICOMOTOR_ESTIMATION_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// LEFT PICOMOTOR
	else if(command==181){ // MOVE BY TICKS
		int status = MovePicomotor(0,data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==182){ // MOVE BY INTERVALS
		int MovedIntervals, MovedTicks;
		int status = MoveIntervals(0, data, &MovedIntervals, &MovedTicks);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==183){ // MOVE BY NM (THROUGH ALGORITHM)
		int status = SetPicomotorLocation(0, REGISTER[memory_PICO0_LOCATION], data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==184){ // INITIALIZE
		int status = InitializePicomotor(0, data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==185){ // CALIBRATE
		float mean, std;
		int status = CalibratePicomotor(0, data, &mean, &std);
		if(!status){
			REGISTER[memory_PICO0_MEAN] =  (long)(mean*1000000);
			REGISTER[memory_PICO0_STD] =  (long)(  std*1000000);
		}
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==186){ // MEASURE ENCODER STATE
		int state;
		int status = GetEncoderState(0, &state);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RIGHT PICOMOTOR
	else if(command==191){ // MOVE BY TICKS
		int status = MovePicomotor(1,data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==192){ // MOVE BY INTERVALS
		int MovedIntervals, MovedTicks;
		int status = MoveIntervals(1, data, &MovedIntervals, &MovedTicks);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==193){ // MOVE BY NM (THROUGH ALGORITHM)
		int status = SetPicomotorLocation(1, REGISTER[memory_PICO1_LOCATION], data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==194){ // INITIALIZE
		int status = InitializePicomotor(1, data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==195){ // CALIBRATE
		float mean, std;
		int status = CalibratePicomotor(1, data, &mean, &std);
		if(!status){
			REGISTER[memory_PICO1_MEAN] =  (long)(mean*1000000);
			REGISTER[memory_PICO1_STD] =  (long)(std*1000000);
		}
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==196){ // MEASURE ENCODER STATE
		int state;
		int status = GetEncoderState(1, &state);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// BOTTOM PICOMOTOR
	else if(command==201){ // MOVE BY TICKS
		int status = MovePicomotor(2,data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==202){ // MOVE BY INTERVALS
		int MovedIntervals, MovedTicks;
		int status = MoveIntervals(2, data, &MovedIntervals, &MovedTicks);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==203){ // MOVE BY NM (THROUGH ALGORITHM)
		int status = SetPicomotorLocation(2, REGISTER[memory_PICO2_LOCATION], data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==204){ // INITIALIZE
		int status = InitializePicomotor(2, data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==205){ // CALIBRATE
		float mean, std;
		int status = CalibratePicomotor(2, data, &mean, &std);
		if(!status) {
			REGISTER[memory_PICO2_MEAN] = (long)(mean*1000000);
			REGISTER[memory_PICO2_STD] =  (long)(std*1000000);
		}
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	else if(command==206){ // MEASURE ENCODER STATE
		int state;
		int status = GetEncoderState(2, &state);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE MUX
	else if (command==210){
		int status = MULTIPLEXER_INIT(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// TURN CHANNEL ON
	else if (command==211){ 
		int status = ChannelOn(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	// TURN CHANNEL OFF
	else if (command==212){ 
		int status = ChannelOff(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE ELECTRODE ALGORITHM
	else if (command==213){
		int status = ELECTRODE_ACTUATION_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// ACTUATE ELECTRODE
	else if (command==214){
		int status = ActuateElectode(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE THERMO-SENSORS
	else if (command==220){
		int status = TEMP_SENSORS_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// MEASURE TEMP FROM MCP9801
	else if(command==221){
		int16_t temp;
		int status = GetTemperatureMCP9801(data, &temp);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// MEASURE TEMP FROM TMP006
	else if(command==222){
		int16_t temp;
		int status = GetTemperatureTMP006(data, &temp);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// RE-INITIALIZE WATCHDOG TIMER
	else if (command==230){
		int status = WATCHDOG_INIT();
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// DISABLE WATCHDOG TIMER
	else if (command==231){
		DisableWatchdogTimer();
		int error = SendFeedback(port,command,0);
		if(error) return error;
	}
	
	/*--------------------------------------------------
                       SPECIAL COMMANDS
	--------------------------------------------------*/
	// command = 240-255

	// SAVE MEMORY
	else if(command==240){
		int status = SaveRegister(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}

	// LOAD MEMORY
	else if(command==241){
		int status = LoadRegister(data);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// WRITE CODE TO EEPROM
	else if(command==245){
		uint16_t length = data & 0xffff;
		uint32_t eeprom_index = data >> 16;
		uint8_t buffer[length + 2];
		buffer[0] = length >> 8;
		buffer[1] = length;
		int error = SendFeedback(port,command,0);
		if(error) return error;	
		
		int status = LoadMessage(port, &(buffer[2]), length,(long)10000);
		if(status==0) status = WriteinEEPROM(eeprom_index, buffer, length);
		error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// GET SIZE OF CODE IN EEPROM
	else if(command==246){
		int length;
		int status = GetSizeofCode(data, &length);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	// READ BYTE OF CODE IN EEPROM
	else if(command==247){
		uint8_t byte;
		int status = ReadCodeinEEPROM(data>>16, data & 0xffff, &byte);
		int error = SendFeedback(port,command,status);
		if(error) return error;
	}
	
	//PING
	else if(command==255){
		int error = SendFeedback(port,command,data);
		if(error) return error;
	}
	
	// WRONG COMMAND
	else{
		int error = SendFeedback(port,254,command);
		if(error) return error;	
	}
	
	return OK;
}

ISR(WDT_vect){
	// Interrupt before power-up from watchdog
	// Saves the register
	SaveRegister(0);
}

int main(void)
{	
	LoadRegister(0);

	USART0_INIT(9600);
	USART1_INIT(9600);
    	SPI_INIT(4000000);
	I2C_INIT(200000);
	
	COMMUNICATION_INIT(1000);
	POWER_INIT();
    	PICOMOTORS_INIT();
	//MULTIPLEXER_INIT(0);
	//MULTIPLEXER_INIT(1);
	SEP_DEV_INIT();
	//TEMP_SENSORS_INIT();
	
	//PICOMOTOR_ESTIMATION_INIT(100);
	//ELECTRODE_ACTUATION_INIT();
	
	int ch = 0;
	int status;
	int port;
	
    while (1)
    {	
		// Receive telecommand (if any)	
		if((port=IsCommandWaiting())){
				status = SaveCommand(port);
				if(status == 0) ParseCommand(port);
		}
		/*
		// Actuate the electrode
		if(REGISTER[memory_ELECTRODE1 + ch]){
			status = ActuateElectode(ch);
			if(status) REGISTER[memory_ELECTRODE1 + ch] = 0; // If problem with electrode, turn it off
		}
		
		// Update electrode index
		if (++ch >= N_electrodes){
			ch = 0;	
		}
			
			*/	
    }
}
