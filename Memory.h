/*
 * Memory.h
 
 * SAVE TO/LOAD FROM NON-VOLTAILE MEMORY
 *
 * Created: 6/17/2016 6:36:48 PM
 *  Author: Thibaud
 */ 

/*
This header files regroups everything related to saving variables. 
During a power cycle, the RAM is reset and clears many data that we want to keep.
To avoid that, we save these data in the EEPROM (non-volatile memory). 

However, EEPROMs have a limited number of writing operation allowed. 
So we do not save a variable every time it changes.
Instead, all the variables are saved in the RAM and sometimes, they are updated in the EEPROM. (by a scheduled event or by a command from the Camera)
*/

#ifndef MEMORY_H_
#define MEMORY_H_

#include <avr/eeprom.h> // To save variables to non-volatile memory

#ifndef OK
#define OK 0
#endif

/*--------------------------------------------------
                     REGISTER 
--------------------------------------------------*/
// ADDRESSES (Code for all the variables to save)
enum memory_enum
{
	memory_PRIVATE,
	
	/* ------------------ CODE ------------------- */
	memory_BOOT_SAFE,
	memory_EEPROM_CODE_ADDR,	  // W/R
	
	/* --------------- INTERFACES ---------------- */
	memory_UART0_BAUD,           // R
	memory_UART0_TX,             // R
	memory_UART0_RX,             // R
	memory_UART0_INDEX,			 // R
	
	memory_UART1_BAUD,           // R
	memory_UART1_TX,             // R
	memory_UART1_RX,             // R
	memory_UART1_INDEX,			 // R
	
	memory_SPI_FREQ,              // R
	memory_SPI_TX,                // R
	
	memory_I2C_FREQ,              // R
	memory_I2C_MAX_ITER,          // W/R
	memory_I2C_ITER,              // R
	memory_I2C_SLA,               // R
	memory_I2C_TX,                // R
	memory_I2C_RX,                // R
	
	memory_ADC_FREQ,              // R
	memory_ADC_RX,				  // R
	
	/* ---------------- DRIVERS ----------------- */
	memory_MESSAGE_COUNT0,         // R
	memory_MESSAGE_COUNT1,         // R
	
	memory_FEEDBACK_COUNT0,        // R
	memory_FEEDBACK_COUNT1,        // R
	
	memory_COMMUNICATION_TIMEOUT,  // R
	
	memory_HV,                     // R
	memory_GND,                    // R
	memory_HV_TOL_V,               // W/R 
	memory_HV_STEP,                // R 
	memory_HV_BIAS,                // R 
	memory_PICOMOTOR_V_FB,         // R    
	memory_HV_BIAS_FB,             // R    
	memory_HV_VOLT_FB,             // R 
	memory_BUS_CURR_FB,            // R   
	memory_BUS_VOLT_FB,            // R  
	memory_SEP_VOLT_FB,            // R 
	
	memory_ENCODER_SELECT,        // R
	memory_ENCODER0_STATE,        // R
	memory_ENCODER1_STATE,        // R
	memory_ENCODER2_STATE,        // R
	
	memory_PICO0_TICKS,           // W/R
	memory_PICO1_TICKS,           // W/R
	memory_PICO2_TICKS,           // W/R
	
	memory_MUX_ACTIVE_CH,         // R
	
	memory_TEMP_MCP9801_1,		  // R
	memory_TEMP_PCT2075_1,		  // R
	memory_TEMP_TMP006_1,         // R
	memory_TEMP_TMP006_2,         // R
	memory_TEMP_TMP006_3,         // R
	
	memory_EEPROM_CODE_LENGTH,    // R
	memory_EEPROM_CODE_DWORD,     // R
		
	/* --------------- ALGORITHMS ---------------- */
	memory_PICO_MAX_TICKS_COUNT,  // W/R
	
	memory_PICO0_LOCATION,        // W/R     //NOT IMPLEMENTED
	memory_PICO1_LOCATION,        // W/R     //NOT IMPLEMENTED
	memory_PICO2_LOCATION,        // W/R     //NOT IMPLEMENTED
	
	memory_ENCODER0_INTERVAL_SIZE,// W/R     //NOT IMPLEMENTED
	memory_ENCODER1_INTERVAL_SIZE,// W/R     //NOT IMPLEMENTED
	memory_ENCODER2_INTERVAL_SIZE,// W/R     //NOT IMPLEMENTED
	
	memory_PICO0_MEAN,            // W/R     //NOT IMPLEMENTED
	memory_PICO1_MEAN,            // W/R     //NOT IMPLEMENTED
	memory_PICO2_MEAN,            // W/R     //NOT IMPLEMENTED
	
	memory_PICO0_STD,             // W/R     //NOT IMPLEMENTED
	memory_PICO1_STD,             // W/R     //NOT IMPLEMENTED
	memory_PICO2_STD,             // W/R     //NOT IMPLEMENTED
	
	memory_HV_TIMER,              // W/R
	memory_ELECTRODE_LIMIT_V,     // W/R
	
	memory_ELECTRODE1,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE2,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE3,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE4,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE5,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE6,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE7,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE8,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE9,            // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE10,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE11,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE12,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE13,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE14,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE15,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE16,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE17,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE18,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE19,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE20,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE21,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE22,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE23,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE24,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE25,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE26,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE27,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE28,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE29,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE30,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE31,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE32,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE33,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE34,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE35,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE36,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE37,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE38,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE39,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE40,           // W/R     //NOT IMPLEMENTED
	memory_ELECTRODE41,           // W/R     //NOT IMPLEMENTED
	
	/* --------------- SCHEDULER ---------------- */
	memory_ELECTRODE_ERROR,		  // W/R
	memory_CURRENT_FUNCTION,	  // R
	

	memoryCOUNT //To count the number of variables to memorize
};

// VECTORS DECLARATION
int32_t REGISTER[memoryCOUNT]; //Register vector in the RAM

// PARAMETERS
#define INT_EEPROM_MAX_ADDR 4096

// ENUM
enum int_eeprom{
	SAVE_REGISTER_CODE = 1,
	SAVE_REGISTER_VALUE_CODE,
	LOAD_REGISTER_CODE,
	LOAD_REGISTER_VALUE_CODE,
	
	INT_EEPROM_OVERLOAD
	};

// FUNCTIONS
int SaveRegister(uint16_t eeprom_register)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SAVE_REGISTER_CODE;

	if(eeprom_register*memoryCOUNT*4 + memoryCOUNT*4 - 1 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Update the EEPROM memory with the current RAM memory */
	eeprom_update_block((const void*)REGISTER, (void*)(eeprom_register*memoryCOUNT*4), memoryCOUNT*4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}
int SaveRegisterValue(uint16_t eeprom_register, uint16_t memory_ID){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | SAVE_REGISTER_VALUE_CODE;

	if(eeprom_register*memoryCOUNT*4 + memory_ID*4 + 3 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Update the EEPROM memory with the current RAM memory */
	eeprom_update_block((const void*)&REGISTER[memory_ID], (void*)(eeprom_register*memoryCOUNT*4 + memory_ID*4), 4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}
int LoadRegister(uint16_t eeprom_register)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | LOAD_REGISTER_CODE;

	if(eeprom_register*memoryCOUNT*4 + memoryCOUNT*4 - 1 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Load the EEPROM memory to the RAM memory */
	eeprom_read_block((void*)REGISTER, (const void*)(eeprom_register*memoryCOUNT*4), memoryCOUNT*4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}
int LoadRegisterValue(uint16_t eeprom_register, uint16_t memory_ID)
{
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION] << 8) | LOAD_REGISTER_VALUE_CODE;

	if(eeprom_register*memoryCOUNT*4 + memory_ID*4 + 3 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Load the EEPROM memory to the RAM memory */
	eeprom_read_block((void*)&REGISTER[memory_ID], (const void*)(eeprom_register*memoryCOUNT*4 + memory_ID*4), 4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}
#endif /* MEMORY_H_ */