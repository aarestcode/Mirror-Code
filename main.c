/*
 * FLIGHT_RM_V2.c
 *
 * Created: 6/14/2016 2:48:57 PM
 * Author : Thibaud
 */ 

#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)

#include "Memory.h"
#include "Interfaces.h"
#include "Drivers.h"
#include "Algorithms.h"

ISR(WDT_vect){
	// Interrupt before power-up from watchdog
	// Saves the register
	SaveRegister(0);
}

int main(void)
{	
	LoadRegister(0);

	UART0_INIT(9600);
	UART1_INIT(9600);
    SPI_INIT(2000000);
	I2C_INIT(200000);
	ADC_INIT(100000);
	
	COMMUNICATION_INIT(1000);
	POWER_INIT();
    //PICOMOTORS_INIT();
	MULTIPLEXER_INIT(0);
	MULTIPLEXER_INIT(1);
	//SEP_DEV_INIT();
	//TEMP_SENSORS_INIT(0);
	
	//PICOMOTOR_ESTIMATION_INIT(100);
	//ELECTRODE_ACTUATION_INIT();
	
	int ch = 0;
	int port;
	
    while (1)
    {	
		// Receive telecommand (if any)
		if ( (port = IsCommandWaiting()) ){	
			ParseCommand(port);
		}
		
		//Actuation loop
		if ( Actuation_ON ){
			// Actuate the electrode
			ActuateElectrode(ch);
			
			// Update electrode index
			if (++ch >= N_electrodes) ch = 0;
		}
				
			
    }
}