/*
 * ScheduluerTestDec16.c
 *
 *	Contains core part of scheduler
 *  Definition of task prototype
 *
 * Created: 12/23/2016 2:27:30 PM
 * Author : WYC
 */ 

#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)

#include "Memory.h"
#include "Interfaces.h"
#include "Drivers.h"
#include "Algorithms.h"

 /* Headers for scheduler*/
const unsigned int tasksNum = 5; //no. of total tasks running on mirror
const unsigned int STOPPED = 0;
const unsigned int RUNNABLE = 1;
const unsigned int LOW = 0; // Priority
const unsigned int HIGH = 1;

/* define task type */
 typedef struct mirrorBoxTask { // Q: can we make definitions while doing the typedef?
   unsigned long TaskSequence; // Sequence of task, only used to identify the task number when updating; unchanged. Starts from 1.
   unsigned long Priority;  // Period of task, this is the initial value of task's position in task list, updatable through xbee command. Takes two values: LOW(0) or HIGH(1). HIGH priority will always be RUNNABLE. 
   bool Status; // Status of task: STOPPED(0) or RUNNABLE(1)
   signed int ActionData; // data stored in struct, ready to be used. Default value is 0. For tasks with no input it's also 0.
   int (*ActFct)(signed int actionData, int port);     // Function to call for task's action. Some of the task functions will take input (actiondata)
}mirrorBoxTask;

// initialize task structs, global variable
mirrorBoxTask mirrorBoxTasks[5]; // need volatile? Tasks number starts from 1. 0 is reserved for ParseCommand.

/* define action function prototype */
int ActFct_Print1(signed int actionData, int port); // Q: can we use same name here? (int port, int taskid, int taskpriority, int taskstatus, long data)
int ActFct_Print2(signed int actionData, int port);
int ActFct_Print3(signed int actionData, int port);
int ActFct_Print4(signed int actionData, int port);
int ActFct_Print5(signed int actionData, int port);

/* parse command function, run at the beginning of every scheduling loop */
int ParseCommand(void){ // parse from command: task sequence, priority, status and action data.
	
	// TaskID, 1 byte
	unsigned int taskId = 0;
	taskId = Message[TaskIDN - 1];
	
	// PriorityID, 1 byte
	unsigned int priority = 0;
	priority = Message[TaskIDN + PriorityN - 1];
	
	// Status, 1 byte
	unsigned int status = 0;
	status = Message[TaskIDN + PriorityN + StatusN - 1];
	
	// Command value, 4 byte
	signed int data = 0;
	for (int II = 0; II < MessageDataN; II++) data |= ((int32_t)Message[TaskIDN + PriorityN + StatusN + II] << 8*(MessageDataN - II - 1));
	
	// Load the task attributes
	mirrorBoxTasks[taskId].TaskSequence = taskId;
	mirrorBoxTasks[taskId].Priority = priority;
	mirrorBoxTasks[taskId].Status = status;
	mirrorBoxTasks[taskId].ActionData = data;
	
	return OK;
	
}

/* main function*/
int main(void)
{
	/* initialize drivers*/
	if(USART0_INIT(9600)) return 1;
	if(USART1_INIT(9600)) return 1;
    if(SPI_INIT(4000000)) return 1;
	if(I2C_INIT(200000)) return 1;
	
	
	if(COMMUNICATION_INIT()) return 2;
    if(PICOMOTORS_INIT()) return 2;
	if(SEP_DEV_INIT()) return 2;
	if(TEMP_SENSORS_INIT()) return 2;
	
	if(PICOMOTOR_ESTIMATION_INIT()) return 3;
	
    /* Initialization of tasks */
	int i = 0;
	// Task 1
	mirrorBoxTasks[i].TaskSequence = 0;
	mirrorBoxTasks[i].Priority = HIGH;
	mirrorBoxTasks[i].Status = RUNNABLE;
	mirrorBoxTasks[i].ActionData = 0;
	mirrorBoxTasks[i].ActFct = &ActFct_Print1;
	
	//
	++i;
	mirrorBoxTasks[i].TaskSequence = 0;
	mirrorBoxTasks[i].Priority = HIGH;
	mirrorBoxTasks[i].Status = RUNNABLE;
	mirrorBoxTasks[i].ActionData = 0;
	mirrorBoxTasks[i].ActFct = &ActFct_Print2;
	
	//
	++i;
	mirrorBoxTasks[i].TaskSequence = 0;
	mirrorBoxTasks[i].Priority = HIGH;
	mirrorBoxTasks[i].Status = RUNNABLE;
	mirrorBoxTasks[i].ActionData = 0;
	mirrorBoxTasks[i].ActFct = &ActFct_Print3;
	
	//
	++i;
	mirrorBoxTasks[i].TaskSequence = 0;
	mirrorBoxTasks[i].Priority = HIGH;
	mirrorBoxTasks[i].Status = RUNNABLE;
	mirrorBoxTasks[i].ActionData = 0;
	mirrorBoxTasks[i].ActFct = &ActFct_Print4;

	//
	++i;
	mirrorBoxTasks[i].TaskSequence = 0;
	mirrorBoxTasks[i].Priority = HIGH;
	mirrorBoxTasks[i].Status = RUNNABLE;
	mirrorBoxTasks[i].ActionData = 0;
	mirrorBoxTasks[i].ActFct = &ActFct_Print5;
		
	/*
	 * Heart of scheduler code
	 * Loop over the task list. Properties:
	 * 1. Can only update one task at a time and must defined to be run to completion, i.e. does not wait on event, block or wait for infinite loop. Then the scheduluer will wait for the whole list (task stack) to finish
	 * 2. About time: at one time multiple tasks can run (in sequence). For each task there's no fixed interval or fixed runtime. The runtime should only be smaller than the longest watchdog overflow.
	 * 3. On top of list is parse cmmd. It'll always be runnable; each time it'll only update one task's attributes.
	 * 4. Non-preemptive, meaning: no interrupt to occur during task running . This is driven by the non-realtime requirement.
	 * 5. Tasks that are always active:
		Write register;
		Save temperature;
		
	 * 6. Tasks could only be awaken:
		Ping;
		Move picomotor;
		Actuate mirror;
		
	 * 7. Tasks exit mechanism: 
	 
	 * One concern is that incoming message will stack at the buffer of xbee chip. We assume xbee can handle the buffer before fetched by the mcu. Otherwise: 1) camera scheduler will wait for certain time before sending the next cmmd 2) camera doesnt send cmmd until receives confirmation
	 */
	unsigned int II;
	int port;
	int error;
	//bool taskStatus;
	
     while(1){ // loop forever the list
	 for (II = 0; II < tasksNum; ++II) { //loop over all the tasks
			
		if (II == 0){ // Before looping, fetch income command. Note: high priority cmmds should take no input.
			
			if((port=IsCommandWaiting())){ // fetch the port we're using 
				SaveCommand(port);
				ParseCommand(); // Q: do we need port? Do we need error?
				}
			} 
		
		else if (mirrorBoxTasks[II].Status == RUNNABLE) { // head of the task list
			
			if(!mirrorBoxTasks[II].Priority){ // Low priority
            error = mirrorBoxTasks[II].ActFct(mirrorBoxTasks[II].ActionData, 2); //execute task action, wait for task to exit normally
			
			SendFeedback(2, mirrorBoxTasks[II].TaskSequence, mirrorBoxTasks[II].Priority, mirrorBoxTasks[II].Status, mirrorBoxTasks[II].ActionData);// Then send back task action function status, include the original data the local mirror box received.
			
			mirrorBoxTasks[II].Status = STOPPED; // Reset status for low priority tasks
				}
				
			else { // High priority tasks
				error = mirrorBoxTasks[II].ActFct(NullCommand, 2); // High priority tasks should take no input
				
				SendFeedback(2, mirrorBoxTasks[II].TaskSequence, mirrorBoxTasks[II].Priority , mirrorBoxTasks[II].Status, NullCommand); // Send back a null command
				}
			}		
		}	
	 }
}

/* define task action functions */
// These are test functions: send back the data they receive. If not receiving data, send back
int ActFct_Print1(signed int actionData, int port){_delay_ms(1000);SendFeedback(port, 1, HIGH, RUNNABLE, actionData); return OK;}; // Q: can we use same name here? (int port, int taskid, int taskpriority, int taskstatus, long data)
int ActFct_Print2(signed int actionData, int port){_delay_ms(1000);SendFeedback(port, 2, HIGH, RUNNABLE, actionData); return OK;};
int ActFct_Print3(signed int actionData, int port){_delay_ms(1000);SendFeedback(port, 3, HIGH, RUNNABLE, actionData); return OK;};
int ActFct_Print4(signed int actionData, int port){_delay_ms(1000);SendFeedback(port, 4, HIGH, RUNNABLE, actionData); return OK;};
int ActFct_Print5(signed int actionData, int port){_delay_ms(1000);SendFeedback(port, 5, HIGH, RUNNABLE, actionData); return OK;};