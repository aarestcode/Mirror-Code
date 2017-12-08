/*
 * SchedulerTestJune28.c
 *
 * Created: 6/28/2017 3:28:25 PM
 * Author : WYC
 */ 

/**
AVR Watch dog test
Hardware: ATMega8 running at 8MHz
*/
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT), Have to define here.

#include "Memory.h"
#include "Interfaces.h"
#include "Drivers.h"
#include "Algorithms.h"

// Task Related 
#define MAX_TASKS (0x10)
#define RUNNABLE (0x00)
#define RUNNING  (0x01)
#define STOPPED  (0x02)
#define ERROR    (0x03)
#define REPEATABLE (0x01) // If the task can be run repeatedly

// ActionCode
#define Add_Task (0x00)
#define Delete_Task (0x01)
#define Get_Status (0x02)

/**********************************************************************/
// Definition of data structure, function prototypes
/**********************************************************************/
// Design a task "type": each task is a function. Pointer to a void function with no arguments
typedef uint8_t (*task_t)(uint32_t);

// basic task control block (TCB)
typedef struct __tcb_t
{
	uint8_t id; // task ID
	task_t task; // pointer to the task function
	// delay before execution
	uint16_t delay, period;
	uint8_t status; // status of task, i.e. health of task. Need to be updated at end of each execution.
	uint32_t data; // data to be transfered to task function
} tcb_t;

// scheduler control functions
void initScheduler(void);
void addTask(uint8_t, task_t, uint8_t, uint32_t);
void deleteTask(uint8_t);
int getTaskStatus(uint8_t);
void dispatchTasks(void);

// prototypes of tasks
uint8_t Task151(uint32_t);

// Initialize the task list: data struct is shared globally
tcb_t task_list[MAX_TASKS];

// Reset number of timer interrupts every start
uint16_t count = 0;

/**********************************************************************/
// Definition of functions used in scheduler
/**********************************************************************/
// Initializes the task list to the max number of tasks
void initScheduler(void)
{
	for(uint8_t i=0; i<MAX_TASKS; i++)
	{
		task_list[i].id = 0; // Sending id 0 is illegal.
		task_list[i].task = (task_t)0x00; // empty function pointer
		task_list[i].delay = 0;
		task_list[i].period = 0;
		task_list[i].status = STOPPED;
		task_list[i].data = 0x00;
	}
}

// Adds a new task to the task list
// scans through the list and places the new task data where it finds free space
void addTask(uint8_t id, task_t task,  uint8_t period, uint32_t data)
{
	uint8_t idx = 0, done = 0x00;
	
	// First check if the task already exists and runnable; if so return. This is to prevent adding redundent task.
	for (idx=0; idx < MAX_TASKS; idx++)
	{
		if(task_list[idx].id == id && (task_list[idx].status == RUNNABLE || task_list[idx].status == RUNNING)) return; // If the task id exist but status is stopped, then this task can be overwritten
	}
	
	idx = 0;
	
	// As long as one task is stopped, it means its associated task in task list can be over written.
	while( idx < MAX_TASKS )
	{
				
		if( task_list[idx].status == STOPPED )
		{
			task_list[idx].id = id;
			task_list[idx].task = task; // Assign the task function pointer
			task_list[idx].delay = period;
			task_list[idx].period = period;
			task_list[idx].status = RUNNABLE;
			task_list[idx].data = data;
			done = 0x01;
		}
		
		if( done ) break; // adding has finished, found a blank spot, exit here.
		idx++; // else check the next available spot in task list
	}

}

// remove task from task list. Stops the task immediately.
// note STOPPED is equivalent to removing a task
void deleteTask(uint8_t id)
{
	// Prevent 
	
	for(uint8_t i=0;i<MAX_TASKS;i++)
	{
		if( task_list[i].id == id )
		{
			task_list[i].status = STOPPED;	// So this idx can be used as plugging a task.		
			break;
		}
	}
}

// gets the task status
int getTaskStatus(uint8_t id) // command is id
{
	for(uint8_t i=0;i<MAX_TASKS;i++)
	{
		if( task_list[i].id == id )
		return task_list[i].status;	
	}
	// Send feedback in ParseCommand_New
	// return ERROR; // if id is invalid
}

// Generates a "tick" with system timer
// each tick 100ms (0.1s) apart
// Will be disturbed by: 
// 1) UART interrupt; but won't be disturbed by watchdog timer (run in parallel independently)
ISR(TIMER0_OVF_vect)
{
	count ++;
	if( count == 392 )
	{
		count = 0;
		// cycle through available tasks and deduct 1 from delay every 0.1s
		for(uint8_t i=0;i<MAX_TASKS;i++)
		{
			if( (task_list[i].status == RUNNABLE) && (task_list[i].delay>0) ) 
			task_list[i].delay--; // If delay is 0, then task is ready to run
		// Important 
			
		}
	}
}

// Parse command
void ParseCommand_New(int port){ // This function update the task list, !tasks are not run here!; detailed operation done in task functions upon get called (Loc_1).

	uint8_t command;
	uint8_t ActionCode;
	uint8_t Period;
	int32_t data;
	uint16_t checksum;
	uint8_t TaskStatus;
	
	//--------------------------------------------------
	//                 MESSAGE CHECK
	//--------------------------------------------------
	CheckMessage(port, &command, &ActionCode, &Period, &data, &checksum); // in drivers.h, decode the incoming message.
	
	switch(command) { // Command no. is task no. in scheduler
		
		/*--------------------------------------------------
						  Read register
		--------------------------------------------------*/
		case 150:
		// Instead of creating a task for this functionality, simply do it as soon as it received the command, non repeatable.
		SendFeedback(port, command, ActionCode, Period, REGISTER[data]);

		/*--------------------------------------------------
						Blink LED
		--------------------------------------------------*/
		case 151:
		
		// Send command back to confirm data has been successfully parsed
		if(ActionCode == Delete_Task){
			deleteTask(command);
			SendFeedback(port, command, ActionCode, Period, data);
		}
		else if(ActionCode == Add_Task){
			addTask(command, Task151, Period, data);
			SendFeedback(port, command, ActionCode, Period, data);
		} 
		else if(ActionCode == Get_Status){
			TaskStatus = getTaskStatus(command);
			SendFeedback(port, command, ActionCode, Period, TaskStatus); // need to include status of task in feedback message.
		}		
		break;
		
		
		default:
		// wrong command (id invalid), send feedback.
		SendFeedback(port, command, ActionCode, Period, data);
		break;
		
	}

}

// Core of the scheduler: dispatches tasks when they are ready to run.
// Notice: will be constantly interrupted by tick function
// This function runs at system frequency
void dispatchTasks(void)
{
	
	int port; 
	int repeatability;
	if ( (port = IsCommandWaiting()) ){ 
		ParseCommand_New(port);
	} // Note: IsCommandWaiting will disable interrupt every time called. So system timer will be stopped for a tiny amount of time (time required to run the function), happens at system frequency; the system timer will pick up the value where it's stopped then continues running.
	// During long task functions, even if new message comes in, will not be accepted and parsed if code is 'stuck' at Loc_1.
	
	for(uint8_t i=0;i<MAX_TASKS;i++)
	{

		if( !task_list[i].delay &&
		task_list[i].status == RUNNABLE )
		{
			
			task_list[i].status = RUNNING; // Change status to running so ISR wont modify the tasks status until the function finishes.
			
			repeatability = (*task_list[i].task)(task_list[i].data);  // Loc_1
			// !!! Important. Running the task function. This will take as long as it needs. But if this function takes too long to run, nothing will happen. 

			// reset the delay
			task_list[i].delay = task_list[i].period;

			// Depends on the task type, keep the current task runnable or remove it from list
			if (repeatability)
			{
				task_list[i].status = RUNNABLE;
			}
			else
				task_list[i].status = STOPPED;		
			
		}
	}
}

// task function for test
uint8_t Task151(uint32_t data) // not using data for now
{
			
	//LED ON
	PORTD &=~(1<<PORTD7);
	//~0.1s delay
	_delay_ms(500);
	PORTD|=(1<<PORTD7);

	// send feedback to console.

	// This task is repeatable
	return REPEATABLE;

}

/**********************************************************************/
// Main loop
/**********************************************************************/
// main function
int main(void)
{
	cli(); // Disable interrupts
	
	LoadRegister(0);

	UART0_INIT(9600);
	UART1_INIT(9600);
	SPI_INIT(2000000);
	I2C_INIT(200000);
	ADC_INIT(100000);
	
	COMMUNICATION_INIT(1000);
	POWER_INIT();
	PICOMOTORS_INIT();
	MULTIPLEXER_INIT(0);
	MULTIPLEXER_INIT(1);
	SEP_DEV_INIT();
	TEMP_SENSORS_INIT(0);
	// If one of these fail, go into watchdog reset mode
	
	
	/* Perform system initialization, including initialization of WDT */
	// Timer0 control register: use 1/8 of system clock frequency
	TCCR0B = 0x02; 
	// Important: 8-bit counter Counter overflow at 255 counts
	
	// timer initial value -> necessary
	TCNT0 = 0;
	// enable Timer0 Overflow interrupt
	TIMSK0 = _BV(TOIE0);
	// enable Timer0 output compare match interrupt
	//TIMSK0 = _BV(OCIE0A); OCR0A = (1 << 7);
	// set PORTD bit0 and bit1 as outputs to light up LED's
	DDRD = _BV(PORTD7);

	// Initialize scheduler: set up the task list
	initScheduler();

	// task1 runs every 1 second
	addTask(151, Task151, 10, 0);

	// Enable all interrupts. Attention!
	//1. During parsing command interrupt will be disabled
	//2. UART interrupt will disturb the system timer (or not?)
	sei(); // Enable global interrupt
	for(;;)
		dispatchTasks();
	
	return 0; // will never reach here
}

// Doomed if reach here