/**
AVR Watch dog test
Hardware: ATMega8 running at 8MHz
*/

/* Standard libraries */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h> // To use printf

/* AVR libraries */
#include <avr/io.h> //General I/O
#include <avr/interrupt.h> // Interrupt use to receive data from UART
#include <avr/wdt.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#define PD1 0x01
#define PD2 0x02

/* Task Related */
#define MAX_TASKS (10)

#define RUNNABLE (0x00)
#define RUNNING  (0x01)
#define STOPPED  (0x02)
#define ERROR    (0x03)

// Design a task "type": pointer to a void function with no arguments 
typedef void (*task_t)(void);

// basic task control block (TCB)
typedef struct __tcb_t
{
    uint8_t id; // task ID
    task_t task; // pointer to the task
    // delay before execution
    uint16_t delay, period; 
    uint8_t status; // status of task
} tcb_t;


// scheduler functions
void initScheduler(void);
void addTask(uint8_t, task_t, uint16_t);
void deleteTask(uint8_t);
uint8_t getTaskStatus(uint8_t);
void dispatchTasks(void);

// prototypes of tasks
void Task1(void);
void Task2(void);

// the task list
tcb_t task_list[MAX_TASKS];

// keeps track of number of timer interrupts
uint16_t count = 0;

// scheduler function definitions

// initialises the task list
void initScheduler(void)
{
    for(uint8_t i=0; i<MAX_TASKS; i++)
    {
        task_list[i].id = 0;
        task_list[i].task = (task_t)0x00;
        task_list[i].delay = 0;
        task_list[i].period = 0;
        task_list[i].status = STOPPED;
    }
}

// adds a new task to the task list
// scans through the list and
// places the new task data where
// it finds free space
void addTask(uint8_t id, task_t task,
             uint16_t period)
{
    uint8_t idx = 0, done = 0x00;   
    while( idx < MAX_TASKS )
    {
        if( task_list[idx].status == STOPPED )
        {
            task_list[idx].id = id;
            task_list[idx].task = task;
            task_list[idx].delay = period;
            task_list[idx].period = period;
            task_list[idx].status = RUNNABLE;           
            done = 0x01;
        }
        if( done ) break;
        idx++;
    }

}

// remove task from task list
// note STOPPED is equivalent
// to removing a task
void deleteTask(uint8_t id)
{   
    for(uint8_t i=0;i<MAX_TASKS;i++)
    {
        if( task_list[i].id == id )
        {
            task_list[i].status = STOPPED;
            break;
        }
    }
}

// gets the task status
// returns ERROR if id is invalid
uint8_t getTaskStatus(uint8_t id)
{
    for(uint8_t i=0;i<MAX_TASKS;i++)
    {
        if( task_list[i].id == id )
            return task_list[i].status;
    }
    return ERROR;
}

// dispatches tasks when they are ready to run
void dispatchTasks(void)
{
    for(uint8_t i=0;i<MAX_TASKS;i++)
    {
        // check for a valid task ready to run
        if( !task_list[i].delay &&
             task_list[i].status == RUNNABLE )
        {
            // task is now running
            task_list[i].status = RUNNING;           
            // call the task
            (*task_list[i].task)();

            // reset the delay
            task_list[i].delay =
                task_list[i].period;
            // task is runnable again
            task_list[i].status = RUNNABLE;
        }
    }   
}   

// generates a "tick"
// each tick 100ms apart
ISR(TIMER0_OVF_vect)
{
    count ++;
    if( count == 392 )
    {
        count = 0;
        // cycle through available tasks
        for(uint8_t i=0;i<MAX_TASKS;i++)
        {               
            if( task_list[i].status == RUNNABLE )
                task_list[i].delay--;
        }
    }
}

// Task definitions

void Task1(void)
{
    static uint8_t status = 0x01;
    if( status )
        PORTD |= _BV(PD0);
    else
        PORTD &= ~_BV(PD0);
    status = !status;
}


void Task2(void)
{
    static uint8_t status = 0x01;
    if( status )
        PORTD |= _BV(PD1);
    else
        PORTD &= ~_BV(PD1);
    status = !status;
}

// main function
int main(void)
{
    // use 1/8 of system clock frequency
    TCCR0 = 0x02;
    // inital timer value = 0
    TCNT0 = 0;   
    // enable Timer0 Overflow interrupt
    TIMSK = _BV(TOIE0);
    // set PORTD bit0 and bit1 as outputs
    DDRD = _BV(PD0)|_BV(PD1); 

    // set up the task list
    initScheduler();

    // add tasks, id is arbitrary
    // task1 runs every 1 second
    addTask(1, Task1, 10);

    // task2 runs every 4 seconds
    addTask(2, Task2, 40);   

    // enable all interrupts
    sei();   
    for(;;)
        dispatchTasks();
    return 0; // will never reach here
}