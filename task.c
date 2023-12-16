#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>  // Assuming you have the TM4C123GH6PM header file

void delay(uint32_t count){
    for (uint32_t i = 0; i < count; i++);
}

// Define macros for GPIO Port F
#define GPIOF_BASE_ADDR 0x40025000

// GPIO Port F registers
volatile uint32_t * const RCGCGPIO = (uint32_t *)(GPIOF_BASE_ADDR + 0x608);
volatile uint32_t * const GPIOF_DIR = (uint32_t *)(GPIOF_BASE_ADDR + 0x400);
volatile uint32_t * const GPIOF_DEN = (uint32_t *)(GPIOF_BASE_ADDR + 0x51C);
volatile uint32_t * const GPIOF_DATA = (uint32_t *)(GPIOF_BASE_ADDR + 0x000);

// Function to turn LED on
void led_on(uint8_t pin){
    GPIOF->DATA |= (1<<3);
}
	
// Function to turn LED off
void led_off(uint8_t pin){
    GPIOF->DATA &= ~(1<<3);
}

// Function to initialize LED GPIO
void led_init_all(void){
    SYSCTL->RCGCGPIO |= (1 << 5);  // Enable clock for GPIO Port F
    GPIOF->DIR |= (1 << 3) | (1 << 2) | (1 << 1);  // Set pins as output
    GPIOF->DEN |= (1 << 3) | (1 << 2) | (1 << 1);  // Enable digital function
    GPIOF->DATA|=(1 << 3);
    GPIOF->DATA |=((1 << 1) | (1 << 2));
    GPIOF->DATA &= ~ 1<<1;
    GPIOF->DATA &= ~ 1<<2;
}


#define MAX_TASKS 5

void task1(void);
void task2(void);
void task3(void);
void task4(void);

void init_systick_timer(uint32_t tick_hz);
__attribute((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);
void init_tasks_stack(void);
void enable_processor_faults(void);
__attribute((naked)) void switch_sp_to_psp(void);
uint32_t get_psp_value(void);
void save_psp_value(uint32_t current_psp_value);
void update_next_task(void);
__attribute((naked)) void PendSV_Handler(void);
void schedule(void);
void task_delay(uint32_t tick_count);

// Stack memory calculations
#define SIZE_TASK_STACK      1024U
#define SIZE_SCHED_STACK     1024U
#define SRAM_START           0x20000000U
#define SRAM_SIZE            ((32) * (1024))
#define SRAM_END             (SRAM_START + SRAM_SIZE)
#define T1_STACK_START       (SRAM_END)
#define T2_STACK_START       (SRAM_END - (1 * SIZE_TASK_STACK))
#define T3_STACK_START       (SRAM_END - (2 * SIZE_TASK_STACK))
#define T4_STACK_START       (SRAM_END - (3 * SIZE_TASK_STACK))
#define IDLE_STACK_START     (SRAM_END - (4 * SIZE_TASK_STACK))
#define SCHED_STACK_START    (SRAM_END - (5 * SIZE_TASK_STACK))
#define TICK_HZ              1000U
#define HSI_CLOCK            16000000U
#define DUMMY_XPSR           0x00100000U
#define TASK_READY_STATE     0x00
#define TASK_BLOCKED_STATE   0xFF
#define INTERRUPT_DISABLE()  do {__asm volatile ("CPSID I");} while(0);
#define INTERRUPT_ENABLE()   do {__asm volatile ("CPSIE I");} while(0);

uint8_t current_task = 1; // Task 1 is running
uint32_t g_tick_count = 0;

typedef struct {
    uint32_t psp_value;
    uint32_t block_count;
    uint8_t current_state;
    void (*task_handler)(void);
} TCB_t;

TCB_t user_tasks[MAX_TASKS];

int main(void){
    enable_processor_faults();

    init_scheduler_stack(SCHED_STACK_START);

    init_tasks_stack();

    led_init_all();

    init_systick_timer(TICK_HZ);

    switch_sp_to_psp();

    user_tasks[current_task].task_handler();  // Start the first task

    for(;;);
}

// Implement the fault handlers
void HardFault_Handler(void){
    // Handle Hard Fault
    while(1);
}

void MemManage_Handler(void){
    // Handle Memory Management Fault
    while(1);
}

void BusFault_Handler(void){
    // Handle Bus Fault
    while(1);
}

void idle_task(void){
    while(1){
        // Idle task
    }
}

void task1_handler(void){
 //   while(1){
        GPIOF->DATA &= ~(1<<3);
        delay(100);  // Assuming delay is in microseconds
       // GPIOF->DATA &= ~(1<<3);
       // delay(100);
 //   }
}

void task2_handler(void){
    //while(1){
        GPIOF->DATA &= ~ (((1<<1)|(1<<2)));
        //delay(100);  // Assuming delay is in microseconds
       // GPIOF->DATA &= ~(((1<<1)|(1<<2)));
        //delay(100);
   // }
}

void task3_handler(void){
    //while(1){
        GPIOF->DATA &= ~(1<<2);
        //delay(100);  // Assuming delay is in microseconds
        //GPIOF->DATA &= ~(1<<2);
        //delay(100);
   // }
}

void task4_handler(void){
   // while(1){
       // GPIOF->DATA|=(1<<1);
       // delay(100);  // Assuming delay is in microseconds
       // GPIOF->DATA &= ~(1<<1);
       // delay(100);
    //}
}

void init_systick_timer(uint32_t tick_hz){
	  
    SysTick->CTRL = 0;                            // Disable SysTick during setup
    SysTick->LOAD = HSI_CLOCK / tick_hz - 1;      // Set the reload value for the desired tick frequency
    SysTick->VAL = 0;                             // Clear the current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;     // Enable SysTick with the processor clock
}

__attribute((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack){
    __asm volatile(
        "MSR MSP, R0\n"   // Move the value of R0 (function argument) to MSP
        "BX LR\n"         // Return from the function (Link Register)
    );
}


void init_tasks_stack(void){
    uint32_t *pPSP;
		// initially all tasks in Running State
		user_tasks[0].current_state =TASK_READY_STATE;// task 0 becomes the idle task which will always be in running state
		user_tasks[1].current_state =TASK_READY_STATE;
		user_tasks[2].current_state =TASK_READY_STATE;
		user_tasks[3].current_state =TASK_READY_STATE;
		user_tasks[4].current_state =TASK_READY_STATE;

		user_tasks[0].psp_value = IDLE_STACK_START;
		user_tasks[1].psp_value = T1_STACK_START;
		user_tasks[2].psp_value = T2_STACK_START;
		user_tasks[3].psp_value = T3_STACK_START;
		user_tasks[4].psp_value = T4_STACK_START;

		user_tasks[0].task_handler = idle_task;
		user_tasks[1].task_handler = task1_handler;
		user_tasks[2].task_handler = task2_handler;
		user_tasks[3].task_handler = task3_handler;
		user_tasks[4].task_handler = task4_handler;
	
    for(int i=0;i<MAX_TASKS;i++){
        pPSP =(uint32_t*) user_tasks[i].psp_value;
        
        pPSP--;
        *pPSP = DUMMY_XPSR; //0x00100000

        pPSP--; //PC
        *pPSP = (uint32_t)user_tasks[i].task_handler; 

        pPSP--; //LR
        *pPSP = 0xFFFFFFFD; 

        for (int j=0;j<13;j++){
            pPSP--;
            *pPSP = 0; 
        }

        user_tasks[i].psp_value=(uint32_t)pPSP;
    }
}

void enable_processor_faults(void){
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;
}

uint32_t get_psp_value(void){
    return user_tasks[current_task].psp_value;
}

void save_psp_value(uint32_t current_psp_value){
    user_tasks[current_task].psp_value = current_psp_value;
}

void update_next_task(void){
    int state = TASK_BLOCKED_STATE;

    for (int i = 0; i < MAX_TASKS; i++){
        current_task++;
        current_task %= MAX_TASKS;
        state = user_tasks[current_task].current_state;
        if ((state == TASK_READY_STATE) && (current_task != 0))
            break;
    }
}

__attribute((naked)) void switch_sp_to_psp(void){
    __asm volatile("PUSH {LR}");  // Preserve LR which connects back to main()
    __asm volatile("BL get_psp_value");
    __asm volatile("MSR PSP, R0");  // Initialize PSP
    __asm volatile("POP {LR}");  // Pops back LR value

    // Change SP to PSP using CONTROL register
    __asm volatile("MOV R0, #0x02");
    __asm volatile("MSR CONTROL, R0");
    __asm volatile("BX LR");  // Back to main()
}

void schedule(void){
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;  // Pend the PendSV exception
}

void task_delay(uint32_t tick_count){
    // Disable interrupt
    INTERRUPT_DISABLE();

    if (current_task){
        user_tasks[current_task].block_count = g_tick_count + tick_count;
        user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
        schedule();
    }

    // Enable interrupt
    INTERRUPT_ENABLE();
}

__attribute((naked)) void PendSV_Handler(void){
    // Save the context of the current task
    __asm volatile("MRS R0, PSP");  // Get the current running task's PSP value
    __asm volatile("STMDB R0!, {R4-R11}");  // Using that PSP value, store SF2 (R4 to R11)
    __asm volatile("PUSH {LR}");  // Save the current LR value
    __asm volatile("BL save_psp_value");  // Save the current value of PSP

    // Retrieve the context of the next task
    __asm volatile("BL update_next_task");  // Decide the next task to run
    __asm volatile("BL get_psp_value");  // Get its past PSP value
    __asm volatile("LDMIA R0!, {R4-R11}");  // Using that PSP value, retrieve SF2 (R4 to R11)
    __asm volatile("MSR PSP, R0");  // Update PSP and exit
    __asm volatile("POP {LR}");  // Pop back LR value
    __asm volatile("BX LR");  // Return from exception
}

void update_global_tick_count(void){
    g_tick_count++;
}

void unblock_tasks(void){
    for (int i = 0; i < MAX_TASKS; i++){
        if (user_tasks[i].current_state != TASK_READY_STATE){
            if (user_tasks[i].block_count == g_tick_count){
                user_tasks[i].current_state = TASK_READY_STATE;
            }
        }
    }
}

void SysTick_Handler(void){
    update_global_tick_count();
    unblock_tasks();
    schedule();
}