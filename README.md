# Hands-On FreeRTOS 🖥️
The following is a repository that contains the exploration of Real-Time Operating Systems (RTOS) based on a Udemy course: [Mastering RTOS: Hands on FreeRTOS and STM32Fx with Debugging](https://www.udemy.com/course/mastering-rtos-hands-on-with-freertos-arduino-and-stm32fx/?utm_source=adwords&utm_medium=udemyads&utm_campaign=Webindex_Catchall_la.EN_cc.CA&utm_term=_._ag_119831896715_._ad_533102824920_._kw__._de_c_._dm__._pl__._ti_dsa-21781902600_._li_9000965_._pd__._&matchtype=&gad_source=1&gclid=CjwKCAiArLyuBhA7EiwA-qo80N3YBt89oN_fUM4my-SwgprhOUh4ZP6upBo_oo2izcF-jyFWsJuC1xoCG_QQAvD_BwE)

# Topics 📃
The topics in this repository include:
* Learn Complete Step by step method to run FreeRTOS on STM32 MCUs using OpenSTM32 System Workbench
* Using STM32 Standard Peripheral Driver APIs to configure peripherals
* FreeRTOS Task Creation , Deletion, Scheduling using with code examples
* Important scheduling policies of FreeRTOS Scheduler
* FreeRTOS Stack and Heap Management
* Right ways of Synchronizing between tasks using Semaphores.
* Right ways of Synchronizing between a task and an interrupt using semaphores
* Synchronization between multiple events and a task
* FreeRTOS Queue management like creation, sending, receiving, blocking, etc
* Implementing mutual exclusion between Tasks using Mutex services and semaphores
* Understanding Context Switching with in detail code explanation
* Understanding Architecture specific codes like SVC_handler, PendSV_handler, SysTick Handler line by line
* You will learn about kernel Tick timer, its importance, the kernel tick frequency and its configuration details.
* Understanding complete ARM Cortex M and FreeRTOS Priority model and its configuration related informations
* FreeRTOS Debugging using SEGGER SystemView Software
* Low power instructions use case in FreeRTOS scenario

# Tools 🔨
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* [NUCLEO-F767ZI](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
* [FreeRTOS](https://www.freertos.org/index.html)
* [SEGGER SystemView](https://www.segger.com/products/development-tools/systemview/)

## Pre-emptive vs Cooperative scheduling

<p align="center">
    <img title="Pre-emptive Scheduling" alt="SEGGER SystemView" src="./Recordings/preemptive_recording_image.png" width="1000" height="500">
</p>

<p align="center">
    <i>
    Pre-emptive scheduling captured in the SEGGER SystemView
    </i>
</p>

* Preemptive multitasking forces apps to share the processor, whether they want to or not. In the above case, "Hello World" is printed as many times as possible until the OS switches to the next scheduled task at the scheduling RTOS tick rate (1ms)

<p align="center">
    <img title="Cooperative Scheduling" alt="SEGGER SystemView" src="./Recordings/cooperative_recording_image.png" width="1000" height="500">
</p>
<p align="center">
    <i>
    Cooperative scheduling captured in the SEGGER SystemView
    </i>
</p>

* In cooperative scheduling the OS never interrupts a running process to initiate a context switch from one process to another. Processes must voluntarily yield control periodically or when logically blocked on a resource. In the above scenario, the code was written in such a way that, once "Hello World" was printed, the CPU would be voluntarily yielded by the task.

## LED Task
* Green LED delay 1000ms
* Red LED delay 800ms
* Blue LED delay 400ms
* RTOS tick 100ms

<p align="center">
    <img title="LED Task NON-Blocking Scheduling" alt="SEGGER SystemView" src="./Recordings/ledtask_recording_image.png" width="1000" height="500">
</p>
<p align="center">
    <i>
    LED task NON-blocking scheduling captured in the SEGGER SystemView
    </i>
</p>

* These tasks are known as continuous tasks because despite having nothing to do during some executions they continue to demand resources from the CPU. The CPU was never free and therefore the current draw of the microcontroller was at its highest.

## Blocking Delay API's
* [vTaskDelay()](https://www.freertos.org/a00127.html) - Specifies a wake time relative to the time at which the function is called
* [vTaskDelayUntil()](https://www.freertos.org/vtaskdelayuntil.html) - Specifies the absolute (exact) time at which it wishes to unblock. The parameter in vTaskDelayUntil is the absolute time in ticks at which you want to be woken calculated as an increment from the time you were last woken.

<p align="center">
    <img title="LED Task Blocking Scheduling vTaskDelay()" alt="SEGGER SystemView" src="./Recordings/ledblocking_tasks_recording_image.png" width="1000" height="500">
</p>
<p align="center">
    <i>
    LED task blocking (vTaskDelay()) scheduling captured in the SEGGER SystemView
    </i>
</p>

* In this example the vTaskDelay() function is called allowing the CPU resources to be freed up once the task is completed. Notice how the idle time now takes up 99% of the time. 
* Unfortunately in this example you won't be able to notice the true difference between vTaskDelay() and vTaskDelayUntil(). 

## Example 005LED_TASK NOTIFY
The following example is composed of the below tasks:
1. Green LED Task
    * Toggle Period: 1s
    * Priority: 3
2.  Red LED Task
    * Toggle Period: 800ms
    * Priority: 2    
3.  Blue LED Task
    * Toggle Period: 400ms
    * Priority: 1
4.  User Button 13 Task
    * Polling every 10ms
    * Priority: 4
    * When the user button is pressed the highest priority LED task is run and deleted.

### RTOS API's Used
#### RTOS Notification API's
* [xTaskNotify()](https://www.freertos.org/xTaskNotify.html) - xTaskNotify() is used to send an event directly to and potentially unblock an RTOS task, and optionally update one of the receiving task’s notification values
* [xTaskNotifyWait()](https://www.freertos.org/xTaskNotifyWait.html) -  xTaskNotifyWait() waits, with an optional timeout, for the calling task to receive a notification. If the receiving RTOS task was already Blocked waiting for a notification when the notification it is waiting for arrives the receiving RTOS task will be removed from the Blocked state and the notification cleared.

#### RTOS Suspend API's
* [vTaskSuspendAll()](https://www.freertos.org/a00134.html) - Suspends the scheduler.  Suspending the scheduler prevents a context switch from occurring but leaves interrupts enabled.  If an interrupt requests a context switch while the scheduler is suspended, then the request is held pending and is performed only when the scheduler is resumed (un-suspended).
* [xTaskResumeAll()](https://www.freertos.org/a00135.html) - Resumes the scheduler after it was suspended using a call to vTaskSuspendAll().

#### RTOS Creation API's
* [vTaskDelete](https://www.freertos.org/a00126.html) - Remove a task from the RTOS kernels management. The task being deleted will be removed from all ready, blocked, suspended and event lists.

<p align="center">
    <img title="LED Notification Scheduling" alt="SEGGER SystemView" src="./Recordings/ledtask_notify_recording_image.png" width="1000" height="500">
</p>
<p align="center">
    <i>
    LED notification scheduling captured in the SEGGER SystemView
    </i>
</p>

In the above snapshot...
1. The scheduler tick is running every 10ms
2. The Button_task runs and detects that the user button has changed states (ON)
3. The scheduler calls xTaskNotify(next_task_handle,0,eNoAction) which sends a notification to the task with the task handle next_task_handle
4. The scheduler runs the next available task which is the LED_green_task
5. The green LED is toggled
6. the LED_green_task calls the function xTaskNotifyWait(0,0, NULL, pdMS_TO_TICKS(1000)) and notices that it has received a notification from another task
7. The LED_green_task enters a block of code which turns on the green LED, suspends the scheduler, sets the next_task_handle, resumes the scheduler, and then deletes the current LED_green_task
8. Then the scheduler returns to it's idle state until the next tick

## Example 006_LED_BTN_ISR
The goal of this example is exactly the same as the previous one with one key difference. For the button programming we are going to use a hardware button interrupt handler rather than a scheduled task. 

### RTOS API's Used
* [xTaskNotifyFromISR](https://www.freertos.org/xTaskNotifyFromISR.html) - Versions of xTaskNotify() and xTaskNotifyIndexed() that can be used from an interrupt service routine (ISR)
* [traceISR_ENTER()](https://www.freertos.org/rtos-trace-macros.html) - Helps the SEGGER system viewer trace that we have entered an interrupt
* [portENTER_CRITICAL()](https://www.freertos.org/taskENTER_CRITICAL_taskEXIT_CRITICAL.html) - In this example, this function is used because we are modifying a variable that is shared between tasks and an interrupt. This allows us to disable interrupts before changing the variable.

### Hardware Interrupts vs Kernel Interrupts
* ARM Cortex M7 has 16 priority levels that range from 0x00 - 0xF0. 0x00 is the highest priority and 0xF0 is the lowest priority.
* The macro <i>configMAX_SYSCALL_INTERRUPT_PRIORITY</i> determines the highest level priority value that an interrupt routine using "FromISR" can use.
* Interrupts that do not call API functions can execute at priorities above the macro <i>configMAX_SYSCALL_INTERRUPT_PRIORITY</i> and therefore never be delayed bu the RTOS. 

### Configuring EXTI line interrupt
* <i>006_LED_BTN_ISR.ioc -> System Core -> NVIC -> EXTI 15_10 interrupt -> Preemption Priority </i>
* Pick a priority level that is smaller than the macro <i>configMAX_SYSCALL_INTERRUPT_PRIORITY</i>

<p align="center">
    <img title="LED Button ISR" alt="SEGGER SystemView" src="./Recordings/ledbtn_isr_recording_image.png" width="1000" height="500">
</p>
<p align="center">
    <i>
    LED button ISR scheduling captured in the SEGGER SystemView
    </i>
</p>

In the above snapshot...
1. The scheduler tick is running every 10ms
2. The user button is pressed which triggers an interrupt
3. The interrupt handler runs the button handler function. 
4. The button handler function calls xTaskNotifyFromISR(next_task_handle, 0, eNoAction, &pxHigherPriorityTaskWoken) which sends a notification to the next_task_handle (In this case it is LED_green_task)
    * <i>&pxHigherPriorityTaskWoken</i> - xTaskNotifyFromISR() will set *pxHigherPriorityTaskWoken to pdTRUE if sending the notification caused a task to unblock, and the unblocked task has a priority higher than the currently running task.
    * If this parameter was set to NULL then the scheduler would have continued to run the previously running idle task despite the LED_green_task being a higher priority and in the ready state. This would result in LED_green_task being run during the next scheduler tick rather than instantly. 
5. portYIELD_FROM_ISR(pxHigherPriorityTaskWoken) causes the scheduler to complete a context switch to run the next highest priority task which is ready 
6. The LED_green_task is run 
7. The green LED is toggled
8. the LED_green_task calls the function xTaskNotifyWait(0,0, NULL, pdMS_TO_TICKS(1000)) and notices that it has received a notification from another task
9. The LED_green_task enters a block of code which turns on the green LED, suspends the scheduler, sets the next_task_handle, resumes the scheduler, and then deletes the current LED_green_task
10. Then the scheduler returns to it's idle state until the next tick

# Resources

