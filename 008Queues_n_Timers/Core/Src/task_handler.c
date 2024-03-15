/*
 * task_handler.c
 *
 *  Created on: Mar 14, 2024
 *      Author: rahim
 */

#include "main.h"

int extract_command(command_t *cmd);
void process_command(command_t *cmd);

void menu_task(void* param)
{
	uint32_t cmd_addr;// Command address that is filled by the notification value
	command_t *cmd;// Pointer to the command object
	int option;
	const char* msg_inv = "////Invalid option////\n";
	const char* msg_menu = "====================\n"
						   "|       Menu        \n"
						   "====================\n"
						   "LED effect    ---> 0\n"
						   "Date and time ---> 1\n"
			   	   	   	   "Exit          ---> 2\n"
			   	   	   	   "Enter your choice here :\n";

	while(1)
	{
		xQueueSend(q_print, &msg_menu, portMAX_DELAY);

		xTaskNotifyWait(0,0,&cmd_addr, portMAX_DELAY);
		cmd = (command_t*)cmd_addr; // Point the command object to the command address received in the notification

		if(cmd->len == 1) // Check for user entry
		{
			option = cmd->payload[0] - 48; // Convert ASCII to a number
			switch(option)
			{
				case 0:
					curr_state = sLedEffect;
					xTaskNotify(handle_led_task,0,eNoAction);
					break;
				case 1:
					curr_state = sRtcMenu;
					xTaskNotify(handle_rtc_task,0,eNoAction);
					break;
				case 2: /*implement exit */
					break;
				default:
					xQueueSend(q_print,&msg_inv,portMAX_DELAY);
					continue;
			}

		}else{
			//invalid entry
			xQueueSend(q_print,&msg_inv,portMAX_DELAY);
			continue;
		}


		//wait to run again when some other task notifies
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
	}
}

void led_task(void* param)
{
	uint32_t cmd_addr;
	command_t *cmd;
	const char* msg_inv = "////Invalid option////\n";
	const char* msg_led = "========================\n"
						  "|      LED Effect      |\n"
						  "========================\n"
						  "(none,e1,e2,e3,e4)\n"
						  "Enter your choice here : ";
	while(1)
	{
		/*Wait for notification (Notify wait) */
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		/*Print LED menu */
		xQueueSend(q_print,&msg_led,portMAX_DELAY);

		/*wait for LED command (Notify wait) */
		xTaskNotifyWait(0,0,&cmd_addr,portMAX_DELAY);
		cmd = (command_t*)cmd_addr;

		if(cmd->len <= 4)
		{
			if(! strcmp((char*)cmd->payload,"none"))
			{
				led_effect_stop();
			}
			else if (! strcmp((char*)cmd->payload,"e1"))
			{
				led_effect(1);
			}
			else if (! strcmp((char*)cmd->payload,"e2"))
			{
				led_effect(2);
			}
			else if (! strcmp((char*)cmd->payload,"e3"))
			{
				led_effect(3);
			}
			else if (! strcmp((char*)cmd->payload,"e4"))
			{
				led_effect(4);
			}
			else
			{
				xQueueSend(q_print,&msg_inv,portMAX_DELAY); /*print invalid message */
				continue;
			}
		}
		else
		{
			xQueueSend(q_print,&msg_inv,portMAX_DELAY);
			continue;
		}

		/* update state variable */
		curr_state = sMainMenu;

		/*Notify menu task */
		xTaskNotify(handle_menu_task,0,eNoAction);

	}
}

void rtc_task(void* param)
{
	while(1)
	{
	}
}

void print_task(void* param)
{
	uint32_t *msg;
	BaseType_t queue_status;
	while(1)
	{
		queue_status = xQueueReceive(q_print, &msg, portMAX_DELAY);
		if (queue_status == pdTRUE)
		{
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen((char*)msg), HAL_MAX_DELAY);
		}
	}
}

void cmd_handler_task(void* param)
{
	BaseType_t ret;
	command_t cmd;// Create a new command object

	while(1)
	{
		// Wait indefinitely until a notification is received
		ret = xTaskNotifyWait(0,0,NULL, portMAX_DELAY);

		if (ret == pdTRUE)
		{
			// Process the user data (command) stored in the input data queue
			process_command(&cmd);
		}


	}
}

void process_command(command_t *cmd)
{
	extract_command(cmd);

	switch(curr_state)
	{
	case sMainMenu:
		xTaskNotify(handle_menu_task, (uint32_t)cmd, eSetValueWithOverwrite);
		break;
	case sLedEffect:
		xTaskNotify(handle_led_task, (uint32_t)cmd, eSetValueWithOverwrite);
		break;
	case sRtcMenu:
	case sRtcTimeConfig:
	case sRtcDateConfig:
	case sRtcReport:
		xTaskNotify(handle_rtc_task, (uint32_t)cmd, eSetValueWithOverwrite);
		break;
	}
}

int extract_command(command_t *cmd)
{
	uint8_t item;//Placeholder data holder
	BaseType_t status;

	//Check if the queue has any data in it
	status = uxQueueMessagesWaiting(q_data);
	if (!status) return -1;
	uint8_t i =0;
	do
	{
		status = xQueueReceive(q_data, &item, 0);
		if (status == pdTRUE) cmd->payload[i++] = item;
	}while(item != '\n');

	cmd->payload[i-1] = '\0';
	cmd->len = i-1; // Save the length of the command excluding null char

	return 0;
}
