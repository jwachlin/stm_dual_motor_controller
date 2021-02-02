/*
 * canbus_task.h
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

#ifndef INC_CANBUS_TASK_H_
#define INC_CANBUS_TASK_H_

#include <stdbool.h>
#include "main.h"

typedef struct
{
	uint32_t id;
	uint8_t length;
	uint8_t data[8];
} canbus_frame_t;

void canbus_task_init(CAN_HandleTypeDef can_handle);
void canbus_task(void);

uint32_t get_device_index(void);
bool add_can_frame_to_rx_queue(canbus_frame_t frame);
bool add_can_frame_to_rx_queue_from_isr(canbus_frame_t frame);
bool add_can_frame_to_tx_queue(canbus_frame_t frame);

#endif /* INC_CANBUS_TASK_H_ */
