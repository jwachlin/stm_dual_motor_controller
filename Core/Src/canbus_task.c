/*
 * canbus_task.c
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

#include <stdint.h>
#include <string.h>

#include "../../Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
#include "../../Middlewares/Third_Party/FreeRTOS/Source/include/queue.h"
#include "../../lib/can_messages.h"
#include "../../lib/motion_primitives.h"
#include "../../lib/encoder_interface.h"
#include "motor_task.h"

 #include "canbus_task.h"

 #define CAN_QUEUE_LENGTH		(15)

 static QueueHandle_t canbus_rx_queue = NULL;
 static QueueHandle_t canbus_tx_queue = NULL;
 static QueueSetHandle_t canbus_queue_set;

 static uint32_t this_device = 1;
 static CAN_HandleTypeDef can_h;

 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
 {
	 // ISR
	 CAN_RxHeaderTypeDef header;
	 canbus_frame_t can_frame;
	 HAL_CAN_GetRxMessage(_hcan, 0, &header, can_frame.data);
	 can_frame.id = header.StdId;
	 can_frame.length = header.DLC;
	 add_can_frame_to_rx_queue_from_isr(can_frame);
 }

void canbus_task_init(CAN_HandleTypeDef can_handle)
{
	this_device = (((HAL_GPIO_ReadPin(INDEX1_GPIO_Port, INDEX1_Pin) == GPIO_PIN_SET) ? 0 : 1) << 0);
	this_device += (((HAL_GPIO_ReadPin(INDEX2_GPIO_Port, INDEX2_Pin) == GPIO_PIN_SET) ? 0 : 1) << 1);
	this_device += (((HAL_GPIO_ReadPin(INDEX3_GPIO_Port, INDEX3_Pin) == GPIO_PIN_SET) ? 0 : 1) << 2);
	this_device += 1; // Always offset by 1, the main controller is 0

	canbus_queue_set = xQueueCreateSet( 2 * CAN_QUEUE_LENGTH );
	canbus_rx_queue = xQueueCreate( CAN_QUEUE_LENGTH, sizeof(canbus_frame_t) );
	canbus_tx_queue = xQueueCreate( CAN_QUEUE_LENGTH, sizeof(canbus_frame_t) );

	xQueueAddToSet( canbus_rx_queue, canbus_queue_set );
	xQueueAddToSet( canbus_tx_queue, canbus_queue_set );

	can_h = can_handle;

	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh=0;
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;

	HAL_CAN_ConfigFilter(&can_h, &sFilterConfig); //configure CAN filter

	HAL_CAN_Start(&can_h);

	HAL_CAN_ActivateNotification(&can_h, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&can_h, CAN_IT_RX_FIFO1_MSG_PENDING );
	HAL_CAN_ActivateNotification(&can_h, CAN_IT_TX_MAILBOX_EMPTY);
}

void canbus_task(void)
{
	QueueSetMemberHandle_t xActivatedMember;
	canbus_frame_t can_frame;

	xActivatedMember = xQueueSelectFromSet( canbus_queue_set, portMAX_DELAY);

	if(xActivatedMember == canbus_rx_queue)
	{
		xQueueReceive( xActivatedMember, &can_frame, 0 );
		can_message_id_t msg;
		msg.raw_id = can_frame.id;
		unpack_can_message(&msg);

		if(msg.can_device == this_device)
		{
			if(msg.can_msg_type == CAN_MSG_TYPE_CMD)
			{
				if(msg.can_class == CAN_MSG_CLASS_CMD_CONTROL)
				{
					if(msg.can_index == CAN_MSG_INDEX_CMD_POSITION)
					{
						uint8_t motor_index = 0;
						float position = 0;
						motor_index = can_frame.data[0];
						memcpy(&position, &can_frame.data[1], 4);
						set_control_mode(POSITION, motor_index);
						set_motor_position(position, motor_index);
					}
					else if(msg.can_index == CAN_MSG_INDEX_CMD_SPEED)
					{
						uint8_t motor_index = 0;
						int32_t speed = 0;
						motor_index = can_frame.data[0];
						memcpy(&speed, &can_frame.data[1], 4);
						set_control_mode(SPEED, motor_index);
						set_motor_speed(speed, motor_index);
					}
					else if(msg.can_index == CAN_MSG_INDEX_CMD_DUTY)
					{
						uint8_t motor_index = 0;
						float duty = 0;
						motor_index = can_frame.data[0];
						memcpy(&duty, &can_frame.data[1], 4);
						set_control_mode(DUTY, motor_index);
						set_motor_duty(duty, motor_index);
					}
					else if(msg.can_index == CAN_MSG_INDEX_CMD_PRIMITIVE)
					{
						 // Primitives affect both motors
						int16_t tau_ms = 0;
						int16_t t_offset_ms = 0;
						uint8_t primitive_index = can_frame.data[0];
						if(primitive_index < NUMBER_PRIMITIVES)
						{
							memcpy(&tau_ms, &can_frame.data[1], 2);
							memcpy(&t_offset_ms, &can_frame.data[3], 2);
							uint8_t invert = can_frame.data[5];
							uint8_t time_reverse = can_frame.data[6];
							motion_primitive_set_index(primitive_index);
							motion_primitive_set_timing(primitive_index, (float) tau_ms * 0.001, (float) t_offset_ms * 0.001, invert, time_reverse);
							set_control_mode(PRIMITIVE, 0);
							set_control_mode(PRIMITIVE, 1);
						}
					}
					else if(msg.can_index == CAN_MSG_INDEX_CMD_PROPRIOCEPTIVE_PRIMITIVE)
					{
						// Primitives affect both motors
						int16_t tau_ms = 0;
						int16_t t_offset_ms = 0;
						uint8_t primitive_index = can_frame.data[0];
						if(primitive_index < NUMBER_PRIMITIVES)
						{
							memcpy(&tau_ms, &can_frame.data[1], 2);
							memcpy(&t_offset_ms, &can_frame.data[3], 2);
							uint8_t invert = can_frame.data[5];
							uint8_t time_reverse = can_frame.data[6];
							motion_primitive_set_index(primitive_index);
							motion_primitive_set_timing(primitive_index, (float) tau_ms * 0.001, (float) t_offset_ms * 0.001, invert, time_reverse);
							set_control_mode(PROPRIOCEPTIVE_PRIMITIVE, 0);
							set_control_mode(PROPRIOCEPTIVE_PRIMITIVE, 1);
						}
					}
				}
				else if(msg.can_class == CAN_MSG_CLASS_CMD_SET_PARAM)
				{
					if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_PRIM_SCALE)
					{
						int8_t x_off, y_off;
						uint8_t primitive_index = can_frame.data[0];
						memcpy(&x_off, &can_frame.data[1], 1);
						memcpy(&y_off, &can_frame.data[2], 1);
						float x_scale = 0.01 *  can_frame.data[3];
						float y_scale = 0.01 *  can_frame.data[4];
						// Offsets received in mm, m used in code
						motion_primitive_set_scaling(primitive_index, 0.001*x_off,0.001*y_off, x_scale, y_scale);
					} else if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_PD_MINMAX)
					{
						uint8_t index_type = can_frame.data[0];
						uint8_t index = (index_type & 0x01);
						uint8_t ctrl_type = ((index_type & 0x06) >> 0x01);
						int16_t kp_temp, kd_temp;
						memcpy(&kp_temp, &can_frame.data[1], 2);
						memcpy(&kd_temp, &can_frame.data[3], 2);
						uint8_t speed_filt_pct = can_frame.data[5];
						uint8_t cmd_max_pct = can_frame.data[6];
						uint8_t cmd_min_pct = can_frame.data[7];
						float kp = kp_temp * 0.01;
						float kd = kd_temp * 1.0;
						float alpha = speed_filt_pct * 0.01;
						float cmd_max = cmd_max_pct * 0.01;
						float cmd_min = cmd_min_pct * 0.01;
						if(alpha > 0.99)
						{
							alpha = 0.99;
						}
						if(alpha < 0.0)
						{
							alpha = 0.0;
						}
						if(cmd_max > 1.0)
						{
							cmd_max = 1.0;
						}
						if(cmd_max < 0.0)
						{
							cmd_max = 0.0;
						}
						if(cmd_min > 1.0)
						{
							cmd_min = 1.0;
						}
						if(cmd_min < 0.0)
						{
							cmd_min = 0.0;
						}
						set_control_params_kp_min_max(index, ctrl_type, kp, kd, alpha, cmd_min, cmd_max);
					} else if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_KI)
					{
						uint8_t index_type = can_frame.data[0];
						uint8_t index = (index_type & 0x01);
						uint8_t ctrl_type = ((index_type & 0x06) >> 0x01);
						float ki;
						memcpy(&ki, &can_frame.data[1], 4);
						set_control_params_ki(index, ctrl_type, ki);
					} else if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_MAX_INTEGRAL)
					{
						uint8_t index_type = can_frame.data[0];
						uint8_t index = (index_type & 0x01);
						uint8_t ctrl_type = ((index_type & 0x06) >> 0x01);
						float int_max;
						memcpy(&int_max, &can_frame.data[1], 4);
						set_control_params_int_max(index, ctrl_type, int_max);
					} else if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_MIN_INTEGRAL)
					{
						uint8_t index_type = can_frame.data[0];
						uint8_t index = (index_type & 0x01);
						uint8_t ctrl_type = ((index_type & 0x06) >> 0x01);
						float int_min;
						memcpy(&int_min, &can_frame.data[1], 4);
						set_control_params_int_min(index, ctrl_type, int_min);
					} else if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_TICKS_PER_REV)
					{
						uint8_t index = can_frame.data[0];
						int32_t ticks_per_rev;
						memcpy(&ticks_per_rev, &can_frame.data[1], 4);
						set_motor_ticks_per_rev(ticks_per_rev, index);
					} else if(msg.can_index == CAN_MSG_INDEX_CMD_PARAM_PRIM_KEYFRAME)
					{
						uint8_t prim_index = can_frame.data[0];
						uint8_t keyframe_index = can_frame.data[1];
						int16_t x_temp, y_temp;
						uint16_t t_part_temp;
						memcpy(&x_temp, &can_frame.data[2], 2);
						memcpy(&y_temp, &can_frame.data[4], 2);
						memcpy(&t_part_temp, &can_frame.data[6], 2);
						motion_primitive_set_keyframe(prim_index, keyframe_index, x_temp * 0.001, y_temp * 0.001, t_part_temp * 1.5259e-5);
					}
				}
				else if(msg.can_class == CAN_MSG_CLASS_CMD_ZERO_POS)
				{
					// TODO index?
					uint8_t motor_index = can_frame.data[0];
					set_motor_encoder_ticks(motor_index, 0);
				}
				else if(msg.can_class == CAN_MSG_CLASS_CMD_TIME)
				{
					uint32_t external_time = 0;
					memcpy(&external_time, &can_frame.data[0], 4);
					motion_primitive_time_sync(external_time);
				}
			}
		}

		// handle other messages
	}
	else if(xActivatedMember == canbus_tx_queue)
	{
		if(xQueueReceive( xActivatedMember, &can_frame, 0 ) == pdTRUE)
		{
			CAN_TxHeaderTypeDef header;
			header.DLC = can_frame.length;
			header.StdId = can_frame.id;
			header.RTR = 0;
			header.IDE = CAN_ID_STD;
			header.TransmitGlobalTime = DISABLE;
			uint32_t mailbox = 0;
			HAL_CAN_AddTxMessage(&can_h, &header, can_frame.data, &mailbox);
		}
	}
}

/*
*	\brief Gets the device index
*/
uint32_t get_device_index(void)
{
	return this_device;
}

/*
*	\brief Adds CAN frame to queue to process
*
*	\param frame The frame to process
*
*	\return True if added to queue, false otherwise
*/
bool add_can_frame_to_rx_queue(canbus_frame_t frame)
 {
	  bool success = false;
	  if(canbus_rx_queue)
	  {
		  success = (xQueueSend(canbus_rx_queue, &frame, 0) == pdTRUE);
	  }
	  return success;
 }

/*
*	\brief Adds CAN frame to queue to process from ISR
*
*	\param frame The frame to process
*
*	\return True if added to queue, false otherwise
*/
bool add_can_frame_to_rx_queue_from_isr(canbus_frame_t frame)
{
	bool success = false;
	if(canbus_rx_queue)
	{
		success = (xQueueSendFromISR(canbus_rx_queue, &frame, NULL) == pdTRUE);
	}
	return success;
}

 /*
*	\brief Adds CAN frame to queue to send
*
*	\param frame The frame to send
*
*	\return True if added to queue, false otherwise
*/
bool add_can_frame_to_tx_queue(canbus_frame_t frame)
{
	bool success = false;
	if(canbus_tx_queue)
	{
		success = (xQueueSend(canbus_tx_queue, &frame, 0) == pdTRUE);
	}
	return success;
}

