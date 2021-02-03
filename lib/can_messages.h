/*
 * can_messages.h
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

#ifndef CAN_MESSAGES_H_
#define CAN_MESSAGES_H_

#include <stdint.h>

#define CAN_MSG_TYPE_SHIFT				(10ul)
#define CAN_MSG_TYPE_MASK				(1ul << CAN_MSG_TYPE_SHIFT)

#define CAN_MSG_CLASS_SHIFT				(7ul)
#define CAN_MSG_CLASS_MASK				(7ul << CAN_MSG_CLASS_SHIFT)

#define CAN_MSG_INDEX_SHIFT				(4ul)
#define CAN_MSG_INDEX_MASK				(7ul << CAN_MSG_INDEX_SHIFT)

#define CAN_MSG_DEVICE_SHIFT			(0ul)
#define CAN_MSG_DEVICE_MASK				(15ul << CAN_MSG_DEVICE_SHIFT)

#define CAN_MSG_TYPE_CMD					(0)
#define CAN_MSG_TYPE_INFO					(1)

#define CAN_MSG_CLASS_CMD_CONTROL			(0)
#define CAN_MSG_CLASS_CMD_TIME				(1)
#define CAN_MSG_CLASS_CMD_SET_PARAM			(2)
#define CAN_MSG_CLASS_CMD_ZERO_POS			(3)

#define CAN_MSG_INDEX_CMD_PARAM_PRIM_SCALE		(0)
#define CAN_MSG_INDEX_CMD_PARAM_PD_MINMAX		(1)
#define CAN_MSG_INDEX_CMD_PARAM_KI				(2)
#define CAN_MSG_INDEX_CMD_PARAM_MAX_INTEGRAL	(3)
#define CAN_MSG_INDEX_CMD_PARAM_MIN_INTEGRAL	(4)
#define CAN_MSG_INDEX_CMD_PARAM_TICKS_PER_REV	(5)
#define CAN_MSG_INDEX_CMD_PARAM_PRIM_KEYFRAME	(6)

#define CAN_MSG_INDEX_CMD_POSITION					(0)
#define CAN_MSG_INDEX_CMD_SPEED						(1)
#define CAN_MSG_INDEX_CMD_CURRENT					(2)
#define CAN_MSG_INDEX_CMD_PRIMITIVE					(3)
#define CAN_MSG_INDEX_CMD_DUTY						(4)
#define CAN_MSG_INDEX_CMD_PROPRIOCEPTIVE_PRIMITIVE	(5)

#define CAN_MSG_CLASS_INFO_TELEMETRY		(0)

#define CAN_MSG_INDEX_INFO_POSITION			(0)
#define CAN_MSG_INDEX_INFO_CURRENT			(1)
#define CAN_MSG_INDEX_INFO_SPEED			(2)
#define CAN_MSG_INDEX_INFO_POSITION_SETPOINT			(3)
#define CAN_MSG_INDEX_INFO_CURRENT_SETPOINT				(4)
#define CAN_MSG_INDEX_INFO_PRIMITIVE_SETPOINT			(5)
#define CAN_MSG_INDEX_INFO_PROPRIO_FORCE				(6)
#define CAN_MSG_INDEX_INFO_DUTY							(7)

typedef struct
{
	uint32_t raw_id;
	uint32_t can_msg_type;
	uint32_t can_class;
	uint32_t can_index;
	uint32_t can_device;
} can_message_id_t;

void unpack_can_message(can_message_id_t * msg);
void pack_can_message(can_message_id_t * msg);

#endif /* CAN_MESSAGES_H_ */
