/*
 * can_messages.c
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

#include "can_messages.h"

 void unpack_can_message(can_message_id_t * msg)
 {
	msg->can_msg_type = ((msg->raw_id & CAN_MSG_TYPE_MASK) >> CAN_MSG_TYPE_SHIFT);
	msg->can_class = ((msg->raw_id & CAN_MSG_CLASS_MASK) >> CAN_MSG_CLASS_SHIFT);
	msg->can_index = ((msg->raw_id & CAN_MSG_INDEX_MASK) >> CAN_MSG_INDEX_SHIFT);
	msg->can_device = ((msg->raw_id & CAN_MSG_DEVICE_MASK) >> CAN_MSG_DEVICE_SHIFT);
 }

 void pack_can_message(can_message_id_t * msg)
 {
	msg->raw_id = ((msg->can_msg_type << CAN_MSG_TYPE_SHIFT) & CAN_MSG_TYPE_MASK);
	msg->raw_id |= ((msg->can_class << CAN_MSG_CLASS_SHIFT) & CAN_MSG_CLASS_MASK);
	msg->raw_id |= ((msg->can_index << CAN_MSG_INDEX_SHIFT) & CAN_MSG_INDEX_MASK);
	msg->raw_id |= ((msg->can_device << CAN_MSG_DEVICE_SHIFT) & CAN_MSG_DEVICE_MASK);
 }
