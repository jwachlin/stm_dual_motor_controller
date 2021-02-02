/*
 * encoder_interface.h
 *
 * Created: 8/30/2020 2:57:14 PM
 *  Author: Jake
 */


#ifndef ENCODER_INTERFACE_H_
#define ENCODER_INTERFACE_H_

#include <stdint.h>

int32_t get_motor_encoder_ticks(uint8_t channel);
void set_motor_encoder_ticks(uint8_t channel, int32_t ticks);

#endif /* ENCODER_INTEFACE_H_ */
