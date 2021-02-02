/*
 * encoder_interface.c
 *
 * Created: 8/30/2020 2:57:25 PM
 *  Author: Jake
 */

 #include <stdint.h>
 #include <stdbool.h>
 #include "motor_task.h"
 #include "../Core/Inc/main.h"

 #include "encoder_interface.h"

// TODO both TIM1 and 3 are 16 bit, so will roll over pretty quickly, especially underflow. Need to handle cycles
 static volatile int32_t ticks_offset[NUMBER_MOTORS] = {0};
 static volatile int32_t ticks_count[NUMBER_MOTORS] = {0};
 static volatile uint16_t last_cnt[NUMBER_MOTORS] = {0};

 int32_t get_motor_encoder_ticks(uint8_t channel)
 {
	 if(channel == 0)
	 {
		 uint16_t current_count =  TIM1->CNT;
		 // Check for rollover
		 if(current_count < 5000 && last_cnt[channel] > 60000)
		 {
			 // Overflow
			 ticks_offset[channel] += 65536;
		 }
		 else if(current_count > 60000 && last_cnt[channel] < 5000)
		 {
			 // Underflow
			 ticks_offset[channel] -= 65536;
		 }

		 last_cnt[channel] = current_count;
		 ticks_count[channel] = current_count + ticks_offset[channel];
	 }
	 else if(channel == 1)
	 {
		 uint16_t current_count =  TIM3->CNT;
		 // Check for rollover
		 if(current_count < 5000 && last_cnt[channel] > 60000)
		 {
			 // Overflow
			 ticks_offset[channel] += 65536;
		 }
		 else if(current_count > 60000 && last_cnt[channel] < 5000)
		 {
			 // Underflow
			 ticks_offset[channel] -= 65536;
		 }

		 last_cnt[channel] = current_count;
		 ticks_count[channel] = current_count + ticks_offset[channel];
	 }
	 else if(channel >= NUMBER_MOTORS)
	 {
		return 0;
	 }
	 return ticks_count[channel];
 }

 void set_motor_encoder_ticks(uint8_t channel, int32_t ticks)
 {
	if(channel >= NUMBER_MOTORS)
	{
		return;
	}
	// TODO check this
	ticks_offset[channel] += (ticks - ticks_count[channel]);
 }

