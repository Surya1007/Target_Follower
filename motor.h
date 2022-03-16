/*
 * motor.h
 *
 *  Created on: 06-Mar-2022
 *      Author: surya
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "main.h"


TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

void left_wheel_control(uint8_t power, uint8_t direction)
{
	/*
	 * Function to communicate with left motor
	 * Arguments:
	 *
	 *
	 * Outputs:
	 * Varies power sent to left motor
	 *
	 * Inputs:
	 * power: PWM value to be sent to the left motor
	 * direction: Direction in which left motor should rotate
	 * 			  1: Forward direction rotation (counter-clockwise rotation)
	 * 			  0: Backward direction rotation (clockwise rotation)
	 *
	 * */


	// Direction = 1 means wheel should rotate in counter-clockwise direction
	if (direction == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	// Direction = 0 means wheel should rotate in clockwise direction
	else if (direction == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	}
	// Sends pwm value to left motor
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, power);
}

float read_left_wheel(float pres_time)
{
	/*
	 * Function to measure left wheel encoder
	 * Arguments:
	 *
	 * Outputs:
	 * left_prev_rotation: Rotations done till previous measurement
	 * prev_time: Time instant at which we previously measured the left wheel encoder
	 * */
	uint16_t left_after_rotation, left_change_rotation;
	static float prev_time = 0;
	static uint16_t left_prev_rotation = 0;
	float encoder_update_time;
	static uint16_t lcount = 0;

	left_after_rotation = (lcount / 20); // Outputs the number of revolutions since the encoder has 20 slots
	//pres_time = __HAL_TIM5_GetCounter(&htim5);

	left_change_rotation = left_after_rotation - left_prev_rotation; // Change in the number of rotations for left wheel from the previous measurement
	encoder_update_time = (pres_time - prev_time) / 1000000; // Outputs the time between measurements in sec
	printf("\r%d, %f, %f, ", left_change_rotation, left_change_rotation / encoder_update_time, encoder_update_time);

	prev_time = pres_time;
	left_prev_rotation = left_after_rotation;

	return left_change_rotation / encoder_update_time;
}



void right_wheel_control(uint16_t power, uint8_t direction)
{
	/*
	* Function to communicate with right motor
	* Arguments:
	*
	*
	* Outputs:
	* Varies power sent to right motor
	*
	* Inputs:
	* power: PWM value to be sent to the right motor
	* direction: Direction in which right motor should rotate
	* 			  1: Forward direction rotation (clockwise rotation)
	* 			  0: Backward direction rotation (counter-clockwise rotation)
	*
	* */


	// Direction = 1 means wheel should rotate in clockwise direction
	if (direction == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	}
	// Direction = 0 means wheel should rotate in counter-clockwise direction
	else if (direction == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	// Sends pwm value to left motor
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, power);


}

float read_right_wheel(float pres_time)
{
	/*
	 * Function to measure right wheel encoder
	 * Arguments:
	 *
	 * Outputs:
	 * right_prev_rotation: Rotations done till previous measurement
	 * prev_time: Time instant at which we previously measured the right wheel encoder
	 *
	 * */
	uint16_t right_after_rotation, right_change_rotation;
	static float prev_time = 0;
	static uint16_t right_prev_rotation = 0;
	float encoder_update_time;
	static uint16_t rcount = 0;

	right_after_rotation = (rcount / 20); // Outputs the number of revolutions since the encoder has 20 slots
	//pres_time = __HAL_TIM5_GetCounter(&htim5);

	right_change_rotation = right_after_rotation - right_prev_rotation; // Change in the number of rotations for right wheel from the previous measurement
	encoder_update_time = (pres_time - prev_time) / 1000000; // Outputs the time between measurements in sec
	printf("%d, %f, %f, %d,\n", right_change_rotation, right_change_rotation / encoder_update_time, encoder_update_time, rcount);

	prev_time = pres_time;
	right_prev_rotation = right_after_rotation;

	return right_change_rotation / encoder_update_time;
}

#endif /* INC_MOTOR_H_ */
