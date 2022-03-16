/*
 * diff_model.h
 *
 *  Created on: 05-Mar-2022
 *      Author: surya
 */

#ifndef INC_DIFF_MODEL_H_
#define INC_DIFF_MODEL_H_


void diff_drive_model(float Y[][1], float X[][1], float U[][1], float delta_t)
{
	float C[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	float D[3][3] = {{delta_t, 0, 0}, {0, delta_t, 0}, {0, 0, delta_t}};
	// Y = CX + DU


}





void unicycle_to_diff_model(float *velocity_left_wheel, float *velocity_right_wheel, float *velocity, float *angular_velocity)
{
	/*
	 * Function to convert uni-cycle model into differential drive robot model
	 * Arguments:
	 *
	 *
	 * Outputs:
	 * velocity_left_wheel: Angular velocity at which left wheel should move
	 * velocity_right_wheel: Angular velocity at which right wheel should move
	 *
	 * Inputs:
	 * velocity: Velocity of the system in uni-cycle model
	 * 			 If system moves forward, velocity is +ve
	 * 			 If system moves backward, velocity is -ve
	 * angular_velocity: Angular velocity of the system in uni-cycle model
	 * 					 If system rotates in clockwise direction, angular_velocity is +ve
	 * 					 If system rotates in counter-clockwise direction, angular_velocity is -ve
	 *
	 *
	 * */
	float base_length = 5, radius_wheel = 5; // Should be in cms

	// Calculate the angular velocity at which the left wheel should rotate
	*velocity_left_wheel = (2 * *velocity) - (*angular_velocity * base_length) / radius_wheel;
	// Calculate the angular velocity at which the right wheel should rotate
	*velocity_right_wheel = (2 * *velocity) + (*angular_velocity * base_length) / radius_wheel;


}


void diff_to_unicycle(float *velocity_left_wheel, float *velocity_right_wheel, float *velocity, float *angular_velocity)
{
	/*
	 * Function to convert differential drive robot model into uni-cycle model
	 * Arguments:
	 *
	 *
	 * Outputs:
	 * velocity: Velocity of the system in uni-cycle model
	 * 			 If system moves forward, velocity is +ve
	 * 			 If system moves backward, velocity is -ve
	 * angular_velocity: Angular velocity of the system in uni-cycle model
	 * 					 If system rotates in clockwise direction, angular_velocity is +ve
	 * 					 If system rotates in counter-clockwise direction, angular_velocity is -ve
	 *
	 * Inputs:
	 * velocity_left_wheel: Angular velocity at which left wheel should move
	 * velocity_right_wheel: Angular velocity at which right wheel should move
	 *
	 *
	 * */
	float base_length = 5, radius_wheel = 5; // Should be in cms

	// Calculate the velocity of unicycle model
	*velocity = radius_wheel * (*velocity_left_wheel + *velocity_right_wheel) / 2;
	// Calculate the angular velocity of unicycle model
	*angular_velocity = radius_wheel * (*velocity_right_wheel - *velocity_left_wheel) / base_length;


}


void velocity_to_pwm(uint16_t *left_pwm, uint16_t *right_pwm, float left_wheel_velocity, float right_wheel_velocity)
{
	/*
	 * Function to convert velocity values into pwm values to control the motor
	 * Arguments:
	 *
	 *
	 * Outputs:
	 * left_pwm: PWM value to be sent to left motor
	 * 			 +ve value indicates wheel should rotate so that robot moves forward (wrt observer rotate in counter-clockwise direction)
	 * 			 -ve value indicates wheel should rotate so that robot moves backward (wrt observer rotate in clockwise direction)
	 * right_pwm: PWM value to be sent to right motor
	 * 			 +ve value indicates wheel should rotate so that robot moves forward (wrt observer rotate in clockwise direction)
	 * 			 -ve value indicates wheel should rotate so that robot moves backward (wrt observer rotate in counter-clockwise direction)
	 *
	 * Inputs:
	 * left_wheel_velocity: Velocity at which left wheel should rotate
	 * right_wheel_velocity: Velocity at which right wheel should rotate
	 *
	 *
	 * */



	// Need to perform experiments to verify this


}


#endif /* INC_DIFF_MODEL_H_ */
