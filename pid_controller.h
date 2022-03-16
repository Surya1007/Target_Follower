/*
 * pid_controller.h
 *
 *  Created on: 05-Mar-2022
 *      Author: surya
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_


void PID_control_velocity(float *output_velocity, float error_velocity, float delta_t)
{
	/*
	 * Arguments:
	 *
	 *
	 * Outputs:
	 * output_velocity: Pointer to output velocity which is further feed to differential drive model
	 *
	 * Inputs:
	 * error_velocity: Takes the error velocity by calculating the difference of feedback velocity from reference velocity given by the algorithm
	 * delta_t: Time difference between previous control and present control action in PID part
	 *
	 *
	 * */
	float Kp, Ki, Kd;
	static float velocity_error_sum;
	static float prev_error_velocity;

	*output_velocity = (Kp * error_velocity) + (Ki * (velocity_error_sum + error_velocity) * delta_t) + (Kd * (error_velocity - prev_error_velocity) / delta_t);
}





void PID_control_angular_velocity(float *output_angular_velocity, float error_angular_velocity, float delta_t)
{
	/*
	 * Arguments:
	 *
	 *
	 * Outputs:
	 * output_angular_velocity: Pointer to output angular velocity which is further feed to differential drive model
	 *
	 * Inputs:
	 * error_angular_velocity: Takes the error angular velocity by calculating the difference of feedback angular velocity from reference angular velocity given by the algorithm
	 * delta_t: Time difference between previous control and present control action in PID part
	 *
	 *
	 * */
	float Kp, Ki, Kd;
	static float angular_velocity_error_sum;
	static float prev_error_angular_velocity;

	*output_angular_velocity = (Kp * error_angular_velocity) + (Ki * (angular_velocity_error_sum + error_angular_velocity) * delta_t) + (Kd * (error_angular_velocity - prev_error_angular_velocity) / delta_t);
}


#endif /* INC_PID_CONTROLLER_H_ */
