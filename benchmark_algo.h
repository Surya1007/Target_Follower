/*
 * benchmark_algo.h
 *
 *  Created on: Mar 11, 2022
 *      Author: surya
 */

#ifndef INC_BENCHMARK_ALGO_H_
#define INC_BENCHMARK_ALGO_H_


void defined_square_path(float U_k[3][1], float robot_global_status[6][1], float *velocity_error, float *angular_velocity_error)
{
	/*
	 *
	 * Arguments:
	 *
	 * Outputs:
	 * U_k[3]: Array containing the control variables for the robot
	 *
	 * Inputs:
	 * robot_global_status[6]: Array containing the state variables of the robot
	 *
	 * */
	float distance = 200; // in cms
	static float target_global_loc[3] = {0, 0, 0}; // x, y, yaw angle

	// Take input (x, y, yaw angle) from predicted states from kalman filter output

	// Check if robot is at origin
	if ((robot_global_status[0][0] == 0) & (robot_global_status[1][0] == 0) & (robot_global_status[2][0] == 0))
	{
		// Go in +ve y-axis for distance cms
		target_global_loc[1] = distance;
		// Turn 90 degrees in clockwise direction
		target_global_loc[2] = 90; // In degrees
	}

	// Check if robot is at position A
	if ((robot_global_status[0][0] == 0) & (robot_global_status[1][0] == distance) & (robot_global_status[2][0] == 90))
	{
		// Then go in +ve x-axis for distance cms
		target_global_loc[0] = distance;
		// Turn 90 degrees in clockwise direction
		target_global_loc[2] = 180; // In degrees
	}

	// Check if robot is at position B
	if ((robot_global_status[0][0] == distance) & (robot_global_status[1][0] == distance) & (robot_global_status[2][0] == 180))
	{
		// Then go in -ve y-axis for distance cms
		target_global_loc[1] = 0;
		// Turn 90 degrees in clockwise direction
		target_global_loc[2] = 270; // In degrees
	}

	// Check if robot is at position C
	if ((robot_global_status[0][0] == distance) & (robot_global_status[1][0] == 0) & (robot_global_status[2][0] == 270))
	{
		// Then go in -ve x-axis for distance cms
		target_global_loc[1] = 0;
		// Turn 90 degrees in clockwise direction
		target_global_loc[2] = 0; // In degrees
	}

	float error_global_loc[3] = {	target_global_loc[0] - robot_global_status[0][0],
									target_global_loc[1] - robot_global_status[1][0],
									target_global_loc[2] - robot_global_status[2][0]};

	float k1 = 12, k2 = 12; // Constants to say veloity is proportional to distance or angle
	*velocity_error = k1 * sqrt(pow(error_global_loc[0], 2) + pow(error_global_loc[1], 2));
	*angular_velocity_error = k2 * (error_global_loc[2] + atan(error_global_loc[1] / error_global_loc[0]));

	// Should output the velocity and angular velocity at which robot should move

}


#endif /* INC_BENCHMARK_ALGO_H_ */
