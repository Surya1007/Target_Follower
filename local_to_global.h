/*
 * local_to_global.h
 * Header file to convert sensor data which is in the local reference frame of the robot
 * to global reference frame
 *
 *  Created on: 11-Mar-2022
 *      Author: surya
 */

#ifndef INC_LOCAL_TO_GLOBAL_H_
#define INC_LOCAL_TO_GLOBAL_H_


#include "math.h"

void sensor_global_reference(float *global_parameter, float *parameter, float yaw)
{
	/*
	 * Parameter is an array of a type of data collected from IMU
	 *
	 *
	 * Arguments:
	 *
	 * Outputs:
	 * global_parameter: Pointer to point to the array to store global data (x, y, z)
	 *
	 * Inputs:
	 * parameter: Array of local sensor data (x, y, z)
	 * yaw: yaw angle wrt Magnetic North
	 *
	 * */


	global_parameter[0] = (parameter[0] * cos(yaw)) + (parameter[1] * sin(yaw));
	global_parameter[1] = (parameter[0] * sin(yaw)) + (parameter[1] * cos(yaw));


}


#endif /* INC_LOCAL_TO_GLOBAL_H_ */
