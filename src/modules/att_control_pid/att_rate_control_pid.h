/*************************************************************
 * Copyright @ Dullerud's Autonomous Lab
 * University of Illinois Urbana Champaign
 * 
 * Author @ Bicheng Zhang <viczhang1990@gmail.com>
 * 
 * <-----------------Drone Software------------------------>
 *
 *
 *            ********                  *****
 *             -A---               --A---
 *                
 *                      U U
 *                      ^
 *
 *
 *
 ************************************************************/

/*
 * @file att_rate_control_pid.h
 *
 * Definition of rate controller for multirotors.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Laurens Mackay <mackayl@student.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Martin Rutschmann <rutmarti@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 *
 * @Developer Bicheng Zhang <zhang368@illinois.edu>
 */

#ifndef ATT_RATE_CONTROL_PID_H_
#define ATT_RATE_CONTROL_PID_H_

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_controls.h>

void att_control_rates_pid(const struct vehicle_rates_setpoint_s *rate_sp,
			      const float rates[], struct actuator_controls_s *actuators, bool reset_integral);

#endif /* ATT_RATE_CONTROL_PID_H_ */
