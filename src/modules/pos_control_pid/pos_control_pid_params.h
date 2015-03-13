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
  *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
 */

/*
 * @file pos_control_pid_params.h
 * 
 * Parameters for position controller
 */

#include <systemlib/param/param.h>

struct pos_control_pid_params {
	//TODO
	float pos_p;
	float pos_d;
	float pos_i;
	float vel_p;
	float vel_i;
	float vel_d;

	float height_p;
	float height_i;
	float height_d;
	//float height_rate;
	float height_min;
	float height_max;
	float thrust_feedforward;
	//float limit_speed_x;
	//float limit_speed_y;
	float limit_height_err;
	float limit_height_int;
	float limit_pos_err;
	float limit_pos_int;
	float limit_vel_err;
	float limit_vel_int;

	//float limit_thrust_int;
	float limit_thrust_upper;
	float limit_thrust_lower;
	//float limit_yaw_step;
	//float manual_threshold;
	float rc_scale_pitch;
	float rc_scale_roll;
	float rc_scale_yaw;

	// speed param added later on
	float limit_pitch;
	float limit_roll;
	float trim_roll;
	float trim_pitch;

	// takeoff param added later on
	//float takeoff_alt;
	//float takeoff_gap;


};

struct pos_control_pid_param_handles {
	param_t pos_p;
	param_t pos_d;
	param_t pos_i;

	param_t vel_p;
	param_t vel_i;
	param_t vel_d;


	param_t height_p;
	param_t height_i;
	param_t height_d;
	//param_t height_rate;
	param_t height_min;
	param_t height_max;
	param_t thrust_feedforward;
	//param_t limit_speed_x;
	//param_t limit_speed_y;
	//param_t limit_height_error;
	param_t limit_height_err;
	param_t limit_height_int;
	param_t limit_pos_err;
	param_t limit_pos_int;
	param_t limit_vel_err;
	param_t limit_vel_int;

	param_t limit_thrust_upper;
	param_t limit_thrust_lower;
	//param_t limit_yaw_step;
	//param_t manual_threshold;
	param_t rc_scale_pitch;
	param_t rc_scale_roll;
	param_t rc_scale_yaw;

	// speed param_t later on
	param_t limit_pitch;
	param_t limit_roll;
	param_t trim_roll;
	param_t trim_pitch;

	// takeoff param_t later on
	//param_t takeoff_alt;
	//param_t takeoff_gap;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct pos_control_pid_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct pos_control_pid_param_handles *h, struct pos_control_pid_params *p);
