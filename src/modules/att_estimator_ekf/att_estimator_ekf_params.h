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
    Author: Tobias Naegeli <naegelit@student.ethz.ch>
            Lorenz Meier <lm@inf.ethz.ch>
*/

/*
 * @file att_estimator_ekf_params.h
 *
 * Parameters for EKF filter
 */

#include <systemlib/param/param.h>

struct att_estimator_ekf_params {
	float r[9];
	float q[12];
	float roll_off;
	float pitch_off;
	float yaw_off;
};

struct att_estimator_ekf_param_handles {
	param_t r0, r1, r2, r3;
	param_t q0, q1, q2, q3, q4;
	param_t roll_off, pitch_off, yaw_off;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct att_estimator_ekf_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct att_estimator_ekf_param_handles *h, struct att_estimator_ekf_params *p);
