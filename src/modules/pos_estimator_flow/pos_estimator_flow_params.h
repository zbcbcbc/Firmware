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
 * @file pos_estimator_pid_params.h
 * 
 * Parameters for position estimator
 */

#include <systemlib/param/param.h>

struct pos_estimator_flow_params {
	float minimum_liftoff_thrust;
	float sonar_upper_lp_threshold;
	float sonar_lower_lp_threshold;
	int debug;
};

struct pos_estimator_flow_param_handles {
	param_t minimum_liftoff_thrust;
	param_t sonar_upper_lp_threshold;
	param_t sonar_lower_lp_threshold;
	param_t debug;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct pos_estimator_flow_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct pos_estimator_flow_param_handles *h, struct pos_estimator_flow_params *p);
