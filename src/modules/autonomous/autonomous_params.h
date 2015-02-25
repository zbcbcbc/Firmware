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


#include <systemlib/param/param.h>

struct autonomous_params {
	float rc_scale_pitch;
	float rc_scale_roll;
	float rc_scale_yaw;
};

struct autonomous_param_handles {
	param_t rc_scale_pitch;
	param_t rc_scale_roll;
	param_t rc_scale_yaw;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct autonomous_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct autonomous_param_handles *h, struct autonomous_params *p);