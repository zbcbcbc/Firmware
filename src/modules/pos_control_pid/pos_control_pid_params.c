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
 * @file pos_control_pid_params.c
 */

#include "pos_control_pid_params.h"

/* controller parameters */

// Position control P gain
PARAM_DEFINE_FLOAT(PC_PID_POS_P, 3.0f);
// Position control D / damping gain
PARAM_DEFINE_FLOAT(PC_PID_POS_I, 0.0f);
// Position control I  gain
PARAM_DEFINE_FLOAT(PC_PID_POS_D, 0.0f);

PARAM_DEFINE_FLOAT(PC_PID_L_POS_ERR, 10.0f); //TODO: temporary
PARAM_DEFINE_FLOAT(PC_PID_L_POS_INT, 10.0f);

// Velocity control P gain
PARAM_DEFINE_FLOAT(PC_PID_VEL_P, 3.0f);
// Velocity control D / damping gain
PARAM_DEFINE_FLOAT(PC_PID_VEL_I, 0.0f);
// Velocity control I  gain
PARAM_DEFINE_FLOAT(PC_PID_VEL_D, 0.0f);

PARAM_DEFINE_FLOAT(PC_PID_L_VEL_ERR, 10.0f); //TODO: temporary
PARAM_DEFINE_FLOAT(PC_PID_L_VEL_INT, 10.0f); //TODO: temporary
// Altitude control P gain
PARAM_DEFINE_FLOAT(PC_PID_HEIGHT_P, 0.15f);
// Altitude control I (integrator) gain
PARAM_DEFINE_FLOAT(PC_PID_HEIGHT_I, 0.0001f);
// Altitude control D gain
PARAM_DEFINE_FLOAT(PC_PID_HEIGHT_D, 0.30f);

PARAM_DEFINE_FLOAT(PC_PID_L_HEIGHT_ERR, 0.5f);
PARAM_DEFINE_FLOAT(PC_PID_L_HEIGHT_INT, 1.0f);
// Altitude control rate limiter
// Altitude control minimum altitude
PARAM_DEFINE_FLOAT(PC_PID_HEIGHT_MIN, 0.6f);
// Altitude control maximum altitude (higher than 1.5m is untested)
PARAM_DEFINE_FLOAT(PC_PID_HEIGHT_MAX, 1.2f);
// Altitude control feed forward throttle - adjust to the
// throttle position (0..1) where the copter hovers in manual flight
PARAM_DEFINE_FLOAT(PC_PID_T_FFWD, 0.69f); // adjust this before flight


//PARAM_DEFINE_FLOAT(PC_PID_L_TH_I, 0.05f);
PARAM_DEFINE_FLOAT(PC_PID_L_TH_U, 0.8f);
PARAM_DEFINE_FLOAT(PC_PID_L_TH_L, 0.4f);
//PARAM_DEFINE_FLOAT(PC_PID_L_YAW_STEP, 0.03f);
//PARAM_DEFINE_FLOAT(PC_PID_MAN_THR, 0.1f);

// Speed paramed defined
//PARAM_DEFINE_FLOAT(PC_PID_S_P, 0.20f);
PARAM_DEFINE_FLOAT(PC_PID_L_PITCH, 0.4f);
PARAM_DEFINE_FLOAT(PC_PID_L_ROLL, 0.4f);


int parameters_init(struct pos_control_pid_param_handles *h)
{
	/* PID parameters */
	h->pos_p	 			=	param_find("PC_PID_POS_P");
	h->pos_d 				=	param_find("PC_PID_POS_D");
	h->pos_i 				=	param_find("PC_PID_POS_I");
	h->vel_p				=	param_find("PC_PID_VEL_P");
	h->vel_i				=	param_find("PC_PID_VEL_I");
	h->vel_d				=	param_find("PC_PID_VEL_D");
	h->height_p 			=	param_find("PC_PID_HEIGHT_P");
	h->height_i 			=	param_find("PC_PID_HEIGHT_I");
	h->height_d 			=	param_find("PC_PID_HEIGHT_D");
	h->height_min			=	param_find("PC_PID_HEIGHT_MIN");
	h->height_max			=	param_find("PC_PID_HEIGHT_MAX");
	h->thrust_feedforward 	=	param_find("PC_PID_T_FFWD");
	h->limit_height_err		=	param_find("PC_PID_L_HEIGHT_ERR");
	h->limit_height_int		=	param_find("PC_PID_L_HEIGHT_INT");
	h->limit_pos_err		=	param_find("PC_PID_L_POS_ERR");
	h->limit_pos_int		=	param_find("PC_PID_L_POS_INT");
	h->limit_vel_err		=	param_find("PC_PID_L_VEL_ERR");
	h->limit_vel_int		=	param_find("PC_PID_L_VEL_INT");
	//h->limit_thrust_int 	=	param_find("PC_PID_L_TH_I");
	h->limit_thrust_upper 	=	param_find("PC_PID_L_TH_U");
	h->limit_thrust_lower 	=	param_find("PC_PID_L_TH_L");
	//h->limit_yaw_step		=	param_find("PC_PID_L_YAW_STEP");
	//h->manual_threshold 	=	param_find("PC_PID_MAN_THR");
	//h->rc_scale_pitch		=   param_find("RC_SCALE_PITCH");
	//h->rc_scale_roll		=   param_find("RC_SCALE_ROLL");
	//h->rc_scale_yaw			=   param_find("RC_SCALE_YAW");

	// speed params 
	//h->speed_p	 			=	param_find("PC_PID_S_P");
	h->limit_pitch 			=	param_find("PC_PID_L_PITCH");
	h->limit_roll 			=	param_find("PC_PID_L_ROLL");
	h->trim_roll 			=	param_find("TRIM_ROLL");
	h->trim_pitch 			=	param_find("TRIM_PITCH");


	// takeoff params
	//h->takeoff_alt = param_find("NAV_TAKEOFF_ALT");
	//h->takeoff_gap = param_find("NAV_TAKEOFF_GAP");

	return OK;
}

int parameters_update(const struct pos_control_pid_param_handles *h, struct pos_control_pid_params *p)
{
	param_get(h->pos_p, &(p->pos_p));
	param_get(h->pos_d, &(p->pos_d));
	param_get(h->pos_i, &(p->pos_i));
	param_get(h->limit_pos_int, &(p->limit_pos_int));
	param_get(h->limit_pos_err, &(p->limit_pos_err));

	param_get(h->vel_p, &(p->vel_p));
	param_get(h->vel_i, &(p->vel_i));
	param_get(h->vel_d, &(p->vel_d));
	param_get(h->limit_vel_err, &(p->limit_vel_err));
	param_get(h->limit_vel_int, &(p->limit_vel_int));

	param_get(h->height_p, &(p->height_p));
	param_get(h->height_i, &(p->height_i));
	param_get(h->height_d, &(p->height_d));
	param_get(h->limit_height_err, &(p->limit_height_err));
	param_get(h->limit_height_int, &(p->limit_height_int));

	//param_get(h->height_rate, &(p->height_rate));
	param_get(h->height_min, &(p->height_min));
	param_get(h->height_max, &(p->height_max));
	param_get(h->thrust_feedforward, &(p->thrust_feedforward));
	
	
	
	//param_get(h->limit_thrust_int, &(p->limit_thrust_int));
	param_get(h->limit_thrust_upper, &(p->limit_thrust_upper));
	param_get(h->limit_thrust_lower, &(p->limit_thrust_lower));
	//param_get(h->limit_yaw_step, &(p->limit_yaw_step));
	//param_get(h->manual_threshold, &(p->manual_threshold));
	//param_get(h->rc_scale_pitch, &(p->rc_scale_pitch));
	//param_get(h->rc_scale_roll, &(p->rc_scale_roll));
	//param_get(h->rc_scale_yaw, &(p->rc_scale_yaw));
	//param_get(h->takeoff_alt, &(p->takeoff_alt));
	//param_get(h->takeoff_gap, &(p->takeoff_gap));

	//param_get(h->speed_p, &(p->speed_p));
	param_get(h->limit_pitch, &(p->limit_pitch));
	param_get(h->limit_roll, &(p->limit_roll));
	param_get(h->trim_roll, &(p->trim_roll));
	param_get(h->trim_pitch, &(p->trim_pitch));

	return OK;
}
