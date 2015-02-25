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
 * @file pos_estimator_pid_params.c
 * 
 * Parameters for position estimator
 */

#include "pos_estimator_flow_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
PARAM_DEFINE_FLOAT(PE_FLOW_LO_THRUST, 0.4f);
PARAM_DEFINE_FLOAT(PE_FLOW_SONAR_LP_U, 0.5f);
PARAM_DEFINE_FLOAT(PE_FLOW_SONAR_LP_L, 0.2f);
PARAM_DEFINE_INT32(PE_FLOW_DEBUG, 0);


int parameters_init(struct pos_estimator_flow_param_handles *h)
{
	/* PID parameters */
	h->minimum_liftoff_thrust	=	param_find("PE_FLOW_LO_THRUST");
	h->sonar_upper_lp_threshold	=	param_find("PE_FLOW_SONAR_LP_U");
	h->sonar_lower_lp_threshold	=	param_find("PE_FLOW_SONAR_LP_L");
	h->debug					=	param_find("PE_FLOW_DEBUG");

	return OK;
}

int parameters_update(const struct pos_estimator_flow_param_handles *h, struct pos_estimator_flow_params *p)
{
	param_get(h->minimum_liftoff_thrust, &(p->minimum_liftoff_thrust));
	param_get(h->sonar_upper_lp_threshold, &(p->sonar_upper_lp_threshold));
	param_get(h->sonar_lower_lp_threshold, &(p->sonar_lower_lp_threshold));
	param_get(h->debug, &(p->debug));

	return OK;
}
