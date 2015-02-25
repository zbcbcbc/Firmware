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


#include "autonomous_params.h"

int parameters_init(struct autonomous_param_handles *h)
{
	h->rc_scale_pitch		=   param_find("RC_SCALE_PITCH");
	h->rc_scale_roll		=   param_find("RC_SCALE_ROLL");
	h->rc_scale_yaw			=   param_find("RC_SCALE_YAW");

	return OK;
}

int parameters_update(const struct autonomous_param_handles *h, struct autonomous_params *p)
{
	param_get(h->rc_scale_pitch, &(p->rc_scale_pitch));
	param_get(h->rc_scale_roll, &(p->rc_scale_roll));
	param_get(h->rc_scale_yaw, &(p->rc_scale_yaw));
	return OK;
}