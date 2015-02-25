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
 * @file attitude_estimator_ekf_params.c
 *
 * Parameters for EKF filter
 */

#include "att_estimator_ekf_params.h"

/* Extended Kalman Filter covariances */

/* gyro process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q0, 1e-4f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q1, 0.08f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q2, 0.009f);
/* gyro offsets process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q3, 0.005f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q4, 0.0f);

/* gyro measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_R0, 0.0008f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_R1, 10000.0f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_R2, 1.0f);
/* accelerometer measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_R3, 0.0f);

/* offsets in roll, pitch and yaw of sensor plane and body */
PARAM_DEFINE_FLOAT(ATT_ROLL_OFF3, 0.0f);
PARAM_DEFINE_FLOAT(ATT_PITCH_OFF3, 0.0f);
PARAM_DEFINE_FLOAT(ATT_YAW_OFF3, 0.0f);

int parameters_init(struct att_estimator_ekf_param_handles *h)
{
	/* PID parameters */
	h->q0 	=	param_find("EKF_ATT_V3_Q0");
	h->q1 	=	param_find("EKF_ATT_V3_Q1");
	h->q2 	=	param_find("EKF_ATT_V3_Q2");
	h->q3 	=	param_find("EKF_ATT_V3_Q3");
	h->q4 	=	param_find("EKF_ATT_V3_Q4");

	h->r0 	=	param_find("EKF_ATT_V3_R0");
	h->r1 	=	param_find("EKF_ATT_V3_R1");
	h->r2 	=	param_find("EKF_ATT_V3_R2");
	h->r3 	=	param_find("EKF_ATT_V3_R3");

	h->roll_off  =	param_find("ATT_ROLL_OFF3");
	h->pitch_off =	param_find("ATT_PITCH_OFF3");
	h->yaw_off   =	param_find("ATT_YAW_OFF3");

	return OK;
}

int parameters_update(const struct att_estimator_ekf_param_handles *h, struct att_estimator_ekf_params *p)
{
	param_get(h->q0, &(p->q[0]));
	param_get(h->q1, &(p->q[1]));
	param_get(h->q2, &(p->q[2]));
	param_get(h->q3, &(p->q[3]));
	param_get(h->q4, &(p->q[4]));

	param_get(h->r0, &(p->r[0]));
	param_get(h->r1, &(p->r[1]));
	param_get(h->r2, &(p->r[2]));
	param_get(h->r3, &(p->r[3]));

	param_get(h->roll_off, &(p->roll_off));
	param_get(h->pitch_off, &(p->pitch_off));
	param_get(h->yaw_off, &(p->yaw_off));

	return OK;
}
