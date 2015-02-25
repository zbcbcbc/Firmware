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


/**
 * @file pos_control_pid.c
 *
 * Optical drone position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <uORB/topics/vehicle_bodyframe_speed_setpoint.h>
#include <uORB/topics/filtered_bottom_flow.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>

#include "pos_control_pid_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int pos_control_pid_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int pos_control_pid_thread_main(int argc, char *argv[]);


/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int pos_control_pid_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("pos_control_pid already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("pos_control_pid",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 6,
					 4096,
					 pos_control_pid_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
		if (thread_running)
			printf("\tpos_control_pid is running\n");
		else
			printf("\tpos_control_pid not started\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
pos_control_pid_thread_main(int argc, char *argv[])
{
	/* welcome user */
	thread_running = true;
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[PC] started");

	uint32_t counter = 0;
	const float time_scale = powf(10.0f,-6.0f);

	/* structures */
	struct actuator_armed_s armed; // armed
	memset(&armed, 0, sizeof(armed));

	struct vehicle_control_mode_s control_mode; // control mode
	memset(&control_mode, 0, sizeof(control_mode));

	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));


	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));

	struct filtered_bottom_flow_s filtered_flow;
	memset(&filtered_flow, 0, sizeof(filtered_flow));

	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));

	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));




	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int filtered_bottom_flow_sub = orb_subscribe(ORB_ID(filtered_bottom_flow));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	/* publish attitude set point */
	orb_advert_t att_sp_pub;
	bool attitude_setpoint_adverted = false; // lazy advertize

	/* parameters init*/
	struct pos_control_pid_params params;
	struct pos_control_pid_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);


	/* init flow sum setpoint */
	float flow_sp_sumx = 0.0f;
	float flow_sp_sumy = 0.0f;

	/* init yaw setpoint */
	float yaw_sp = 0.0f;


	/* height controller states */
	bool standby_phase = true;
	bool landing_phase = false;
	bool operation_phase = false;

	/* states */
	float integrated_h_error = 0.0f;
	float last_local_pos_z = 0.0f;
	bool update_flow_sp_sumx = false;
	bool update_flow_sp_sumy = false;
	uint64_t local_ref_timestamp = 0; //TODO: what is this?
	uint64_t last_time = 0.0f;
	float dt = 0.0f; // s

	/* flags */
	bool update_position_sp = false;
	bool global_pos_sp_valid = false;
	bool local_pos_sp_valid = false;
	bool reset_man_sp_z = true;
	bool reset_man_sp_xy = true;
	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool was_armed = false;
	bool reset_auto_sp_xy = true;
	bool reset_auto_sp_z = true;
	bool reset_takeoff_sp = true;


	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "pos_control_pid_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "pos_control_pid_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "pos_control_pid_err");

	static bool sensors_ready = false;

	while (!thread_should_exit) {
		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready) {
			/* polling */
			struct pollfd fds[2] = {
				{ .fd = filtered_bottom_flow_sub, .events = POLLIN }, // positions from estimator
				{ .fd = parameter_update_sub,   .events = POLLIN }

			};

			/* wait for a position update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			} else if (ret == 0) {
				/* no return value, ignore */
//				printf("[flow position control] no filtered flow updates\n");
			} else {
				/* parameter update available? */
				if (fds[1].revents & POLLIN) {
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					mavlink_log_info(mavlink_fd,"[PC] parameters updated.");
				}

				
				if (fds[0].revents & POLLIN) {
					/* Position/Speed changed, update control output */

					//mavlink_log_info(mavlink_fd, "[PC] getting sth...");
					perf_begin(mc_loop_perf);
					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of filtered bottom flow */
					orb_copy(ORB_ID(filtered_bottom_flow), filtered_bottom_flow_sub, &filtered_flow);
					/* get a local copy of local position */
					orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
					/* get a local copy of control mode */
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
					/* get a local copy of local position sp */
					//orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);

					/*	
					if (control_mode.flag_control_altitude_enabled || 
							control_mode.flag_control_velocity_enabled || 
							control_mode.flag_control_position_enabled) */

					//mavlink_log_info(mavlink_fd, "[PC] copy complete...");

					struct vehicle_bodyframe_speed_setpoint_s speed_sp;
					memset(&speed_sp, 0, sizeof(speed_sp));

					/* update flow sum setpoint */
					if (update_flow_sp_sumx) {
						flow_sp_sumx = filtered_flow.sumx;
						update_flow_sp_sumx = false;
					}
					if (update_flow_sp_sumy) {
						flow_sp_sumy = filtered_flow.sumy;
						update_flow_sp_sumy = false;
					} 

					/* calc new bodyframe speed setpoints */
					float speed_body_x = (flow_sp_sumx - filtered_flow.sumx) * params.pos_p - filtered_flow.vx * params.pos_d;
					float speed_body_y = (flow_sp_sumy - filtered_flow.sumy) * params.pos_p - filtered_flow.vy * params.pos_d;
					float speed_limit_height_factor = params.height_min; // the settings are for 1 meter

					//mavlink_log_info(mavlink_fd, "[PC] body_x:%0.2f, body_y:%0.2f", speed_body_x, speed_body_y);

					//if (control_mode.flag_control_manual_enabled) {	
					// Hack: right now manual control mode only


					if (was_armed && !armed.armed) {
						standby_phase = true;
						operation_phase = false;
						landing_phase = false;
						integrated_h_error = 0;
						was_armed = false;
						mavlink_log_info(mavlink_fd, "[PC] changed from armed to not armed");
					} else if (!was_armed && armed.armed) {
						was_armed = true;
						standby_phase = true;
						operation_phase = false;
						landing_phase = false;
						integrated_h_error = 0;
						mavlink_log_info(mavlink_fd, "[PC] changed from not armed to armed");
					}

					//mavlink_log_info(mavlink_fd, "[PC] armed:%d,arm_flag:%d", armed.armed, control_mode.flag_armed);


					/* Manual Control Mode, Manual mode has priority than Autonomous Mode */
					//mavlink_log_info(mavlink_fd, "[PC] Manual control mode...");
					/* get a local copy of manual setpoint */
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
						

					float manual_pitch = manual.pitch / params.rc_scale_pitch; // 0 to 1
					float manual_roll = manual.roll / params.rc_scale_roll; // 0 to 1
					float manual_yaw = manual.yaw / params.rc_scale_yaw; // -1 to 1


					/* calc dt */
					if(last_time == 0) {
						last_time = hrt_absolute_time();
						continue;
					}

					dt = ((float) (hrt_absolute_time() - last_time)) * time_scale;
					last_time = hrt_absolute_time();


					/* Set speed setpoint with manual_pitch and manual_roll */
					if(isfinite(manual_pitch) && isfinite(manual_roll)) {
						if(fabsf(manual_pitch) > params.manual_threshold) {
							speed_body_x = -manual_pitch * params.limit_speed_x * speed_limit_height_factor;
							update_flow_sp_sumx = true;
						}

						if(fabsf(manual_roll) > params.manual_threshold) {
							speed_body_y = manual_roll * params.limit_speed_y * speed_limit_height_factor;
							update_flow_sp_sumy = true;
						}
					}

					/* saturate speed setpoints */

					if(speed_body_x > params.limit_speed_x * speed_limit_height_factor)
						speed_sp.vx = params.limit_speed_x * speed_limit_height_factor;
					else if(speed_body_x < -params.limit_speed_x * speed_limit_height_factor)
						speed_sp.vx = -params.limit_speed_x * speed_limit_height_factor;
					else 
						speed_sp.vx = speed_body_x;

					if(speed_body_y > params.limit_speed_y * speed_limit_height_factor)
						speed_sp.vy = params.limit_speed_y * speed_limit_height_factor;
					else if(speed_body_y < -params.limit_speed_y * speed_limit_height_factor)
						speed_sp.vy = -params.limit_speed_y * speed_limit_height_factor;
					else
						speed_sp.vy = speed_body_y;
						
					/* manual yaw change */
					if(isfinite(manual_yaw) && isfinite(manual.throttle)) {
						if(fabsf(manual_yaw) > params.manual_threshold && manual.throttle > 0.2f) {
							yaw_sp += manual_yaw * params.limit_yaw_step;
							/* modulo for rotation -pi +pi */
							if(yaw_sp < -M_PI_F) yaw_sp = yaw_sp + M_TWOPI_F;
							else if(yaw_sp > M_PI_F) yaw_sp = yaw_sp - M_TWOPI_F;
						}
					}

					/* forward yaw setpoint */
					speed_sp.yaw_sp = yaw_sp;


					/* manual height control
					 * 0-20%: thrust linear down
					 * 20%-40%: down
					 * 40%-60%: stabilize altitude
					 * 60-100%: up
						 */
					float thrust_control = 0.0f;
					float height_sp = local_pos.z;




					//mavlink_log_info(mavlink_fd, "[PC] 1:%d,2:%d,3:%d,4:%d,thr:%0.2f", 
					//		ground_phase, takeoff_phase, loiter_phase, landing_phase, manual.throttle);

					if (isfinite(manual.throttle)) {

						if ( (operation_phase && manual.throttle >= -0.2f) || (standby_phase && manual.throttle > 0.2f)) {
								
							operation_phase = true;
							standby_phase = false;
							height_sp = params.height_min;

							//mavlink_log_info(mavlink_fd, "[PC] 1 thr:%.2f,z:%0.2f", thrust_control, local_pos.z);

							/* calc new thrust with PID */
							float height_error = (local_pos.z - (-height_sp));

							/* instead of speed limitation, limit height error (downwards) */
							if(height_error > params.limit_height_error)
								height_error = params.limit_height_error;
							else if(height_error < -params.limit_height_error)
								height_error = -params.limit_height_error;


								/* Integration saturation */
							float height_speed = last_local_pos_z - local_pos.z;
							float thrust_p = height_error * params.height_p;
							float thrust_diff =  - height_speed * params.height_d;

							integrated_h_error += height_error;
							float integrated_thrust_addition = integrated_h_error * params.height_i;

							if(integrated_thrust_addition > params.limit_thrust_int)
								integrated_thrust_addition = params.limit_thrust_int;
							if(integrated_thrust_addition < -params.limit_thrust_int)
								integrated_thrust_addition = -params.limit_thrust_int;

							thrust_control = params.thrust_feedforward + thrust_p + thrust_diff + integrated_thrust_addition;

								/* add attitude component
									* F = Fz / (cos(pitch)*cos(roll)) -> can be found in rotM
									*/
//									// TODO problem with attitude
//									if (att.R_valid && att.R[2][2] > 0)
//										thrust_control = thrust_control / att.R[2][2];

							/* set thrust lower limit */
							if(thrust_control < params.limit_thrust_lower)
								thrust_control = params.limit_thrust_lower;
								

							mavlink_log_info(mavlink_fd, "[PC]2 tr:%.2f,p:%0.2f,i:%0.2f,z:%0.2f,sp:%0.2f", 
									thrust_control, thrust_p, integrated_thrust_addition, local_pos.z, -height_sp);


						} else if (landing_phase || (operation_phase && manual.throttle < -0.2f)) {
							// Landing 
								
							/* landing initialization */

							/* set current height as setpoint to avoid steps */
							landing_phase = true;
							operation_phase = false;

							height_sp -= 0.01;

							float height_error = (local_pos.z - (-height_sp));

							if(height_error > params.limit_height_error)
								height_error = params.limit_height_error;
							else if(height_error < -params.limit_height_error)
								height_error = -params.limit_height_error;

							float height_speed = last_local_pos_z - local_pos.z;
							float thrust_p = height_error * params.height_p;
							float thrust_diff =  - height_speed * params.height_d;

							integrated_h_error += height_error;
							float integrated_thrust_addition = integrated_h_error * params.height_i;

							if(integrated_thrust_addition > params.limit_thrust_int)
								integrated_thrust_addition = params.limit_thrust_int;
							if(integrated_thrust_addition < -params.limit_thrust_int)
								integrated_thrust_addition = -params.limit_thrust_int;

							/* update height setpoint if needed*/

							/* instead of speed limitation, limit height error (downwards) */


							thrust_control = params.thrust_feedforward + thrust_p + thrust_diff + integrated_thrust_addition;

							/* add attitude component
								* F = Fz / (cos(pitch)*cos(roll)) -> can be found in rotM
								*/
//								// TODO problem with attitude
//								if (att.R_valid && att.R[2][2] > 0)
//									thrust_control = thrust_control / att.R[2][2];

							/* set thrust lower limit */
							if(thrust_control < params.limit_thrust_lower)
								thrust_control = params.limit_thrust_lower;

							/* assume ground position here */
							if (-local_pos.z < 0.3f || height_sp <= 0.3f) {
								/* reset integral if on ground */
								integrated_h_error = 0.0f;
								/* switch to start phase */
								standby_phase = true;
								landing_phase = false;
							}

							mavlink_log_info(mavlink_fd, "[PC]3 tr:%.2f,p:%0.2f,i:%0.2f,z:%0.2f,sp:%0.2f", 
									thrust_control, thrust_p, integrated_thrust_addition, local_pos.z, -height_sp);

						}
					}

					/* set thrust upper limit */
					if(thrust_control > params.limit_thrust_upper)
						thrust_control = params.limit_thrust_upper;
						
					/* store actual height for speed estimation */
					last_local_pos_z = local_pos.z;

					speed_sp.thrust_sp =  thrust_control; //manual.throttle;
					speed_sp.timestamp = hrt_absolute_time();

					//mavlink_log_info(mavlink_fd, "[PC] sp vx:%0.2f,vy:%0.2f,thr:%0.2f", speed_sp.vx, speed_sp.vy, speed_sp.thrust_sp);


						
					

					//mavlink_log_info(mavlink_fd, "[dpc]sp_vx:%0.2f,vy:%0.2f;f_vx:%0.2f,f_vy:%0.2f", 
					//					speed_sp.vx, speed_sp.vy, filtered_flow.vx, filtered_flow.vy);

					/* calculate new roll/pitch based on velocity set point using p controller */
					float pitch_body = -(speed_sp.vx - filtered_flow.vx) * params.speed_p;
					float roll_body  =  (speed_sp.vy - filtered_flow.vy) * params.speed_p;

					//mavlink_log_info(mavlink_fd, "[dpc] pitch_body:%0.2f, roll_body:%0.2f", pitch_body, roll_body);

					/* roll and pitch saturation */
					if((pitch_body <= params.limit_pitch) && (pitch_body >= -params.limit_pitch)) {
						att_sp.pitch_body = pitch_body;
					} else {
						if(pitch_body > params.limit_pitch) att_sp.pitch_body = params.limit_pitch;
						if(pitch_body < -params.limit_pitch) att_sp.pitch_body = -params.limit_pitch;
					}

					if((roll_body <= params.limit_roll) && (roll_body >= -params.limit_roll)) {
						att_sp.roll_body = roll_body;
					} else {
						if(roll_body > params.limit_roll)
							att_sp.roll_body = params.limit_roll;
						if(roll_body < -params.limit_roll)
							att_sp.roll_body = -params.limit_roll;
					}

					/* set yaw setpoint forward*/
					att_sp.yaw_body = speed_sp.yaw_sp;

					/* add trim from parameters */
					att_sp.roll_body = att_sp.roll_body + params.trim_roll;
					att_sp.pitch_body = att_sp.pitch_body + params.trim_pitch;

					att_sp.thrust = speed_sp.thrust_sp;
					att_sp.timestamp = hrt_absolute_time();


					//mavlink_log_info(mavlink_fd, "[dpc]r_sp:%0.2f,p_sp:%0.2f,t_sp:%0.2f", att_sp.roll_body, att_sp.pitch_body, att_sp.thrust);


					/* lazy publish new attitude setpoint */
					if(isfinite(att_sp.pitch_body) && isfinite(att_sp.roll_body) 
							&& isfinite(att_sp.yaw_body) && isfinite(att_sp.thrust)) 
					{
						if (attitude_setpoint_adverted) {
								orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
						} else {
							att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
							attitude_setpoint_adverted = true;
						}
					}

				} //end if position/speed change
			} // end if retrived update

			counter++;

		} else {
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a flow msg, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			} else if (ret == 0) {
				/* no return value, ignore */
				mavlink_log_info(mavlink_fd,"[PC] no attitude received.\n");
			} else {
				if (fds[0].revents & POLLIN) {
					sensors_ready = true;
					mavlink_log_info(mavlink_fd,"[PC] initialized, yeah~\n");
				}
			}
		}
	}

	mavlink_log_info(mavlink_fd,"[pc] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(local_pos_sub);
	close(local_pos_sp_sub);
	close(armed_sub);
	close(control_mode_sub);
	close(manual_control_setpoint_sub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}



