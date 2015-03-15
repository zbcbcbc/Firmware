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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

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

/**
 * Saturate
 */
static void saturate(float *input, const float upperlimit, const float lowerlimit);

static void 
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

static void
saturate(float *input, const float upperlimit, const float lowerlimit)
{
	if ((*input) > upperlimit)
		*input = upperlimit;
	else if ((*input) < lowerlimit)
		*input = lowerlimit;
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

	const float time_scale = powf(10.0f,-6.0f);

	/* structures */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));

	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));

	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));

	struct actuator_armed_s armed; // armed
	memset(&armed, 0, sizeof(armed));

	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));


	/* subscribe to attitude, motor setpoints and system state */
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* publish attitude set point */
	orb_advert_t att_sp_pub;
	bool attitude_setpoint_adverted = false; // lazy advertize

	/* parameters init*/
	struct pos_control_pid_params params;
	struct pos_control_pid_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);


	/* init yaw setpoint */
	float yaw_sp = 0.0f;


	/* states */
	float last_local_pos_z = 0.0f;
	uint64_t last_time = 0.0f;
	float dt = 0.0f; // s

	// Integral variables
	float err_x_integral = 0.0f;
	float err_y_integral = 0.0f;
	float err_z_integral = 0.0f;

	float err_vx_integral = 0.0f;
	float err_vy_integral = 0.0f;
	float err_vz_integral = 0.0f;


	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "pos_control_pid_runtime");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "pos_control_pid_err");

	static bool sensors_ready = false;

	printf("[pos_controller_pid] starting\n");
	mavlink_log_info(mavlink_fd,"[pc] initialized\n");

	while (!thread_should_exit) {
		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready) {
			/* polling */
			struct pollfd fds[2] = {
				{ .fd = local_pos_sub, .events = POLLIN }, // positions from estimator
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
					// Position/Speed changed, update control output

					//calculate dt
					if(last_time == 0) {
						last_time = hrt_absolute_time();
						continue;
					}

					dt = ((float) (hrt_absolute_time() - last_time)) * time_scale;
					last_time = hrt_absolute_time();


					//mavlink_log_info(mavlink_fd, "[PC] getting sth...");
					perf_begin(mc_loop_perf);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of local position */
					orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
					/* get a local copy of local position setpoint */
					orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);

					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);

					/* manual control setpoint */
					//orb_check(manual_control_setpoint_sub, &updated);

					//if (updated) {
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
					//}

					float manual_pitch = manual.pitch / 3; // TODO: params.rc_scale_pitch 0 to 1
					float manual_roll = manual.roll / 3; // 0 to 1
					float manual_yaw = manual.yaw / 3; // -1 to 1

					if (!armed.armed) {
						//TODO: In non-armed state, reset all state persistent variables
						err_x_integral = err_vx_integral = 0.0f;
						err_y_integral = err_vy_integral = 0.0f;
						err_z_integral = err_vz_integral = 0.0f;

					} else {
						//TODO:
						// x position pid controller 
						float err_pos_x = local_pos_sp.x - local_pos.x;
						saturate(&err_pos_x, params.limit_pos_err, -params.limit_pos_err);
						err_x_integral += err_pos_x * dt;
						saturate(&err_x_integral, params.limit_pos_int, -params.limit_pos_int);
						float cntrl_x_sig = err_pos_x * params.pos_p + err_x_integral * params.pos_i + local_pos.vx * params.pos_d;
						float x_vel_err = cntrl_x_sig - local_pos.vx;
						saturate(&x_vel_err, params.limit_vel_err, -params.limit_vel_err);
						err_vx_integral += x_vel_err * dt;
						saturate(&err_vx_integral, params.limit_vel_int, -params.limit_vel_int);
						float pitch_control = -1 * (x_vel_err * params.vel_p + err_vx_integral * params.vel_i +
													local_pos.vx * params.vel_d);
						saturate(&pitch_control, params.limit_pitch, -params.limit_pitch);


						// y position pid controller 
						float err_pos_y = local_pos_sp.y - local_pos.y;
						saturate(&err_pos_y, params.limit_pos_err, -params.limit_pos_err);
						err_y_integral += err_pos_y * dt;
						saturate(&err_y_integral, params.limit_pos_int, -params.limit_pos_int);
						float cntrl_y_sig = err_pos_y * params.pos_p + err_y_integral + local_pos.vy * params.pos_d;
						float y_vel_err = cntrl_y_sig - local_pos.vy;
						err_vy_integral += y_vel_err * dt;
						saturate(&err_vy_integral, params.limit_pos_int, -params.limit_pos_int);
						float roll_control = (y_vel_err * params.vel_p - err_vy_integral * params.pos_i + local_pos.vy * params.pos_d);
						saturate(&roll_control, params.limit_roll, -params.limit_roll);
					

						// z position pid controller
						float err_pos_z = local_pos.z - local_pos_sp.z;
						saturate(&err_pos_z, params.limit_height_err, -params.limit_height_err);
						float thrust_p = err_pos_z * params.height_p;
						float thrust_d = (local_pos.z - last_local_pos_z) * params.height_d;
						err_z_integral += err_pos_z;
						saturate(&err_z_integral, params.limit_height_int, -params.limit_height_int);
						float thrust_i = err_z_integral * params.height_i;
						float thrust_control = params.thrust_feedforward + thrust_p + thrust_d + thrust_i;
						saturate(&thrust_control, params.limit_thrust_upper, params.limit_thrust_lower);


						//mavlink_log_info(mavlink_fd, "[PC]z:%.2f,z_sp:%.2ftr:%.2f",
							//		local_pos.z, local_pos_sp.z,  thrust_control);
						
						/* store actual height for speed estimation */
						last_local_pos_z = local_pos.z;


						//TODO: set yaw setpoint forward
						att_sp.yaw_body = 0;

						/* add trim from parameters */
						//TODO: too risky
						if (isfinite(manual_pitch)) {
							att_sp.pitch_body = manual_pitch;
						} else {
							att_sp.pitch_body = 0;
						}

						if (isfinite(manual_roll)) {
							att_sp.roll_body = manual_roll;
						} else {
							att_sp.roll_body = 0;
						}


						if (local_pos_sp.z != 0) att_sp.thrust = thrust_control;
						else att_sp.thrust = 0;
						att_sp.timestamp = hrt_absolute_time();


						mavlink_log_info(mavlink_fd, "[PC]r_sp:%0.2f,p_sp:%0.2f,t_sp:%0.2f", att_sp.roll_body, att_sp.pitch_body, att_sp.thrust);


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
					} //endif armed


				} //end if position/speed change
			} // end if retrived update


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
				}
			}
		}
	}

	mavlink_log_info(mavlink_fd,"[pc] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(armed_sub);
	close(vehicle_attitude_sub);
	close(local_pos_sub);
	close(local_pos_sp_sub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}



