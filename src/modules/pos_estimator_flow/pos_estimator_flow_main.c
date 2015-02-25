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

/**
 * @file pos_estimator_flow_main.c
 *
 * Optical drone position estimator
 * Take Optical flow data, sonar sensor data to estimate Drone's current 
 * 		Local position data: x, y, z, vx, vy, vz, 
 *		Bodyframe velocity: filteredflow_vx, filteredflow_vy, filteredflow_vz
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <systemlib/perf_counter.h>
#include <poll.h>

#include <mavlink/mavlink_log.h>

#include "pos_estimator_flow_params.h"

__EXPORT int pos_estimator_flow_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */



int pos_estimator_flow_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int pos_estimator_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("drone position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

	
		thread_should_exit = false;
		daemon_task = task_spawn_cmd("pos_estimator_flow",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 5,
					 4096,
					 pos_estimator_flow_thread_main,
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
			printf("\tdrone position estimator is running\n");
		else
			printf("\tdrone position estimator not started\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int pos_estimator_flow_thread_main(int argc, char *argv[])
{
	/* welcome user */
	static int mavlink_fd; /* For mavlink debug */
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	thread_running = true;
	printf("[pos_estimator_flow] starting\n");
	mavlink_log_info(mavlink_fd, "[pe] started");

	/* rotation matrix for transformation of optical flow speed vectors */
	static const int8_t rotM_flow_sensor[3][3] =   {{  0, -1, 0 },
													{ 1, 0, 0 },
													{  0, 0, 1 }}; // 90deg rotated
	const float time_scale = powf(10.0f,-6.0f);
	static float speed[3] = {0.0f, 0.0f, 0.0f};
	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	static float global_speed[3] = {0.0f, 0.0f, 0.0f};
	static uint32_t counter = 0;
	static uint64_t time_last_flow = 0; // in ms
	static float dt = 0.0f; // seconds
	static float sonar_last = 0.0f;
	static bool sonar_valid = false;
	static float sonar_lp = 0.0f;

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));

	/* subscribe to parameter changes */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to armed topic */
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* subscribe to safety topic */
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	/* subscribe to attitude */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* subscribe to attitude setpoint */
	int vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

	/* subscribe to optical flow*/
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* init local position and filtered flow struct */
	struct vehicle_local_position_s local_pos = {
			.x = 0.0f,
			.y = 0.0f,
			.z = 0.0f,
			.vx = 0.0f,
			.vy = 0.0f,
			.vz = 0.0f
	};
	struct filtered_bottom_flow_s filtered_flow = {
			.sumx = 0.0f,
			.sumy = 0.0f,
			.vx = 0.0f,
			.vy = 0.0f
	};

	/* advert pub messages */
	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
	orb_advert_t filtered_flow_pub = orb_advertise(ORB_ID(filtered_bottom_flow), &filtered_flow);

	/* vehicle flying status parameters */
	bool sensors_ready = false;

	/* parameters init*/
	struct pos_estimator_flow_params params;
	struct pos_estimator_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "pos_estimator_flow_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "pos_estimator_flow_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "pos_estimator_flow_err");

	while (!thread_should_exit)
	{

		if (sensors_ready)
		{
			/*This runs at the rate of the sensors */
			struct pollfd fds[2] = {
					{ .fd = optical_flow_sub, .events = POLLIN },
					{ .fd = parameter_update_sub,   .events = POLLIN }
			};

			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			}
			else if (ret == 0)
			{
				/* no return value, ignore */
//				printf("[flow position estimator] no bottom flow.\n");
			}
			else
			{

				/* parameter update available? */
				if (fds[1].revents & POLLIN)
				{
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[pos_estimator_flow] parameters updated.\n");
				}

				/* only if flow data changed */
				if (fds[0].revents & POLLIN)
				{
					perf_begin(mc_loop_perf);

					orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);
					/* got flow, updating attitude and status as well */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);

					/* vehicle state estimation */
					float sonar_new = flow.ground_distance_m;

					/* set liftoff boolean
					 * -> at bottom sonar sometimes does not work correctly, and has to be calibrated (distance higher than 0.3m)
					 * -> accept sonar measurements after reaching calibration distance (values between 0.3m and 1.0m for some time)
					 * -> minimum sonar value 0.3m
					 */

					/* calc dt between flow timestamps */
					/* ignore first flow msg */
					if(time_last_flow == 0)
					{
						time_last_flow = flow.timestamp;
						continue;
					}
					dt = (float)(flow.timestamp - time_last_flow) * time_scale ;
					time_last_flow = flow.timestamp;

					/* only make position update if vehicle is lift off or DEBUG is activated*/
					/* WARNING: changed to start estimate asap... */
					if (armed.armed || params.debug) {
						/* copy flow */
						flow_speed[0] = flow.flow_comp_x_m;
						flow_speed[1] = flow.flow_comp_y_m;
						flow_speed[2] = 0.0f;

						/* convert to bodyframe velocity */
						for(uint8_t i = 0; i < 3; i++)
						{
							float sum = 0.0f;
							for(uint8_t j = 0; j < 3; j++)
								sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];

							speed[i] = sum;
						}

						/* update filtered flow */
						filtered_flow.sumx = filtered_flow.sumx + speed[0] * dt;
						filtered_flow.sumy = filtered_flow.sumy + speed[1] * dt;
						filtered_flow.vx = speed[0];
						filtered_flow.vy = speed[1];

						// TODO add yaw rotation correction (with distance to vehicle zero)

						/* convert to globalframe velocity
						 * -> local position is currently not used for position control
						 */
						for(uint8_t i = 0; i < 3; i++)
						{
							float sum = 0.0f;
							for(uint8_t j = 0; j < 3; j++)
								sum = sum + speed[j] * att.R[i][j];

							global_speed[i] = sum;
						}

						//local_pos.x = local_pos.x + global_speed[0] * dt;
						//local_pos.y = local_pos.y + global_speed[1] * dt;
						//local_pos.vx = global_speed[0];
						//local_pos.vy = global_speed[1];
						// WARNING: currently using filtered_flow velocity
						local_pos.x = local_pos.x + filtered_flow.vx * dt;
						local_pos.y = local_pos.y + filtered_flow.vy * dt;						
						local_pos.vx = filtered_flow.vx;
						local_pos.vy = filtered_flow.vy;
						local_pos.xy_valid = true;
						local_pos.v_xy_valid = true;

					} else {
						// If vehicle is not armed
						// set speed to zero and let position as it is
						filtered_flow.vx = 0;
						filtered_flow.vy = 0;
						local_pos.vx = 0;
						local_pos.vy = 0;
						local_pos.xy_valid = false;
						local_pos.v_xy_valid = false;
					}

					// Now filtering ground distance
					if (!armed.armed) {
						// Vehicle has not liftoff or is not armed
						//mavlink_log_info(mavlink_fd, "[fpe] not possible to fly");
						sonar_valid = false;
						local_pos.z = 0.0f;
						local_pos.z_valid = false;

					} else {
						// Vehicle  is armed
						sonar_valid = true;
					}

					if (sonar_valid || params.debug) {
						// Low pass filter for sonar sensor
						// if new value or with sonar update frequency
						if (sonar_new != sonar_last || counter % 10 == 0) {
							sonar_lp = 0.05f * sonar_new + 0.95f * sonar_lp;
							sonar_last = sonar_new;
						}

						float height_diff = sonar_new - sonar_lp;

						/* if over 1/2m spike follow lowpass */
						if (height_diff < -params.sonar_lower_lp_threshold || height_diff > params.sonar_upper_lp_threshold) {
							local_pos.z = -sonar_lp;

						} else {
							local_pos.z = -sonar_new;
						}

						local_pos.z_valid = true;
					}

					
					//mavlink_log_info(mavlink_fd, "[PE] vx:%0.2f,vy:%0.2f,sumx:%0.2f,sumy:%0.2f", 
					//		filtered_flow.vx, filtered_flow.vy, filtered_flow.sumx, filtered_flow.sumy);

					filtered_flow.timestamp = hrt_absolute_time();
					local_pos.timestamp = hrt_absolute_time();

					/* publish local position */
					if(isfinite(local_pos.x) && isfinite(local_pos.y) && isfinite(local_pos.z)
							&& isfinite(local_pos.vx) && isfinite(local_pos.vy))
					{
						//mavlink_log_info(mavlink_fd, "[fpe] sonar_new:%.2f,sonar_lp:%.2f,sonar_last:%.2f", sonar_new, sonar_lp, sonar_last);
						orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);
					}

					/* publish filtered flow */
					if(isfinite(filtered_flow.sumx) && isfinite(filtered_flow.sumy) && isfinite(filtered_flow.vx) && isfinite(filtered_flow.vy))
					{
						orb_publish(ORB_ID(filtered_bottom_flow), filtered_flow_pub, &filtered_flow);
					}

					/* measure in what intervals the position estimator runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);

				}
			}

		}
		else
		{
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a attitude message, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
				printf("[pos_estimator_flow] no attitude received.\n");
			}
			else
			{
				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[pos_estimator_flow] initialized.\n");
				}
			}
		}

		counter++;
	}

	printf("[pos_estimator_flow] exiting.\n");
	thread_running = false;

	close(vehicle_attitude_setpoint_sub);
	close(vehicle_attitude_sub);
	close(armed_sub);
	close(control_mode_sub);
	close(parameter_update_sub);
	close(optical_flow_sub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}


