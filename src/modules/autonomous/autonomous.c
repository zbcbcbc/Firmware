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
 *   Author: Bicheng Zhang <viczhang1990@yahoo.com>
 *   		 
 */


/**
 * @file autonomous.c
 *
 * Autonomous logic 
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <math.h>
#include <sys/prctl.h>
#include <uORB/uORB.h>
#include <mavlink/mavlink_log.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>


#include "autonomous_params.h"


/* Run various autonomous algorithms and basic safe protection */

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */


__EXPORT int autonomous_main(int argc, char *argv[]);

/**
 * Mainloop of autonomous algorithm.
 */
static int autonomous_thread_main(int argc, char *argv[]);


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

int autonomous_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("autonomous already running\n");
			// this is not an error 
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("autonomous",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 2048,
					 autonomous_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);

		//printf("daemon task creation finished, id is:%d\n", deamon_task);
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
			printf("\tautonomous is running\n");
		else
			printf("\tautonomous not started\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


static int
autonomous_thread_main(int argc, char *argv[])
{

	// welcome user 
	thread_running = true;
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[auto] started");

	perf_counter_t auto_loop_perf = perf_alloc(PC_ELAPSED, "auto_loop_perf");
	perf_counter_t auto_err_perf = perf_alloc(PC_COUNT, "autonomous_err");

	// parameters init
	struct autonomous_params params;
	struct autonomous_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	// publish attitude set point
	orb_advert_t local_pos_sp_pub;
	bool local_pos_sp_adverted = false; // lazy advertize

	// Initialize data structs 
	struct actuator_armed_s armed; // armed
	memset(&armed, 0, sizeof(armed));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));

	// Set up subscriptions 
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	// autonomous states
	bool standby_phase = true;
	bool landing_phase = false;
	bool operation_phase = false;
	bool was_armed = false;

	float height_sp = 0; // local_pos_sp = 0 means no throttle control



	printf("autonomous initialized..\n\n");
	fflush(stdout);

	
	while (!thread_should_exit) {
		
		// polling 
		struct pollfd fds[1] = {
			{ .fd = manual_control_setpoint_sub, .events = POLLIN }
		};

		// wait for a position update, check for exit condition every 500 ms

		//printf("autonomous polling...\n\n");
		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			// poll error, count it in perf 
			perf_count(auto_err_perf);
		} else if (ret == 0) {
			// no return value, ignore 
			//			printf("[flow position control] no filtered flow updates\n");
		} else {

			perf_begin(auto_loop_perf);

			// Get a local copy 
			// TODO: update if and only if the values are updated
			orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
			orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);

			if (was_armed && !armed.armed) {
				standby_phase = true;
				operation_phase = false;
				landing_phase = false;
				was_armed = false;
				mavlink_log_info(mavlink_fd, "[auto] changed from armed to not armed");
			} else if (!was_armed && armed.armed) {
				was_armed = true;
				standby_phase = true;
				operation_phase = false;
				landing_phase = false;
				mavlink_log_info(mavlink_fd, "[auto] changed from not armed to armed");
			}

			struct vehicle_local_position_s local_pos;
			orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);


			float manual_pitch = manual.pitch / params.rc_scale_pitch; // 0 to 1
			float manual_roll = manual.roll / params.rc_scale_roll; // 0 to 1
			float manual_yaw = manual.yaw / params.rc_scale_yaw; // -1 to 1


			if (isfinite(manual.throttle)) {
				// manual control step in
				if ((operation_phase && manual.throttle >= -0.2f) || (standby_phase && manual.throttle > 0.2f)) {	
					operation_phase = true;
					landing_phase = false;
					standby_phase = false;
					height_sp = 0.6;

					//mavlink_log_info(mavlink_fd, "[auto] triggered take off to 0.6 height");

				} else if (landing_phase || (operation_phase && manual.throttle < -0.2f)) {
					// Landing 
					// set current height as setpoint to avoid steps 
					landing_phase = true;
					operation_phase = false;
					height_sp -= 0.05;

					mavlink_log_info(mavlink_fd, "[auto] triggered to start to land, height_sp:%0.3f", height_sp);

					if (-local_pos.z < 0.3f || height_sp <= 0.3f) {
						// assume ground position 
						standby_phase = true;
						landing_phase = false;

						//mavlink_log_info(mavlink_fd, "[auto] landing succeed");
					}
				}

				local_pos_sp.z = - height_sp;

				if (local_pos_sp_adverted) {
					orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);
				} else {
					local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
					local_pos_sp_adverted = true;
				}

			}// endof isfinite manual control

			local_pos_sp.x = local_pos.x;
			local_pos_sp.y = local_pos.y;

		}
		
	}// endof while

	mavlink_log_info(mavlink_fd, "[auto] ending now...\n");

	thread_running = false;

	close(armed_sub);
	close(manual_control_setpoint_sub);
	close(local_pos_sub);
	close(local_pos_sp_pub);

	perf_print_counter(auto_loop_perf);
	perf_free(auto_loop_perf);

	fflush(stdout);
	return 0;
}


