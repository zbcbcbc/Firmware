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
 * @file att_control_pid_main.c
 *
 * Implementation of multirotor attitude control main loop.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @Developer Bicheng Zhang <zhang368@illinois.edu>
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
#include <getopt.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
//#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>

#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "att_control_pid.h"
#include "att_rate_control_pid.h"

__EXPORT int att_control_pid_main(int argc, char *argv[]);

static bool thread_should_exit;
static int att_control_pid_task;
static bool motor_test_mode = false;
static const float min_takeoff_throttle = 0.3f;
static const float yaw_deadzone = 0.01f;

static int
att_control_pid_thread_main(int argc, char *argv[])
{
	/* declare and safely initialize all structs */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct offboard_control_setpoint_s offboard_sp;
	memset(&offboard_sp, 0, sizeof(offboard_sp));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	//struct manual_control_setpoint_s manual;
	//memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* subscribe */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int offboard_control_setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	int vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	//int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	orb_advert_t rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* register the perf counter */
	perf_counter_t att_control_pid_loop_perf = perf_alloc(PC_ELAPSED, "att_control_pid_runtime");
	perf_counter_t att_control_pid_interval_perf = perf_alloc(PC_INTERVAL, "att_control_pid_interval");
	perf_counter_t att_control_pid_err_perf = perf_alloc(PC_COUNT, "att_control_pid_err");

	warnx("starting");

	/* store last control mode to detect mode switches */
	bool control_yaw_position = true;
	bool reset_yaw_sp = true;

	struct pollfd fds[1] = {
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
	};

	while (!thread_should_exit) {

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(att_control_pid_err_perf);

		} else if (ret > 0) {
			/* only run controller if attitude changed */
			perf_begin(att_control_pid_loop_perf);

			/* attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

			bool updated;

			/* parameters */
			orb_check(parameter_update_sub, &updated);

			if (updated) {
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
				/* update parameters */
			}

			/* control mode */
			orb_check(vehicle_control_mode_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub, &control_mode);
			}

			/* manual control setpoint */
			//orb_check(manual_control_setpoint_sub, &updated);

			//if (updated) {
				//orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
			//}

			/* attitude setpoint */
			orb_check(vehicle_attitude_setpoint_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
			}

			/* offboard control setpoint */
			orb_check(offboard_control_setpoint_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(offboard_control_setpoint), offboard_control_setpoint_sub, &offboard_sp);
			}

			/* vehicle status */
			orb_check(vehicle_status_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);
			}

			/* sensors */
			orb_check(sensor_combined_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
			}

			/* set flag to safe value */
			control_yaw_position = false;

			/* reset yaw setpoint if not armed */
			if (!control_mode.flag_armed) {
				reset_yaw_sp = true;
			}

			

			/* check if we should we reset integrals */
			bool reset_integral = !control_mode.flag_armed || att_sp.thrust < 0.1f;	// TODO use landed status instead of throttle

			/* run attitude controller */
			//float manual_pitch = manual.pitch / 1000; // 0 to 1 WARNING: the scale could be wrong
			//float manual_roll = manual.roll / 1000; // 0 to 1
			//att_sp.roll_body = manual_roll >= 0.8 ? 0.8 : manual_roll; //TODO: dangerous hack
			//att_sp.pitch_body = manual_pitch >= 0.8 ? 0.8 : manual_pitch;


			att_control_pid(&att_sp, &att, &rates_sp, control_yaw_position, reset_integral);
			orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);


			/* measure in what intervals the controller runs */
			perf_count(att_control_pid_interval_perf);

			/* run rates controller if needed */

			/* get current rate setpoint */
			bool rates_sp_updated = false;
			orb_check(vehicle_rates_setpoint_sub, &rates_sp_updated);

			if (rates_sp_updated) {
				orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub, &rates_sp);
			}

			/* apply controller */
			float rates[3];
			rates[0] = att.rollspeed;
			rates[1] = att.pitchspeed;
			rates[2] = att.yawspeed; //TODO: yaw rate is unstable
			//rates[2] = 0;
			att_control_rates_pid(&rates_sp, rates, &actuators, reset_integral);


			actuators.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

			perf_end(att_control_pid_loop_perf);
		}
	}

	warnx("stopping, disarming motors");

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	close(vehicle_attitude_sub);
	close(vehicle_control_mode_sub);
	//close(manual_control_setpoint_sub);
	close(actuator_pub);
	close(att_sp_pub);

	perf_print_counter(att_control_pid_loop_perf);
	perf_free(att_control_pid_loop_perf);

	fflush(stdout);
	exit(0);
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: att_control_pid [-m <mode>] [-t] {start|status|stop}\n");
	fprintf(stderr, "    <mode> is 'rates' or 'attitude'\n");
	fprintf(stderr, "    -t enables motor test mode with 10%% thrust\n");
	exit(1);
}

int att_control_pid_main(int argc, char *argv[])
{
	int	ch;
	unsigned int optioncount = 0;

	while ((ch = getopt(argc, argv, "tm:")) != EOF) {
		switch (ch) {
		case 't':
			motor_test_mode = true;
			optioncount += 1;
			break;

		case ':':
			usage("missing parameter");
			break;

		default:
			fprintf(stderr, "option: -%c\n", ch);
			usage("unrecognized option");
			break;
		}
	}

	argc -= optioncount;
	//argv += optioncount;

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1 + optioncount], "start")) {

		thread_should_exit = false;
		att_control_pid_task = task_spawn_cmd("att_control_pid",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 15,
					 2048,
					 att_control_pid_thread_main,
					 NULL);
		exit(0);
	}

	if (!strcmp(argv[1 + optioncount], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
