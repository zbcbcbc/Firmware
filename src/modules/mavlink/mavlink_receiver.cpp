/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_receiver.c
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

/* XXX trim includes */
#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
#include <mavlink/mavlink_log.h>
#include <commander/px4_custom_mode.h>

__BEGIN_DECLS

#include "mavlink_bridge_header.h"
#include "waypoints.h"
#include "orb_topics.h"
#include "missionlib.h"
//#include "mavlink_hil.h"
#include "mavlink_parameters.h"
#include "util.h"

extern bool gcs_link;

__END_DECLS

/* XXX should be in a header somewhere */
extern "C" pthread_t receive_start(int uart);

static void handle_message(mavlink_message_t *msg);
static void *receive_thread(void *arg);

static mavlink_status_t status;
//static struct vehicle_vicon_position_s vicon_position;
static struct vehicle_command_s vcmd;

//struct vehicle_global_position_s hil_global_pos;
//struct vehicle_attitude_s hil_attitude;
//struct vehicle_gps_position_s hil_gps;
//struct sensor_combined_s hil_sensors;
//struct battery_status_s	hil_battery_status;
//static orb_advert_t pub_hil_global_pos = -1;
//static orb_advert_t pub_hil_attitude = -1;
//static orb_advert_t pub_hil_gps = -1;
//static orb_advert_t pub_hil_sensors = -1;
//static orb_advert_t pub_hil_gyro = -1;
//static orb_advert_t pub_hil_accel = -1;
//static orb_advert_t pub_hil_mag = -1;
//static orb_advert_t pub_hil_baro = -1;
//static orb_advert_t pub_hil_airspeed = -1;
//static orb_advert_t pub_hil_battery = -1;

/* packet counter */
//static int hil_counter = 0;
//static int hil_frames = 0;
static uint64_t old_timestamp = 0;


//static orb_advert_t kin_pos_pub = 0;
static orb_advert_t mc_pub = 0;
static orb_advert_t flow_pub = -1;
static orb_advert_t cmd_pub = -1;
//static orb_advert_t vicon_position_pub = -1;
static orb_advert_t telemetry_status_pub = -1;

static int mavlink_fd;

static void
handle_message(mavlink_message_t *msg) {

	/* Receive command... */
	if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {

		
		// EDITED
		mavlink_log_info(mavlink_fd, "[mavlink] received COMMAND_LONG");

		mavlink_command_long_t cmd_mavlink;
		mavlink_msg_command_long_decode(msg, &cmd_mavlink);

		if (cmd_mavlink.target_system == mavlink_system.sysid && ((cmd_mavlink.target_component == mavlink_system.compid)
				|| (cmd_mavlink.target_component == MAV_COMP_ID_ALL))) {
			//check for MAVLINK terminate command
			if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
				/* This is the link shutdown command, terminate mavlink */
				printf("[mavlink] Terminating .. \n");
				fflush(stdout);
				usleep(50000);

				/* terminate other threads and this thread */
				thread_should_exit = true;

			} else {

				/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
				vcmd.param1 = cmd_mavlink.param1;
				vcmd.param2 = cmd_mavlink.param2;
				vcmd.param3 = cmd_mavlink.param3;
				vcmd.param4 = cmd_mavlink.param4;
				vcmd.param5 = cmd_mavlink.param5;
				vcmd.param6 = cmd_mavlink.param6;
				vcmd.param7 = cmd_mavlink.param7;
				// XXX do proper translation
				vcmd.command = (enum VEHICLE_CMD)cmd_mavlink.command;
				vcmd.target_system = cmd_mavlink.target_system;
				vcmd.target_component = cmd_mavlink.target_component;
				vcmd.source_system = msg->sysid;
				vcmd.source_component = msg->compid;
				vcmd.confirmation =  cmd_mavlink.confirmation;

				/* check if topic is advertised */
				

				if (cmd_pub <= 0) {
					cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

				} else {
					/* publish */
					orb_publish(ORB_ID(vehicle_command), cmd_pub, &vcmd);
				}
			}
		}
	}

	/*Optical flow messages, coming from mavlink_onbard */
	if (msg->msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {

		//mavlink_log_info(mavlink_fd, "[mavlink] received OPTICAL_FLOW data lalala");

		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		struct optical_flow_s f;

		f.timestamp = flow.time_usec;
		f.flow_raw_x = flow.flow_x;
		f.flow_raw_y = flow.flow_y;
		f.flow_comp_x_m = flow.flow_comp_m_x;
		f.flow_comp_y_m = flow.flow_comp_m_y;
		f.ground_distance_m = flow.ground_distance;
		f.quality = flow.quality;
		f.sensor_id = flow.sensor_id;

		/* check if topic is advertised */
		if (flow_pub <= 0) {
			flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

		} else {
			/* publish */
			orb_publish(ORB_ID(optical_flow), flow_pub, &f);
		}
	}

	/* Set vehicle mode */
	if (msg->msgid == MAVLINK_MSG_ID_SET_MODE) {

		
		// EDITED
		mavlink_log_info(mavlink_fd, "[mavlink] received SET_MODE");

		/* Set mode on request */
		mavlink_set_mode_t new_mode;
		mavlink_msg_set_mode_decode(msg, &new_mode);

		union px4_custom_mode custom_mode;
		custom_mode.data = new_mode.custom_mode;
		/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
		vcmd.param1 = new_mode.base_mode;
		vcmd.param2 = custom_mode.main_mode;
		vcmd.param3 = 0;
		vcmd.param4 = 0;
		vcmd.param5 = 0;
		vcmd.param6 = 0;
		vcmd.param7 = 0;
		vcmd.command = VEHICLE_CMD_DO_SET_MODE;
		vcmd.target_system = new_mode.target_system;
		vcmd.target_component = MAV_COMP_ID_ALL;
		vcmd.source_system = msg->sysid;
		vcmd.source_component = msg->compid;
		vcmd.confirmation = 1;

		/* check if topic is advertised */
		if (cmd_pub <= 0) {
			cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

		} else {
			/* create command */
			orb_publish(ORB_ID(vehicle_command), cmd_pub, &vcmd);
		}
	}

	/* Handle Vicon position estimates 
		Currently disabled
	if (msg->msgid == MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE) {
		mavlink_vicon_position_estimate_t pos;
		mavlink_msg_vicon_position_estimate_decode(msg, &pos);

		vicon_position.timestamp = hrt_absolute_time();

		vicon_position.x = pos.x;
		vicon_position.y = pos.y;
		vicon_position.z = pos.z;

		vicon_position.roll = pos.roll;
		vicon_position.pitch = pos.pitch;
		vicon_position.yaw = pos.yaw;

		if (vicon_position_pub <= 0) {
			vicon_position_pub = orb_advertise(ORB_ID(vehicle_vicon_position), &vicon_position);

		} else {
			orb_publish(ORB_ID(vehicle_vicon_position), vicon_position_pub, &vicon_position);
		}
	}*/


	/* handle status updates of the radio */
	if (msg->msgid == MAVLINK_MSG_ID_RADIO_STATUS) {

		// EDITED
		mavlink_log_info(mavlink_fd, "[mavlink] received RADIO_STATUS");

		struct telemetry_status_s tstatus;

		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		/* publish telemetry status topic */
		tstatus.timestamp = hrt_absolute_time();
		tstatus.type = TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO;
		tstatus.rssi = rstatus.rssi;
		tstatus.remote_rssi = rstatus.remrssi;
		tstatus.txbuf = rstatus.txbuf;
		tstatus.noise = rstatus.noise;
		tstatus.remote_noise = rstatus.remnoise;
		tstatus.rxerrors = rstatus.rxerrors;
		tstatus.fixed = rstatus.fixed;

		if (telemetry_status_pub == 0) {
			telemetry_status_pub = orb_advertise(ORB_ID(telemetry_status), &tstatus);

		} else {
			orb_publish(ORB_ID(telemetry_status), telemetry_status_pub, &tstatus);
		}
	}

	/* Added as a tunnel to manually control through radio modules */
	if (msg->msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {

		// EDITED
		//mavlink_log_info(mavlink_fd, "[mavlink] received MANUAL_CONTROL");

		mavlink_manual_control_t man;
		mavlink_msg_manual_control_decode(msg, &man);

		struct manual_control_setpoint_s mc;
		

		int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

		/* get a copy first, to prevent altering values that are not sent by the mavlink command */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &mc);
		mc.timestamp = hrt_absolute_time();
		mc.roll = man.x / 1000.0f;
		mc.pitch = man.y / 1000.0f;
		mc.yaw = man.r / 1000.0f; //WARNING: cancel yaw control and set yaw set point to 0 always.
		mc.throttle = man.z / 1000.0f;
		

	
		/* fake RC channels with manual control input from simulator */

		if (mc_pub == 0) {
			mc_pub = orb_advertise(ORB_ID(manual_control_setpoint), &mc);

		} else {
			// EDITED for debugging
			//mavlink_log_info(mavlink_fd, "[mavlink] manual control setpoint published");
			orb_publish(ORB_ID(manual_control_setpoint), mc_pub, &mc);
		}

		
	}
	/*
	if (msg->msgid == MAVLINK_MSG_ID_KINECT_POSITION) {
		mavlink_kinect_position_t mav_kin_pos;
		mavlink_msg_kinect_position_decode(msg, &mav_kin_pos);

		struct vehicle_kinect_position_s kin_pos;
		

		int kin_pos_sub = orb_subscribe(ORB_ID(vehicle_kinect_position));

		/* get a copy first, to prevent altering values that are not sent by the mavlink command 
		orb_copy(ORB_ID(vehicle_kinect_position), kin_pos_sub, &kin_pos);
		kin_pos.timestamp = hrt_absolute_time();
		kin_pos.x = mav_kin_pos.x;
		kin_pos.y = mav_kin_pos.y;
		kin_pos.valid = true;

		if (kin_pos_pub == 0) {
			kin_pos_pub = orb_advertise(ORB_ID(vehicle_kinect_position), &kin_pos);
		} else {
			orb_publish(ORB_ID(vehicle_kinect_position), kin_pos_pub, &kin_pos);
		}
	}*/

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 * Currently disabled
	 */
}


/**
 * Receive data from UART.
 */
static void *
receive_thread(void *arg)
{
	int uart_fd = *((int *)arg);

	const int timeout = 1000;
	uint8_t buf[32];

	mavlink_message_t msg;

	prctl(PR_SET_NAME, "mavlink_uart_rcv", getpid());

	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	ssize_t nread = 0;

	// EDITED: open mavlink_fd
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	while (!thread_should_exit) {
		if (poll(fds, 1, timeout) > 0) {
			if (nread < sizeof(buf)) {
				/* to avoid reading very small chunks wait for data before reading */
				usleep(1000);
			}

			/* non-blocking read. read may return negative values */
			nread = read(uart_fd, buf, sizeof(buf));

			/* if read failed, this loop won't execute */
			for (ssize_t i = 0; i < nread; i++) {
				if (mavlink_parse_char(chan, buf[i], &msg, &status)) {
					/* handle generic messages and commands */
					handle_message(&msg);

					/* handle packet with waypoint component */
					mavlink_wpm_message_handler(&msg, &global_pos, &local_pos);

					/* handle packet with parameter component */
					mavlink_pm_message_handler(MAVLINK_COMM_0, &msg);
				}
			}
		}
	}

	return NULL;
}

pthread_t
receive_start(int uart)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	// set to non-blocking read
	int flags = fcntl(uart, F_GETFL, 0);
	fcntl(uart, F_SETFL, flags | O_NONBLOCK);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 3000);

	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, receive_thread, &uart);

	pthread_attr_destroy(&receiveloop_attr);
	return thread;
}
