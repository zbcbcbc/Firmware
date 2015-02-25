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


#include <sys/types.h>
#include <sys/stat.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>


#include <systemlib/cpuload.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define CL "\033[K" // clear line


static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;



/**
 * daemon management function.
 */
__EXPORT int system_usage_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int system_usage_thread_main(int argc, char *argv[]);


/**
 * Open the file to log 
 */
static 
int open_logfile(void);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
        if (reason)
                warnx("%s\n", reason);
        errx(1, "usage: system_usage {start|stop|status} [-p <additional params>]\n\n");
}


extern struct system_load_s system_load;

static const char *
tstate_name(const tstate_t s)
{
	switch (s) {
	case TSTATE_TASK_INVALID:    return "init";

	case TSTATE_TASK_PENDING:    return "PEND";
	case TSTATE_TASK_READYTORUN: return "READY";
	case TSTATE_TASK_RUNNING:    return "RUN";

	case TSTATE_TASK_INACTIVE:   return "inact";
	case TSTATE_WAIT_SEM:        return "w:sem";
#ifndef CONFIG_DISABLE_SIGNALS
	case TSTATE_WAIT_SIG:        return "w:sig";
#endif
#ifndef CONFIG_DISABLE_MQUEUE
	case TSTATE_WAIT_MQNOTEMPTY: return "w:mqe";
	case TSTATE_WAIT_MQNOTFULL:  return "w:mqf";
#endif
#ifdef CONFIG_PAGING
	case TSTATE_WAIT_PAGEFILL:   return "w:pgf";
#endif

	default:
		return "ERROR";
	}
}





/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int system_usage_main(int argc, char *argv[]) {
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("daemon alreading running\n");
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("system_usage", 
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									4096,
									system_usage_thread_main,
									(argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}


	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	} 

	if (!strcmp(argv[1], "status")) {
		if (thread_running) 
			warnx("\trunning\n");
		else
			warnx("\tnot started\n");
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static
int open_logfile()
{
	/* make folder on sdcard */
	uint16_t file_number = 1; // start with file log001

	/* string to hold the path to the log */
	char path_buf[64] = "";

	int fd = 0;

	char *folder_path = "/fs/microsd/system_usage";

	/* create log folder,  return -1 if dir already exists */
	int ret = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IRWXO);

	/* set up file path: e.g. /fs/microsd/sess001/log001.bin */
	sprintf(path_buf, "%s/system_usage_log", folder_path);



	fd = fopen(path_buf, "w");

	if (fd == 0) {
		warn("opening %s failed", path_buf);
		return 0;
	}

	char buf[128] = "";
	sprintf(buf, CL "%4s %*-s %8s %6s %11s %10s %-6s\n",
						   "PID",
						   CONFIG_TASK_NAME_SIZE, "COMMAND",
						   "CPU(ms)",
						   "CPU(%)",
						   "USED/STACK",
						   "PRIO(BASE)",
#if CONFIG_RR_INTERVAL > 0
						   "TSLICE"
#else
						   "STATE"
#endif
						   );

	size_t w_ret = fwrite(buf, 1, strlen(buf), fd);

	warnx("logging to: %s.", path_buf);
	//mavlink_log_info(mavlink_fd, "[sdlog2] log: %s", path_buf);

	return fd;

	
}


int system_usage_thread_main(int argc, char *argv[]) {
	warnx("[system usage] starting...\n");

	thread_running = true;

	// initiate variables
	uint64_t total_user_time = 0;

	int running_count = 0;
	int blocked_count = 0;

	uint64_t new_time = hrt_absolute_time();
	uint64_t interval_start_time = new_time;

	uint64_t last_times[CONFIG_MAX_TASKS];
	float curr_loads[CONFIG_MAX_TASKS];

	for (int t = 0; t < CONFIG_MAX_TASKS; t++)
		last_times[t] = 0;

	float interval_time_ms_inv = 0.f;

	/* clear screen */
	printf("\033[2J");


	int logfile_fd = open_logfile();

	while (!thread_should_exit) {
		int   i;
		uint64_t curr_time_us;
		uint64_t idle_time_us;

		curr_time_us = hrt_absolute_time();
		idle_time_us = system_load.tasks[0].total_runtime;

		if (new_time > interval_start_time)
			interval_time_ms_inv = 1.f / ((float)((new_time - interval_start_time) / 1000));

		running_count = 0;
		blocked_count = 0;
		total_user_time = 0;

		for (i = 0; i < CONFIG_MAX_TASKS; i++) {
			uint64_t interval_runtime;

			if (system_load.tasks[i].valid) {
				switch (system_load.tasks[i].tcb->task_state) {
				case TSTATE_TASK_PENDING:
				case TSTATE_TASK_READYTORUN:
				case TSTATE_TASK_RUNNING:
					running_count++;
					break;

				case TSTATE_TASK_INVALID:
				case TSTATE_TASK_INACTIVE:
				case TSTATE_WAIT_SEM:
#ifndef CONFIG_DISABLE_SIGNALS
				case TSTATE_WAIT_SIG:
#endif
#ifndef CONFIG_DISABLE_MQUEUE
				case TSTATE_WAIT_MQNOTEMPTY:
				case TSTATE_WAIT_MQNOTFULL:
#endif
#ifdef CONFIG_PAGING
				case TSTATE_WAIT_PAGEFILL:
#endif
					blocked_count++;
					break;
				}
			}

			interval_runtime = (system_load.tasks[i].valid && last_times[i] > 0 &&
								system_load.tasks[i].total_runtime > last_times[i])
				? (system_load.tasks[i].total_runtime - last_times[i]) / 1000
				: 0;

			last_times[i] = system_load.tasks[i].total_runtime;

			if (system_load.tasks[i].valid && (new_time > interval_start_time)) {
				curr_loads[i] = interval_runtime * interval_time_ms_inv;

				if (i > 0)
					total_user_time += interval_runtime;
			} else {
				curr_loads[i] = 0;
			}
		}

		for (i = 0; i < CONFIG_MAX_TASKS; i++) {
			if (system_load.tasks[i].valid && (new_time > interval_start_time)) {

				size_t ret;
				if (system_load.tasks[i].tcb->pid == 0) {
					float idle;
					float task_load;
					float sched_load;

					idle = curr_loads[0];
					task_load = (float)(total_user_time) * interval_time_ms_inv;

					/* this can happen if one tasks total runtime was not computed
					   correctly by the scheduler instrumentation TODO */
					if (task_load > (1.f - idle))
						task_load = (1.f - idle);

					sched_load = 1.f - idle - task_load;


					/* Log system info */
					char buf1[128] = "";	
					sprintf(buf1, CL "CPU usage: %.2f%% tasks, %.2f%% sched, %.2f%% idle\n",
						   (double)(task_load * 100.f),
						   (double)(sched_load * 100.f),
						   (double)(idle * 100.f));

					//printf("%s has %d strlen, has size %d\n", buf1, strlen(buf1), sizeof buf1);

					ret = fwrite(buf1, 1, strlen(buf1), logfile_fd);
					//printf("system info written %d bytes\n", ret);
					/* print system information */
					//printf("\033[H"); /* move cursor home and clear screen */
					//printf(CL "Processes: %d total, %d running, %d sleeping\n",
					//	   system_load.total_count,
					//	   running_count,
					//	   blocked_count);
					//printf(CL "CPU usage: %.2f%% tasks, %.2f%% sched, %.2f%% idle\n",
					//	   (double)(task_load * 100.f),
					//	   (double)(sched_load * 100.f),
					//	   (double)(idle * 100.f));
					//printf(CL "Uptime: %.3fs total, %.3fs idle\n\n",
					//	   (double)curr_time_us / 1000000.d,
					//	   (double)idle_time_us / 1000000.d);

					/* header for task list */
					//printf(CL "%4s %*-s %8s %6s %11s %10s %-6s\n",
					//	   "PID",
					//	   CONFIG_TASK_NAME_SIZE, "COMMAND",
					//	   "CPU(ms)",
					//	   "CPU(%)",
					//	   "USED/STACK",
					//	   "PRIO(BASE)",
//#if CONFIG_RR_INTERVAL > 0
//						   "TSLICE"
//#else
//						   "STATE"
//#endif
//						   );
				}

				unsigned stack_size = (uintptr_t)system_load.tasks[i].tcb->adj_stack_ptr -
					(uintptr_t)system_load.tasks[i].tcb->stack_alloc_ptr;
				unsigned stack_free = 0;
				uint8_t *stack_sweeper = (uint8_t *)system_load.tasks[i].tcb->stack_alloc_ptr;

				while (stack_free < stack_size) {
					if (*stack_sweeper++ != 0xff)
						break;

					stack_free++;
				}

				char buf2[128] = "";


				sprintf(buf2, CL "%4d %*-s %8lld %2d.%03d %5u/%5u %3u (%3u) \n",
					   system_load.tasks[i].tcb->pid,
					   CONFIG_TASK_NAME_SIZE, system_load.tasks[i].tcb->name,
					   (system_load.tasks[i].total_runtime / 1000),
					   (int)(curr_loads[i] * 100),
					   (int)(curr_loads[i] * 100000.0f - (int)(curr_loads[i] * 1000.0f) * 100),
					   stack_size - stack_free,
					   stack_size,
					   system_load.tasks[i].tcb->sched_priority,
					   system_load.tasks[i].tcb->base_priority);

				//printf("%s has %d strlen, has size %d\n", buf2, strlen(buf2), sizeof buf2);

				ret = fwrite(buf2, 1, strlen(buf2), logfile_fd);

				//printf("process info written:%d bytes\n", ret);
			}
		}

		interval_start_time = new_time;

		// sleep 200 ms
		usleep(200000);
		new_time = hrt_absolute_time();
	}

	warnx("[system usage] exiting...\n");

	fclose(logfile_fd);
	thread_running = false;

	return 0;
}



