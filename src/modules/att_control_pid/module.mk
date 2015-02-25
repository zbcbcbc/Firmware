#
# Build multirotor attitude controller
#

MODULE_COMMAND	= att_control_pid

SRCS		= att_control_pid_main.c \
		  att_control_pid.c \
		  att_rate_control_pid.c
