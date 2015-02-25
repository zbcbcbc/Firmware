#
# Attitude estimator (Extended Kalman Filter)
#

MODULE_COMMAND	 = att_estimator_ekf

SRCS		 = att_estimator_ekf_main.cpp \
		   att_estimator_ekf_params.c \
		   codegen/eye.c \
		   codegen/attitudeKalmanfilter.c \
		   codegen/mrdivide.c \
		   codegen/rdivide.c \
		   codegen/attitudeKalmanfilter_initialize.c \
		   codegen/attitudeKalmanfilter_terminate.c \
		   codegen/rt_nonfinite.c \
		   codegen/rtGetInf.c \
		   codegen/rtGetNaN.c \
		   codegen/norm.c \
		   codegen/cross.c
