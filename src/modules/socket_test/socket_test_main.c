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
 * Currently Socket is not supported on PX4FMU board due to Hardware driver missing at Nuttx.
 * Solution is to stack full linux board on top as gateway.
 */


#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <mavlink/mavlink_log.h>

#define BUFLEN 512
#define NPACK 10
#define PORT 5555
#define SRV_IP "192.168.0.87"


/**
 * daemon management function.
 */
__EXPORT int socket_test_main(int argc, char *argv[]);

int socket_test_main(int argc, char *argv[])
{

	static int mavlink_fd; /* For mavlink debug */
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	printf("[socket_test] starting\n");

	
	struct sockaddr_in si_me, si_other;
	int s, slen = sizeof(si_other);
	char buf[BUFLEN] = "Hey this is a success msg from drone!";

	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		perror(s);
		mavlink_log_info(mavlink_fd, "[socket] initiate udp socket err");
		exit(1);
	}

	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	if (inet_aton(SRV_IP, &si_other.sin_addr) == 0) {
		mavlink_log_info(mavlink_fd, "[socket] inet_aton err");
		exit(1);
	}

	for (int i=0; i<NPACK; i++) {
		if (sendto(s, buf, strlen(buf), 0, &si_other, slen) == -1) {
			mavlink_log_info(mavlink_fd, "[socket] udp socket sendto err");
			break;
		}
	}

	close(s);




	return 0;
}





