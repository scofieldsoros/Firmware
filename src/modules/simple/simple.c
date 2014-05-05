/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot.
 */
 
#include <nuttx/config.h>
#include <stdio.h>
//#include <errno.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <modules/commander/gyro_calibration.h>
#include <drivers/drv_pwm_output.h>
#include <poll.h>


#include <mavlink/mavlink_log.h>

#define GYRO_DEVICE_PATH_1 "/dev/gyro1"
 
 __EXPORT int simple_main(int argc, char *argv[]);



// get the servo output
 int get_servo_output()
 {
 	char *dev = "/dev/pwm_output";
 	printf("device: %s\n", dev);

 	unsigned servo_count;
 	int ret;

 	int fd = open(dev, 0);
 	if (fd < 0)
 		err(1, "can't open %s", dev);

 	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
 	if(ret != OK)
 		err(1,"get servo count failed!");
 	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE,0);

 	while(1){
		/* print current servo values */
 		for (unsigned i = 0; i < servo_count; i++) {
 			servo_position_t spos;

 			ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);
 			if (ret == OK){
 				printf("channel %u: %u us\n", i+1, spos);
 				mavlink_log_info(fd,"chanel %u:%u us",i+1,spos);
 			}
 			else
 			{
 				printf("get servo position failed!\n");
 				mavlink_log_info(fd,"get servo position failed!");
 			}
 		}
		//sleep 0.5 second
 		usleep(500000);

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		/* abort on user request */
		char c;
		ret = poll(&fds, 1, 0);
		if (ret > 0) {
			read(0, &c, 1);
			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				exit(0);
 			}
 		}
 	}
 	return OK;
 }
 
 int simple_main(int argc, char *argv[])
 {
	get_servo_output();
 }