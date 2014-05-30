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

#include <stdio.h>

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int simple_main(int argc, char *argv[]);


 
#include <nuttx/config.h>

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>

//for gpio
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_gpio.h>

#include <drivers/drv_hrt.h>	/* High-resolution timer */
#include <arch/board/board.h>
#include <board_config.h>


__EXPORT int simple_main(int argc, char *argv[]);


static hrt_abstime echo_start_time,echo_stop_time;

/* temp gobal */
int gpio_fd;
char *gpio_dev = PX4FMU_DEVICE_PATH;
static uint32_t falling_count = 0;

static int GPIO_EXT_2_falling_handler(int irq, FAR void *context)
{
	falling_count ++;
	echo_stop_time = hrt_absolute_time();
	return OK;
}

/* using the interrupt way */
void interruptGPIO()
{
	gpio_fd = open(gpio_dev, O_RDWR);
	float distance = 0;
	hrt_abstime t,t_start;
	hrt_abstime temp_t;
	t = hrt_absolute_time();
	t_start = t;
	uint32_t falling_count_1,falling_count_2;

	/* 
	 * register to the nuttx of the gpio event 
	 * we must set the PIN to GPIO_EXTI for external interrupt
	 * the GPIO_PIN4 <-> GPIO_EXT_1 and GPIO_PIN5 <-> GPIO_EXT_2 
	 * see "board_config.h"
	 */
	uint32_t pinset = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN5);
	xcpt_t oldhandler = stm32_gpiosetevent(pinset ,false ,true , false, GPIO_EXT_2_falling_handler);

	if(oldhandler != NULL)
	{
		printf("WARNING: oldhandler:%p is not NULL! Button events may be lost or aliased!\n",oldhandler);
	}

	/* don't know is it that if ioctl is set from INPUT to OUTPUT or OUTPUT to input 
	 * will trigger a pusle */
	/* run for 50s */
	while(t < t_start + 1000000 * 5)
	{
		/* trigger measure */
		printf("measure started\n");
		printf("falling_count 0 = %d\n",falling_count);

		echo_start_time = hrt_absolute_time(); 

		/* 
		 * the GPIO_SET_OUTPUT will trigger a pulse of the GPIO_EXT_2 !!
		 * so we don't need to do ioctl(gpio_fd, GPIO_SET, GPIO_EXT_1) 
		 * and ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_1) anymore...
		 *
		 * this function takes about 20us. seems a little dangerous, 
		 * is it possible that it won't trigger the sonar?? haven't seem yet...
		 */
		printf("gpio_fd = %d, GPIO_SET_OUTPUT = %d, GPIO_EXT_1 = %d\n",gpio_fd,GPIO_SET_OUTPUT,GPIO_EXT_1);
		ioctl(gpio_fd, GPIO_SET_OUTPUT, GPIO_EXT_1);
		falling_count_1 = falling_count;
		printf("falling_count 1 = %d\n",falling_count);

		/*
		ioctl(gpio_fd, GPIO_SET, GPIO_EXT_1);
		usleep(10);		// this function takes about 2000us ,but the ioctl only takes 20us 
		ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_1);
		*/

		/* 
		 * must be set to GPIO_SET_INPUT, or we can't get the value of GPIO_EXT_2 
		 * this will trigger the sonar to send a ultrasonic wave which will set the 'Echo'(GPIO_EXT_2) pin
		 * seems this will trigger a pusle too...
		 */
		printf("gpio_fd = %d, GPIO_SET_INPUT = %d, GPIO_EXT_2 = %d\n",gpio_fd,GPIO_SET_INPUT,GPIO_EXT_2);
		ioctl(gpio_fd, GPIO_SET_INPUT, GPIO_EXT_2);
		falling_count_2 = falling_count;
		printf("falling_count 2 = %d\n",falling_count);

		/* 
		 * sleep some time for getting the interrupt. 
		 * this determines the frequency of collecting data
		 */
		usleep(200000);
		if(echo_stop_time < echo_start_time)
		{
			echo_stop_time = hrt_absolute_time();
			printf("ERROR: we didn't get the falling! \n");
		}
		if(falling_count_1 == falling_count_2)
		{
			printf("SONAR ERROR: please check the SONAR or the connection!!\n");
		}
		distance = (float)(echo_stop_time - echo_start_time) * 170.0f / 1000000.0f;
		printf("distance :%.2f m\n",distance);
		printf("start :%llu , stop: %llu\n",echo_start_time,echo_stop_time);
		printf("falling_count 3 = %d\n",falling_count);
		t = hrt_absolute_time();
	}
	printf("total interrupt count :%d\n",falling_count);
}

int sonar_detect()
{
	return 1;
}

/* maybe cpp would be much better on namespace. I can use init() rather than sonar_init() in cpp */
int sonar_init()
{

}

int simple_main(int argc, char *argv[])
{
	//get_servo_output();
	interruptGPIO();
	return OK;
}

