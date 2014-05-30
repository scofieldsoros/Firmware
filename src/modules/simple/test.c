#include <nuttx/config.h>

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>

//#include <errno.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <modules/commander/gyro_calibration.h>
#include <drivers/drv_pwm_output.h>

//for gpio
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_gpio.h>

#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <board_config.h>

#include <mavlink/mavlink_log.h>

int main()
{
	unsigned int b = 123456789;
	printf("b in llu= %llu\n",b);
}




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


/*
* GPIOs 0 and 1 must have the same direction as they are buffered
* by a shared 2-port driver. Any attempt to set either sets both.
*/

/* maybe is't much better to use interupt  */
void testGPIO()
{
	int gpio_fd;
	char *gpio_dev;
	gpio_dev = PX4FMU_DEVICE_PATH;
	gpio_fd = open(gpio_dev, O_RDWR);

	uint32_t gpio_values;
	double distance;

	hrt_abstime t,t_start;
	hrt_abstime echo_start_time,echo_stop_time;
	hrt_abstime t1,t2;

	hrt_abstime timeout = 100000;/*  */
	hrt_abstime furthest_time = 25000; // if we don't get response in 25ms,than it's out of our range
	hrt_abstime last_trig_time;
	t = hrt_absolute_time();
	t_start = t;
	last_trig_time = hrt_absolute_time();

	/* initialize */
	ioctl(gpio_fd, GPIO_SET_OUTPUT, GPIO_EXT_1);
	ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_1);
	ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_2);
	ioctl(gpio_fd, GPIO_SET_INPUT, GPIO_EXT_2);
	last_trig_time = hrt_absolute_time();

	int flag = 0;
	/* run for 50s */
	while(t < t_start + 1000000 * 50)
	{
		gpio_values = 0;
		while(true)
		{
			printf("measure started\n");
			ioctl(gpio_fd, GPIO_SET_OUTPUT, GPIO_EXT_1);
			ioctl(gpio_fd, GPIO_SET, GPIO_EXT_1);
			usleep(10);
			ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_1);
			ioctl(gpio_fd, GPIO_SET_INPUT, GPIO_EXT_2);
			t_start = hrt_absolute_time();
//			}
			/* if we can't get the 1 on GPIO_EXT_2, that means the 
			 * distance is either too close or too far.if it's 
			 */
			ioctl(gpio_fd, GPIO_GET, &gpio_values);
			if(gpio_values & GPIO_EXT_2)
			{
				echo_start_time = hrt_absolute_time();
				break;
			}
			usleep(timeout);
		}

		/* if timeout, which means the distance is too far. we should break the cycle */
		while((gpio_values & GPIO_EXT_2) && (t < t_start + furthest_time))
		{
			ioctl(gpio_fd, GPIO_GET, &gpio_values);
			t = hrt_absolute_time();
		}
		echo_stop_time = hrt_absolute_time();

		distance = (double)(echo_stop_time - echo_start_time) * 170.0f / 1000000.0f;
		printf("distance :%.2f m\n",distance);

		/* if we got the right distance ,we should sleep some time 
		 * because we didn't do that in the trig loop 
		 */
		if(distance > 0.4f && distance < 4.0f)
		{
			usleep(timeout);
			/* publish the orb */
		}
	}
}




/************************************************************************************
 * Name: stm32_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *  The previous value of the interrupt handler function pointer.  This value may,
 *  for example, be used to restore the previous handler when multiple handlers are
 *  used.
 *
 ************************************************************************************/

//EXTERN xcpt_t stm32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,bool event, xcpt_t func);

 
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

#include <mavlink/mavlink_log.h>

#include <drivers/drv_range_finder.h>
#include <drivers/sensor.h>

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
	while(t < t_start + 1000000 * 50)
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
			printf("SONAR ERROR: please check the SONAR or the connection!!");
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

