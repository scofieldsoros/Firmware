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

/* 
 * @file hcsr04.cpp
 * 
 * driver for the HC-SR04 ultrasonic sonar
 * this driver is write accroding to the mb12xx driver 
 *
 */

#include <nuttx/config.h>

#include <drivers/device/device.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

//for gpio
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_gpio.h>

/* Device limits */
#define HCSR04_MIN_DISTANCE (0.40f)
#define HCSR04_MAX_DISTANCE (4.00f)

#define HCSR04_CONVERSION_INTERVAL 100000 /* 100ms */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
 static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

static uint32_t 		_falling_count;
static hrt_abstime 		_echo_start_time;
static hrt_abstime 		_echo_stop_time;

class HCSR04 : public device::CDev
{
public:
	HCSR04();
	virtual ~HCSR04();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

	private:
		float				_min_distance;
		float				_max_distance;
		work_s				_work;		//wqueue.h
		RingBuffer			*_reports;
		bool				_sensor_ok;
		int					_measure_ticks;
		bool				_collect_phase;
		
		int 				_gpio_fd;

		orb_advert_t		_range_finder_topic;

		perf_counter_t		_sample_perf;
		perf_counter_t		_comms_errors;
		perf_counter_t		_buffer_overflows;

		/**
		* Initialise the automatic measurement state machine and start it.
		*
		* @note This function is called at open and error time.  
		*/
		void				start();

		/**
		* Stop the automatic measurement state machine.
		*/
		void				stop();

		/**
		* Set the min and max distance thresholds if you want the end points of the sensors
		* range to be brought in at all, otherwise it will use the defaults HCSR04_MIN_DISTANCE
		* and HCSR04_MAX_DISTANCE
		*/
		void				set_minimum_distance(float min);
		void				set_maximum_distance(float max);
		float				get_minimum_distance();
		float				get_maximum_distance();

		/**
		* Perform a poll cycle; collect from the previous measurement
		* and start a new one.
		*/
		void				cycle();
		int 				init_meassure();
		int					measure();
		int					collect();
		float 				get_distance();

		/**
		* Static trampoline from the workq context; because we don't have a
		* generic workq wrapper yet.
		*
		* @param arg		Instance pointer for the driver that is polling.
		*/
		static void			cycle_trampoline(void *arg);

		static int 			GPIO_EXT_2_falling_handler(int irq, FAR void *context);

};

/*
 * Driver 'main' command.
 */
 extern "C" __EXPORT int hcsr04_main(int argc, char *argv[]);

HCSR04::HCSR04() :
	CDev("HCSR04",RANGE_FINDER_DEVICE_PATH ,0 ),
	_min_distance(HCSR04_MIN_DISTANCE),
	_max_distance(HCSR04_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_range_finder_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "hcsr04_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hcsr04_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "hcsr04_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

HCSR04::~HCSR04()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}
}

int
HCSR04::init()
{
	int ret = ERROR;

	/* do CDev init */
	if (CDev::init() != OK) {
		goto out;
	}

	_falling_count = 0;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(range_finder_report));

	if (_reports == nullptr) {
		goto out;
	}

	/* get a publish handle on the range finder topic */
	struct range_finder_report zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_range_finder_topic = orb_advertise(ORB_ID(sensor_range_finder), &zero_report);

	if (_range_finder_topic < 0) {
		debug("failed to create sensor_range_finder object. Did you start uOrb?");
	}

	/* init the mesurement */
	ret = init_meassure();
	if(ret != OK)
	{
		err(ret, "%s open failed ", RANGE_FINDER_DEVICE_PATH);
	}
	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	out:
	return ret;
}

int HCSR04::init_meassure()
{
	/* how to call the system open ?? */
	_gpio_fd = ::open(PX4FMU_DEVICE_PATH, O_RDWR);
	if(_gpio_fd < 0)
		return 1;

	/* 
	* register to the nuttx of the gpio event 
	* we must set the PIN to GPIO_EXTI for external interrupt
	*
	* the GPIO_PIN4 <-> GPIO_EXT_1 and GPIO_PIN5 <-> GPIO_EXT_2 
	* see "board_config.h"
	*/
	uint32_t pinset = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN5);
	xcpt_t oldhandler = stm32_gpiosetevent(pinset ,false ,true , false, GPIO_EXT_2_falling_handler);

	if(oldhandler != NULL)
	{
		log("WARNING: oldhandler:%p is not NULL! Button events may be lost or aliased!\n",oldhandler);
	}

	warnx("init_messure success!");

	return OK;
}

/* interrupt handler for GPIO falling events */
int
HCSR04::GPIO_EXT_2_falling_handler(int irq, FAR void *context)
{
//	_falling_count ++;
//	_echo_stop_time = hrt_absolute_time();
	return OK;
}


void
HCSR04::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
HCSR04::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
HCSR04::get_minimum_distance()
{
	return _min_distance;
}

float
HCSR04::get_maximum_distance()
{
	return _max_distance;
}

int
HCSR04::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

		case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
				case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling (DRDY) not supported */
				case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
				case 0:
				return -EINVAL;

				/* set default/max polling rate.  default is max */
				case SENSOR_POLLRATE_MAX:
				case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(HCSR04_CONVERSION_INTERVAL);

				/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
				default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate. */
					// if the conversion speed can't reach the request speed, return -EINVAL
					if (ticks < USEC2TICK(HCSR04_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}
		// G get; S set 
		case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}
		// XXX ??
		return (1000 / _measure_ticks);

		//default 2, can be resized through this method
		case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

		case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

		case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

		case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

		case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

		default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
HCSR04::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct range_finder_report);
	struct range_finder_report *rbuf = reinterpret_cast<struct range_finder_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

	/*
	 * While there is space in the caller's buffer, and reports, copy them.
	 * Note that we may be pre-empted by the workq thread while we are doing this;
	 * we are careful to avoid racing with them.
	 */
	 while (count--) {
		if (_reports->get(rbuf)) {
			ret += sizeof(*rbuf);
			rbuf++;
		}
	 }

	/* if there was no data, warn the caller */
	 return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(HCSR04_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}


/*
 * give the GPIO_EXT_1 a pusle last logger than 10us to trigger a messurement
 *
 *  we will restart mesurement if measure() return errors
 */
 int
 HCSR04::measure()
 {
	int ret;
	uint32_t falling_count_1,falling_count_2;
	_echo_start_time = hrt_absolute_time(); 

	/* 
	 * this will trigger the sonar to send a ultrasonic wave which will set the 'Echo'(GPIO_EXT_2) pin
	 *
	 * the GPIO_SET_OUTPUT will trigger a pulse of the GPIO_EXT_2 !!
	 * so we don't need to do ioctl(gpio_fd, GPIO_SET, GPIO_EXT_1) 
	 * and ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_1) anymore...
	 *
	 * this function takes about 20us. seems a little dangerous, 
	 * is it possible that it won't trigger the sonar?? through haven't seem yet...
	 */
	ret = ::ioctl(_gpio_fd, GPIO_SET_OUTPUT, GPIO_EXT_1);
	if(ret != OK){
		log("521:GPIO_SET_OUTPUT error!");
		perf_count(_comms_errors);
		return ret;
	}

	falling_count_1 = _falling_count;

	/*
	ioctl(gpio_fd, GPIO_SET, GPIO_EXT_1);
	usleep(10);		// this function takes about 2000us ,but the ioctl only takes 20us 
	ioctl(gpio_fd, GPIO_CLEAR, GPIO_EXT_1);
	*/

	/* 
	 * must be set to GPIO_SET_INPUT, or we can't get the value of GPIO_EXT_2 
	 * seems this will trigger a pusle too...
	 */
	 ret = ::ioctl(_gpio_fd, GPIO_SET_INPUT, GPIO_EXT_2);
	 if(ret != OK){
	 	log("539:GPIO_SET_INPUT error!");
		perf_count(_comms_errors);
		return ret;
	 }
	 falling_count_2 = _falling_count;
	 if(falling_count_1 == falling_count_2)
	 {
		perf_count(_comms_errors);
		// TODO should check whether the sensor is connected or not!
		log("SONAR ERROR: please check the SONAR or the connection!!");
	 }
	return OK;
}


int
HCSR04::collect()
{
	int	ret = -EIO;

	float distance;

	perf_begin(_sample_perf);

	distance = get_distance();
	struct range_finder_report report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.distance = distance;
	report.valid = distance > get_minimum_distance() && distance < get_maximum_distance() ? 1 : 0;

	/* publish it */
	orb_publish(ORB_ID(sensor_range_finder), _range_finder_topic, &report);

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

/* get the distance  */
float HCSR04::get_distance()
{
	if(_echo_stop_time < _echo_start_time)
	{
		_echo_stop_time = hrt_absolute_time();
	}
	float distance = (float)(_echo_stop_time - _echo_start_time) * 170.0f / 1000000.0f;
	//	printf("distance = %.2f\n",distance);
	return distance;
}

void
HCSR04::start()
{
/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HCSR04::cycle_trampoline, this, 1);
//	cycle_trampoline(this);
	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_RANGEFINDER
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}

	warnx("HCSR04: driver started!");
}

void
HCSR04::stop()
{
	work_cancel(HPWORK, &_work);
}

void
HCSR04::cycle_trampoline(void *arg)
{
	HCSR04 *dev = (HCSR04 *)arg;
	dev->cycle();
}

void
HCSR04::cycle()
{
	/* time to collect(the mesurement stage has done) */
	if (_collect_phase) {
		/* perform collection */
		if (OK != collect()) {
			log("649:collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(HCSR04_CONVERSION_INTERVAL)) {
			/* schedule a fresh cycle call when we are ready to measure again
			 * normally, use the default pollrate, we don't do this
			 *
			 * if the pollrate is bigger than default, we need for another
			 *  (wait _measure_ticks - USEC2TICK(HCSR04_CONVERSION_INTERVAL))
			 */
			work_queue(HPWORK,
				&_work,
				(worker_t)&HCSR04::cycle_trampoline,
				this,
				_measure_ticks - USEC2TICK(HCSR04_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		log("680:measure error");
		err(1,"measure error!");
		/* restart the measurement state machine */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;
//	usleep(HCSR04_CONVERSION_INTERVAL);
	/* schedule a fresh cycle call when the measurement is done 
	 * 
	 * HPWROK hight priority,  LPWORK low priority
	*/

	work_queue(HPWORK,
		&_work,
		(worker_t)&HCSR04::cycle_trampoline,
		this,
		USEC2TICK(HCSR04_CONVERSION_INTERVAL));

}

void
HCSR04::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}



/**
 * Local functions in support of the shell command.
 */
 namespace hcsr04
 {

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
	const int ERROR = -1;

	HCSR04	*g_dev;

	void	start();
	void	stop();
	void	test();
	void	reset();
	void	info();

/**
 * Start the driver.
 */
 void
 start()
 {
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	//TO DO
	g_dev = new HCSR04();

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

	fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
 }

/**
 * Stop the driver
 */
 void stop()
 {
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
 }

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
 void
 test()
 {
	struct range_finder_report report;
	ssize_t sz;
	int ret;

	int fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'hcsr04 start' if the driver is not running", RANGE_FINDER_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (float)report.distance);
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		//TODO  where is poll and what does it do ?  2000 waiting time?? 2ms or 2s ?
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("measurement: %0.3f", (float)report.distance);
		warnx("time:        %lld", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
 }

/**
 * Reset the driver.
 */
 void
 reset()
 {
	int fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "driver open failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
 }

/**
 * Print a little info about the driver.
 */
 void
 info()
 {
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}


	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
 }

} // namespace

int
hcsr04_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	 if (!strcmp(argv[1], "start")) {
		hcsr04::start();
	 }

	/*
	 * Stop the driver
	 */
	 if (!strcmp(argv[1], "stop")) {
		hcsr04::stop();
	 }

	/*
	 * Test the driver/device.
	 */
	 if (!strcmp(argv[1], "test")) {
		hcsr04::test();
	 }

	/*
	 * Reset the driver.
	 */
	 if (!strcmp(argv[1], "reset")) {
		hcsr04::reset();
	 }

	/*
	 * Print driver information.
	 */
	 if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		hcsr04::info();
	 }

	 errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
	}
