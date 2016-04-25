#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/uORB.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/**
 * Stable Duckling app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int stable_duckling_main(int argc, char *argv[]);

class StableDuckling 
{
public:
	/**
	 * Constructor
	 */
	StableDuckling();

	/**
	 * Destructor, also kills the main task
	 */
	~StableDuckling();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();
private:
	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int	_control_task;			/**< task handle */

	int	_v_att_sub;			/**< vehicle attitude subscription */
	int 	_armed_sub;			/**< arming status subscription */

	orb_advert_t	_actuators_0_pub;	/**< attitude actuator controls publication */
	orb_advert_t	_armed_pub;		/**< arming status publication */

	struct vehicle_attitude_s	_v_att;			/**< vehicle attitude */
	struct actuator_controls_s	_actuators;		/**< actuator controls */
	struct actuator_armed_s		_armed;			/**< actuator arming status */

	float _kp;
	float _ki;
	float _kd;

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace stable_duckling 
{
	StableDuckling *g_duckling = nullptr;
}

StableDuckling::StableDuckling() :
	_task_should_exit(false),
	_control_task(-1),
	_v_att_sub(-1),
	_armed_sub(-1),
	_actuators_0_pub(nullptr),
	_armed_pub(nullptr)	
{
}

StableDuckling::~StableDuckling()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	stable_duckling::g_duckling = nullptr;
}

void
StableDuckling::task_main_trampoline(int argc, char *argv[])
{
	stable_duckling::g_duckling->task_main();
}


void
StableDuckling::task_main()
{
	/* subscribe to sensor_combined topic */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(_v_att_sub, 10);

	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

	_armed.timestamp = hrt_absolute_time();
	_armed.ready_to_arm = true;
	_armed.armed = true;
	_armed_pub = orb_advertise(ORB_ID(actuator_armed), &_armed);
	orb_publish(ORB_ID(actuator_armed), _armed_pub, &_armed);

	/* read back values to validate */
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

	if (_armed.ready_to_arm && _armed.armed) {
		warnx("Actuator armed");

	} else {
		errx(1, "Arming actuators failed");
	}

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for sensor update of 1 file descriptor for 10 ms  */
		int poll_ret = poll(fds, 1, 10);

		/* timed out - periodic check for _task_should_exit */
	        if (poll_ret == 0) {
			continue;
	        } 

	        if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			warnx("poll error %d, %d", poll_ret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
			warnx("Euler angles:\t%8.4f\t%8.4f\t%8.4f",
			       (double)_v_att.roll,
			       (double)_v_att.pitch,
			       (double)_v_att.yaw);

			// analyse attitude data
			// ...
			
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -1.0f;
			_actuators.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
		}


	}

	_control_task = -1;
	return;
}

int
StableDuckling::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("stable_duckling",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&StableDuckling::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int stable_duckling_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: stable_duckling {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (stable_duckling::g_duckling != nullptr) {
			warnx("already running");
			return 1;
		}

		stable_duckling::g_duckling = new StableDuckling;

		if (stable_duckling::g_duckling == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != stable_duckling::g_duckling->start()) {
			delete stable_duckling::g_duckling;
			stable_duckling::g_duckling = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (stable_duckling::g_duckling == nullptr) {
			warnx("not running");
			return 1;
		}

		delete stable_duckling::g_duckling;
		stable_duckling::g_duckling = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (stable_duckling::g_duckling) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}