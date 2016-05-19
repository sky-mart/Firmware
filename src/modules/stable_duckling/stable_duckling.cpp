#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/safety.h>
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

enum StableDucklingMode
{
	SILENCE = 0,
	MANUAL,
	PID,
	IMPULSE,
};

enum StableDucklingCommand
{
	SET_MODE = 10001,
	SET_PWM,
	SET_PID_COEFFS,
	SET_IMP_PARAMS
};

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
	int 	_v_cmd_sub;			/**< vehicle command subscription */
	int 	_safety_sub;			/**< safety subscription */

	orb_advert_t	_actuators_0_pub;	/**< attitude actuator controls publication */
	orb_advert_t	_armed_pub;		/**< arming status publication */
	orb_advert_t	_safety_pub;		/**< safety check publication */

	struct vehicle_attitude_s	_v_att;			/**< vehicle attitude */
	struct actuator_controls_s	_actuators;		/**< actuator controls */
	struct actuator_armed_s		_armed;			/**< actuator arming status */
	struct vehicle_command_s	_v_cmd;			/**< vehicle command */
	struct vehicle_attitude_s 	_v_att_init;		/**< initial vehicle attitude */
	struct safety_s			_safety;

	float _kp;
	float _ki;
	float _kd;

	float _roll_integral;
	float _thrust;

	/**
	 * Assume that _thrust depends on control signal like
	 * _thrust = _k_thrust * control_signal + _b_thrust 
	 */
	float _k_thrust;
	float _b_thrust;

	StableDucklingMode _mode;

	/**
	 * Impulse mode allows to explore system response
	 * on impulses of different length and amplitude
	 */
	int _impulse_index;
	int _impulse_length; // in CONTROL_INTERVALS
	float _impulse_ampl; // control signal value

	static const float MIN_CONTROL;
	static const float MAX_CONTROL;
	static const int CONTROL_INTERVAL;

	void apply_thrust();
	void stop_motors();
	void control();
	void analyse_command();
	void safety_off();
	void arm_actuators();

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
	_v_cmd_sub(-1),
	_safety_sub(-1),
	_actuators_0_pub(nullptr),
	_armed_pub(nullptr),
	_safety_pub(nullptr),
	_roll_integral(0),
	_k_thrust(0.4638f), // experimental values
	_b_thrust(0.421),
	_mode(SILENCE),
	_impulse_index(0),
	_impulse_length(20),
	_impulse_ampl(1.0)	
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

void StableDuckling::stop_motors()
{
	_actuators.control[0] = MIN_CONTROL;
	_actuators.control[1] = MIN_CONTROL;
}

void StableDuckling::control() 
{
	switch (_mode) {
	case PID:
		_roll_integral += _v_att.roll;

		_thrust = 	_kp * _v_att.roll +
				_ki * _roll_integral +
				_kd * _v_att.rollspeed;

		apply_thrust();
		break;
	case IMPULSE:
		if (_impulse_index == _impulse_length) {
			_actuators.control[0] = MIN_CONTROL;
			_actuators.control[1] = MIN_CONTROL;
			_impulse_index = 0;
			_mode = SILENCE;
		} else {
			_actuators.control[0] = _impulse_ampl;
			_actuators.control[1] = MIN_CONTROL;
			_impulse_index++;
		}
		break;
	case MANUAL:
		break;
	default:
		stop_motors();
	}				

	_actuators.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
}

void StableDuckling::apply_thrust() 
{
	if (_thrust > 0) {
		_actuators.control[0] = (_thrust - _b_thrust) / _k_thrust;
		if (_actuators.control[0] > MAX_CONTROL) 
			_actuators.control[0] = MAX_CONTROL;
		_actuators.control[1] = MIN_CONTROL;
	} else {
		_actuators.control[1] = (_thrust - _b_thrust) / _k_thrust;
		if (_actuators.control[1] > MAX_CONTROL) 
			_actuators.control[1] = MAX_CONTROL;
		_actuators.control[0] = MIN_CONTROL;
	}
}

void StableDuckling::analyse_command() 
{
	switch ((StableDucklingCommand)_v_cmd.command) {
	case SET_MODE:
		_mode = (StableDucklingMode)_v_cmd.param1;
		stop_motors();
		warnx("mode %d set", _mode);
		break;
	case SET_PWM:
		_actuators.control[0] = _v_cmd.param1;
		_actuators.control[1] = _v_cmd.param2;
		warnx("pwm (%.2f %.2f) set", 
			(double)_actuators.control[0], 
			(double)_actuators.control[1]);
		break;
	case SET_PID_COEFFS:
		_kp = _v_cmd.param1;
		_ki = _v_cmd.param2;
		_kd = _v_cmd.param3;
		warnx("pid coeffs (%.2f %.2f %.2f) set",
			(double)_kp,
			(double)_ki,
			(double)_kd);
		break;
	case SET_IMP_PARAMS:
		_impulse_length = (int)_v_cmd.param1;
		_impulse_ampl = _v_cmd.param2;
		break;
	default:
		break;
	}
}

void StableDuckling::safety_off()
{
	_safety.timestamp = hrt_absolute_time();
	_safety.safety_switch_available = false;
	_safety.safety_off = true;
	_safety_pub = orb_advertise(ORB_ID(safety), &_safety);
	orb_publish(ORB_ID(safety), _safety_pub, &_safety);

	_safety_sub = orb_subscribe(ORB_ID(safety));
	orb_copy(ORB_ID(safety), _safety_sub, &_safety);

	if (_safety.safety_off) {
		warnx("Safety's turned off");
	} else {
		errx(1, "Turning safety off failed");
	}
}

void StableDuckling::arm_actuators()
{
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
}

void
StableDuckling::task_main_trampoline(int argc, char *argv[])
{
	stable_duckling::g_duckling->task_main();
}


void
StableDuckling::task_main()
{
	_v_cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	orb_set_interval(_v_cmd_sub, CONTROL_INTERVAL);

	/* subscribe to sensor_combined topic */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(_v_att_sub, CONTROL_INTERVAL);

	// safety_off();
	// arm_actuators();	
	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[2];
	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	fds[1].fd = _v_cmd_sub;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for sensor update of 1 file descriptor for 10 ms  */
		int poll_ret = poll(fds, 2, CONTROL_INTERVAL);

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

			control();
		}

		if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_command), _v_cmd_sub, &_v_cmd);

			analyse_command();
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

const float StableDuckling::MIN_CONTROL = -1.0f;
const float StableDuckling::MAX_CONTROL = 1.0f;
const int StableDuckling::CONTROL_INTERVAL = 10; 	// ms

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