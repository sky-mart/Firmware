#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/stable_duckling.h>
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

#include <math.h>

#include <drivers/ws2812/ws2812.h>

#define DT CONTROL_INTERVAL
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
	SIN,
	TWO_STAGE
};

enum StableDucklingCommand
{
	SET_MODE = 10001,
	SET_PWM,
	SET_PID_COEFFS,
	SET_IMP_PARAMS,
	SET_ANCHOR_ROLL,
	SET_MIN_CONTROL,
	SET_SIN_PARAMS,
	SET_TWO_PARAMS
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
	orb_advert_t	_stable_pub;		/**< my logging data message publication */

	struct vehicle_attitude_s	_v_att;			/**< vehicle attitude */
	struct actuator_controls_s	_actuators;		/**< actuator controls */
	struct vehicle_command_s	_v_cmd;			/**< vehicle command */
	struct vehicle_attitude_s 	_v_att_init;		/**< initial vehicle attitude */
	struct stable_duckling_s	_stable;		/**< my logging data message */

	float _kp;
	float _ki;
	float _kd;

	float _rollspeed_prev;
	float _roll_integral;
	float _thrust;

	bool _anchor_roll_set;
	float _anchor_roll;
	int _anchor_index;
	int _anchor_length;

	/**
	 * Assume that _thrust depends on control signal like
	 * _thrust = _k_thrust * control_signal + _b_thrust 
	 */
	float _k_thrust;
	float _b_thrust;

	StableDucklingMode _mode;

	/**
	 * Impulse mode allows to explore system response
	 * to impulses of different length and amplitude
	 */
	int _impulse_index;
	int _impulse_length; // in CONTROL_INTERVALS
	float _impulse_ampl; // control signal value

	/**
	 * Sin mode allows to explore system frequency responses
	 */
	double _sin_mag;	// sin magnitude
	double _sin_freq;	// sin frequency
	double _sin_phase; 	// sin phase 

	/**
	 * Two stage mode
	 */
	float _tau;			// typical system time
	float _two_stage_step;

	/**
	 * For faster reaction motors have to be always on
	 */
	float _min_control;

	uint8_t _cur_led_color;

	static const int BACK; // back motor
	static const int LEFT;
	static const int RIGHT;
	static const float MIN_CONTROL;
	static const float MAX_CONTROL;
	static const int CONTROL_INTERVAL;
	static const char LED_COLOR[2][3]; // 2 colors, green-red-blue

	void two_stage();
	void apply_thrust();
	void stop_motors();
	void anchor();
	void control();
	void analyse_command();
	void toggle_led();

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
	_stable_pub(nullptr),
	_kp(1.0),
	_ki(0),
	_kd(0),
	_rollspeed_prev(0),
	_roll_integral(0),
	_anchor_roll_set(false),
	_anchor_roll(0),
	_anchor_index(0),
	_anchor_length(100),
	_k_thrust(0.4638f), // experimental values
	_b_thrust(0.421f),
	_mode(SILENCE),
	_impulse_index(0),
	_impulse_length(20),
	_impulse_ampl(1.0f),
	_sin_mag(0.1),
	_sin_freq(1.0),
	_sin_phase(0),
	_tau(1.0f),
	_two_stage_step(2.0f),
	_min_control(-1.0f),
	_cur_led_color(0)	
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
	_actuators.control[LEFT] = MIN_CONTROL;
	_actuators.control[RIGHT] = MIN_CONTROL;
	_actuators.control[BACK] = MIN_CONTROL;
}

void StableDuckling::anchor() 
{
	if (!_anchor_roll_set) {
		if (_anchor_index < _anchor_length) {
			_anchor_roll += _v_att.roll;
			_anchor_index++;
		} else {
			_anchor_roll /= _anchor_length;
			_anchor_index = 0;
			_anchor_roll_set = true;
			_roll_integral = 0;

			_stable.anchor_roll = _anchor_roll;
			orb_publish(ORB_ID(stable_duckling), _stable_pub, &_stable);

			warnx("new anchor: %.2f", (double)_anchor_roll);
		}
	}
	_v_att.roll -= _anchor_roll;
}

void StableDuckling::two_stage() 
{
#define TWO_STAGE_DEBUG_NUM 100
	static int two_stage_debug_counter = 0;

	if (two_stage_debug_counter < TWO_STAGE_DEBUG_NUM) {
		warnx("step #%d", two_stage_debug_counter + 1);
		warnx("a: %.2f, w: %.2f, e: %.8f", 
			(double)_v_att.roll,
			(double)_v_att.rollspeed,
			(double)_v_att.rollacc
			);
	}

	// first stage: define desired roll acceleration
	float rollacc_des = 0;
	if (_v_att.roll * _v_att.rollspeed > 0) {
		rollacc_des = - (4*_v_att.roll + 3*_v_att.rollspeed) / 
					_tau / _tau;
	} else if (-4*_v_att.roll/_v_att.rollspeed/_tau > 1) {
		rollacc_des = - (4*_v_att.roll + 3*_v_att.rollspeed) / 
					_tau / _tau;
	} else {
		float t1 = - 2 * _v_att.roll / _v_att.rollspeed;
		rollacc_des = - _v_att.rollspeed / t1;
	}

	// if (two_stage_debug_counter < TWO_STAGE_DEBUG_NUM) {
	// 	warnx("e_des: %.2f", (double)rollacc_des);
	// 	warnx("actuators: %.2f, %.2f", 
	// 		(double)_actuators.control[LEFT], 
	// 		(double)_actuators.control[RIGHT]
	// 		);
	// }

	// second stage: pick up pwm duty cycle
	float pcur = 0, pdes = 0;
	if (rollacc_des < 0) {
		pcur = 100 * (_actuators.control[RIGHT] - _min_control) / (MAX_CONTROL - _min_control);

		if (rollacc_des < _stable.rollacc) {
			pdes = pcur + _two_stage_step;
		} else {
			pdes = pcur - _two_stage_step;
		}

		if (pdes < 0) pdes = 0;
		if (pdes > 100) pdes = 100;

		if (two_stage_debug_counter < TWO_STAGE_DEBUG_NUM) {
			warnx("pcur: %.2f, pdes: %.2f", 
				(double)pcur,
				(double)pdes
				);
		}


		// set pwm
		_actuators.control[RIGHT] = (MAX_CONTROL - _min_control) * pdes / 100 + _min_control;
		_actuators.control[LEFT] = _min_control;
	} else {
		pcur = 100 * (_actuators.control[LEFT] - _min_control) / (MAX_CONTROL - _min_control);

		if (rollacc_des > _stable.rollacc) {
			pdes = pcur + _two_stage_step;
		} else {
			pdes = pcur - _two_stage_step;
		}

		if (pdes < 0) pdes = 0;
		if (pdes > 100) pdes = 100;

		if (two_stage_debug_counter < TWO_STAGE_DEBUG_NUM) {
			warnx("pcur: %.2f, pdes: %.2f", 
				(double)pcur,
				(double)pdes
				);
		}


		_actuators.control[LEFT] = (MAX_CONTROL - _min_control) * pdes / 100 + _min_control;
		_actuators.control[RIGHT] = _min_control;
	}

	_stable.roll 		= _v_att.roll;
	_stable.rollspeed 	= _v_att.rollspeed;
	_stable.rollacc_des 	= rollacc_des;
	orb_publish(ORB_ID(stable_duckling), _stable_pub, &_stable);

	if (two_stage_debug_counter < TWO_STAGE_DEBUG_NUM) {
		warnx("actuators: %.2f, %.2f\n", 
			(double)_actuators.control[LEFT], 
			(double)_actuators.control[RIGHT]
			);
	}
	two_stage_debug_counter++;
}

void StableDuckling::control() 
{
	switch (_mode) {
	case TWO_STAGE:
		two_stage();
		break;
	case PID:
		_roll_integral += _v_att.roll;

		_thrust = 	_kp * _v_att.roll +
				_ki * _roll_integral +
				_kd * _v_att.rollspeed;

		apply_thrust();
		break;
	case IMPULSE:
		if (_impulse_index == _impulse_length) {
			_actuators.control[LEFT] = MIN_CONTROL;
			_actuators.control[RIGHT] = MIN_CONTROL;
			_impulse_index = 0;
			_mode = SILENCE;
		} else {
			_actuators.control[LEFT] = _impulse_ampl;
			_actuators.control[RIGHT] = MIN_CONTROL;
			_impulse_index++;
		}
		break;
	case SIN: {
		double dt = CONTROL_INTERVAL / 1000.0;
		_sin_phase += 2*M_PI*_sin_freq*dt;
		while (_sin_phase > 2*M_PI) {
			_sin_phase -= 2*M_PI;
		}
		_thrust = _sin_mag * sin(_sin_phase);
		apply_thrust();
		break;
	}	
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
	/* roll increases clockwise if we look from tail, 
	   consequently, when thrust is positive,
	   we should turn right engine
	*/
	if (_thrust > 0) {
		_actuators.control[RIGHT] = (_thrust - _b_thrust) / _k_thrust;
		if (_actuators.control[RIGHT] > MAX_CONTROL) 
			_actuators.control[RIGHT] = MAX_CONTROL;
		_actuators.control[LEFT] = _min_control;
	} else {
		_actuators.control[LEFT] = (-_thrust - _b_thrust) / _k_thrust;
		if (_actuators.control[LEFT] > MAX_CONTROL) 
			_actuators.control[LEFT] = MAX_CONTROL;
		_actuators.control[RIGHT] = _min_control;
	}
}

void StableDuckling::analyse_command() 
{
	switch ((StableDucklingCommand)_v_cmd.command) {
	case SET_MODE:
		_mode = (StableDucklingMode)_v_cmd.param1;
		stop_motors();
		warnx("mode %d set", _mode);
		if (_mode == PID) {
			_roll_integral = 0;
		}
		break;
	case SET_PWM: {
		int motor = (int)_v_cmd.param1;
		_actuators.control[motor] = _v_cmd.param2;
		break;
	}
		// _actuators.control[0] = _v_cmd.param1;
		// _actuators.control[1] = _v_cmd.param2;
		// warnx("pwm (%.2f %.2f) set", 
		// 	(double)_actuators.control[0], 
		// 	(double)_actuators.control[1]);
		
	case SET_PID_COEFFS:
		_kp = _v_cmd.param1;
		_ki = _v_cmd.param2;
		_kd = _v_cmd.param3;

		_stable.k_p = _kp;
		_stable.k_i = _ki;
		_stable.k_d = _kd;
		orb_publish(ORB_ID(stable_duckling), _stable_pub, &_stable);

		warnx("pid coeffs (%.2f %.2f %.2f) set",
			(double)_kp,
			(double)_ki,
			(double)_kd);
		break;
	case SET_IMP_PARAMS:
		_impulse_length = (int)_v_cmd.param1;
		_impulse_ampl = _v_cmd.param2;
		break;
	case SET_ANCHOR_ROLL:
		_anchor_roll_set = false;
		_anchor_length = (int)_v_cmd.param1;
		break;
	case SET_MIN_CONTROL:
		_min_control = _v_cmd.param1;
		break;
	case SET_SIN_PARAMS:
		_sin_mag = (double)_v_cmd.param1;
		_sin_freq = (double)_v_cmd.param2;
		break;
	case SET_TWO_PARAMS:
		_tau = _v_cmd.param1;
		_two_stage_step = _v_cmd.param2;

		_stable.tau = _tau;
		_stable.step = _two_stage_step;
		orb_publish(ORB_ID(stable_duckling), _stable_pub, &_stable);

		warnx("tau: %.2f, max_step: %.2f", (double)_tau, (double)_two_stage_step);
		break;
	default:
		break;
	}
	toggle_led();
}

void StableDuckling::toggle_led()
{
	int fd = open(ws2812_path, O_WRONLY);
	write(fd, LED_COLOR[_cur_led_color], 3);
	close(fd);
	_cur_led_color ^= 0x01;
}

void
StableDuckling::task_main_trampoline(int argc, char *argv[])
{
	stable_duckling::g_duckling->task_main();
}


void
StableDuckling::task_main()
{
	toggle_led();

	_v_cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	orb_set_interval(_v_cmd_sub, CONTROL_INTERVAL);

	/* subscribe to sensor_combined topic */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(_v_att_sub, CONTROL_INTERVAL);

	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

	_stable.anchor_roll = _anchor_roll;
	_stable.k_p = _kp;
	_stable.k_i = _ki;
	_stable.k_d = _kd;
	_stable.roll = 0;
	_stable.rollspeed = 0;
	_stable.rollacc = 0;
	_stable.rollacc_des = 0;
	_stable.tau = _tau;
	_stable.step = _two_stage_step;
	_stable_pub = orb_advertise(ORB_ID(stable_duckling), &_stable);

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

			_stable.rollacc = (_v_att.rollspeed - _rollspeed_prev) / DT;

			anchor();
			control();

			_rollspeed_prev = _v_att.rollspeed;
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

const int StableDuckling::BACK = 2;
const int StableDuckling::LEFT = 0;
const int StableDuckling::RIGHT = 1;
const float StableDuckling::MIN_CONTROL = -1.0f;
const float StableDuckling::MAX_CONTROL = 1.0f;
const int StableDuckling::CONTROL_INTERVAL = 10; 	// ms

const char StableDuckling::LED_COLOR[2][3] = {
	{0xff, 0x00, 0x00}, // green
	{0xff, 0xff, 0xff}  // white
};

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