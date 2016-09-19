#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
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

#include <drivers/ws2812/ws2812.h>

#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0)

/**
 * Trainer app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int trainer_main(int argc, char *argv[]);


class Trainer 
{
public:
	/**
	 * Constructor
	 */
	Trainer();

	/**
	 * Destructor, also kills the main task
	 */
	~Trainer();

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

	struct vehicle_attitude_s	_v_att;			/**< vehicle attitude */

	int 	_led_num;
	double 	_max_angle;
	int 	_label_len;
	int 	_label_color;
	double 	_dps_per_led;
	int 	_tail_color;
	bool 	_direct_mode;
	bool 	_tail_behind;

	static const int CONTROL_INTERVAL;

	void fire_leds();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();	
};

namespace trainer 
{
	Trainer *g_trainer = nullptr;
}

Trainer::Trainer() :
	_task_should_exit(false),
	_control_task(-1),
	_v_att_sub(-1)
{
	FILE *settings_file = fopen("/fs/microsd/etc/trainer.txt", "r");
	if (settings_file == nullptr) {
		warnx("failed to open settings file");
	} else {
#define OPTION_LEN 20
#define ARG_LEN 20
#define BUF_LEN (OPTION_LEN+ARG_LEN)
		char buf[BUF_LEN], option[OPTION_LEN], arg[ARG_LEN];
		while (fgets(buf, BUF_LEN, settings_file)) {
			int buf_len = strlen(buf);
			char *space_ptr = strchr(buf, ' ');
			int option_len = space_ptr - buf;

			if (option_len > OPTION_LEN-1) {
				errx(0, "too long option");
			}
			memcpy(option, buf, option_len);
			option[option_len] = '\0';

			int arg_len = buf_len - option_len - 1;
			if (arg_len > ARG_LEN-1) {
				errx(0, "too long argument");
			}
			memcpy(arg, buf + option_len + 1, arg_len);
			arg[arg_len] = '\0';


			if (!strcmp(option, "led_num")) {
				_led_num = atoi(arg);
				printf("led_num: %d\n", _led_num);
			} else if (!strcmp(option, "max_angle")) {
				_max_angle = atof(arg);
				printf("max_angle: %.2f\n", _max_angle);
			} else if (!strcmp(option, "label_len")) {
				_label_len = atoi(arg);
				printf("label_len: %d\n", _label_len);
			} else if (!strcmp(option, "label_color")) {
				_label_color = strtol(arg, 0, 16);
				printf("label_color: %d\n", _label_color);
				printf("green: %d, red: %d, blue: %d\n", 
					(uint8_t)((_label_color & (0xff << 8*0)) >> 8*0),
					(uint8_t)((_label_color & (0xff << 8*1)) >> 8*1),
					(uint8_t)((_label_color & (0xff << 8*2)) >> 8*2)
					);
			} else if (!strcmp(option, "dps_per_led")) {
				_dps_per_led = atof(arg);
				printf("dps_per_led: %.2f\n", _dps_per_led);
			} else if (!strcmp(option, "tail_color")) {
				_tail_color = strtol(arg, 0, 16);
				printf("tail_color: %d\n", _tail_color);
			} else if (!strcmp(option, "direct_mode")) {
				_direct_mode = (bool)atoi(arg);
				printf("direct_mode: %d\n", _direct_mode);
			} else if (!strcmp(option, "tail_behind")) {
				_tail_behind = (bool)atoi(arg);
				printf("tail_behind: %d\n", _tail_behind);
			}
		}
		fclose(settings_file);		
	}


}

Trainer::~Trainer()
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

	trainer::g_trainer = nullptr;
}

void Trainer::fire_leds()
{
	int mid = _led_num / 2;
	float rads_per_led = _max_angle * DEG_TO_RAD / mid;

	int bias = _direct_mode ? 1 : -1;
	int label = mid + bias * _v_att.roll / rads_per_led;
	int label_begin = label - _label_len / 2;
	int label_end = label + _label_len / 2;

	if (label_begin < 0) {
		label_begin = 0;
		label_end = label_begin + _label_len - 1;
	}
	if (label_end > _led_num - 1) {
		label_end = _led_num - 1;
		label_begin = label_end - _label_len + 1;
	}
	
	
	int tail_begin = -1;
	int tail_end = -1;

	float angle_rate = (_v_att.rollspeed > 0) ? _v_att.rollspeed : -_v_att.rollspeed;
	int tail_len = angle_rate / (float)(_dps_per_led * DEG_TO_RAD);

	if (_direct_mode) {
		if (_tail_behind) {
			if (_v_att.rollspeed > 0) {
				goto tail_left;
			} else {
				goto tail_right;
			} 
		} else {
			if (_v_att.rollspeed > 0) {
				goto tail_right;
			} else {
				goto tail_left;
			} 
		}
	} else {
		if (_tail_behind) {
			if (_v_att.rollspeed > 0) {
				goto tail_right;
			} else {
				goto tail_left;
			} 
		} else {
			if (_v_att.rollspeed > 0) {
				goto tail_left;
			} else {
				goto tail_right;
			} 
		}
	}

tail_left:
	tail_begin = label_begin - tail_len;
	tail_end = label_begin - 1;
	goto fire;
tail_right:
	tail_begin = label_end + 1;
	tail_end = label_end + tail_len;
	goto fire;
fire:
	if (tail_begin < 0) tail_begin = 0;
	if (tail_end > _led_num - 1) tail_end = _led_num - 1;
	

	char strip[_led_num*3];
	memset(strip, 0x00, _led_num*3);

	// label
	for (int i = label_begin; i <= label_end; i++) {	
		strip[3*i+0] = (uint8_t)((_label_color & (0xff << 8*0)) >> 8*0);
		strip[3*i+1] = (uint8_t)((_label_color & (0xff << 8*1)) >> 8*1);	
		strip[3*i+2] = (uint8_t)((_label_color & (0xff << 8*2)) >> 8*2);
	}

	// tail
	for (int i = tail_begin; i <= tail_end; i++) {	
		strip[3*i+0] = (uint8_t)((_tail_color & (0xff << 8*0)) >> 8*0);
		strip[3*i+1] = (uint8_t)((_tail_color & (0xff << 8*1)) >> 8*1);	
		strip[3*i+2] = (uint8_t)((_tail_color & (0xff << 8*2)) >> 8*2);
	}

	int fd = open(ws2812_path, O_WRONLY);
	write(fd, strip, _led_num*3);
	close(fd);
}

void
Trainer::task_main_trampoline(int argc, char *argv[])
{
	trainer::g_trainer->task_main();
}

void
Trainer::task_main()
{
	/* subscribe to sensor_combined topic */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(_v_att_sub, CONTROL_INTERVAL);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for sensor update of 1 file descriptor for 10 ms  */
		int poll_ret = poll(fds, 1, CONTROL_INTERVAL);

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
			
			fire_leds();
		}

	}

	_control_task = -1;
	return;
}

int
Trainer::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("trainer",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&Trainer::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

const int Trainer::CONTROL_INTERVAL = 10; 	// ms

int trainer_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: trainer {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (trainer::g_trainer != nullptr) {
			warnx("already running");
			return 1;
		}

		trainer::g_trainer = new Trainer;

		if (trainer::g_trainer == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != trainer::g_trainer->start()) {
			delete trainer::g_trainer;
			trainer::g_trainer = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (trainer::g_trainer == nullptr) {
			warnx("not running");
			return 1;
		}

		delete trainer::g_trainer;
		trainer::g_trainer = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (trainer::g_trainer) {
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