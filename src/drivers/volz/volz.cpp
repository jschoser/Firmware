/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <float.h>
#include <math.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/esc_status.h>
#include <termios.h>

/***
 * Driver for the Volz DA22 servo using RS485. Position commands with 6 bytes each are written to the TELEM2 UART port.
 * The module also supports multiple command-line argument that can be used to assign servo IDs, failsafe timeout, ...
 * This version of the driver also contains some methods to retrieve the response signal of the servos, and recognise
 * if they correspond to expected values. However, these methods are currently not being called anywhere since testing
 * for sending commands still has to succeed.
 * A lot of inspiration for the structure of this program was taken from the DShot driver.
 */


using namespace time_literals; // TODO: necessary?

#if !defined(BOARD_HAS_PWM) // TODO: necessary?
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif


class VolzOutput : public cdev::CDev, public ModuleBase<VolzOutput>, public OutputModuleInterface
{
public:
    static constexpr int VOLZ_ID_UNKNOWN = 0x1F;  // ID to be used to send a command to all connected servos
    static constexpr int VOLZ_CMD_LEN = 6;  // each command contains six bytes

	VolzOutput();
	virtual ~VolzOutput();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();


	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	// IDs to characterise the commands and responses for different purposes
	typedef enum{
        POS_CMD = 0xDD,
        SET_ACTUATOR_ID = 0xAA,
        SET_FAILSAFE_TIMEOUT = 0xCC,
        SET_CURRENT_POS_AS_FAILSAFE_POS = 0xBC,
        SET_CURRENT_POS_AS_ZERO = 0x99
	} volz_command_t;

    typedef enum{
        POS_CMD_RESP = 0x44,
        SET_ACTUATOR_ID_RESP = 0x55,
        SET_FAILSAFE_TIMEOUT_RESP = 0x66,
        SET_CURRENT_POS_AS_FAILSAFE_POS_RESP = 0x5C,
        SET_CURRENT_POS_AS_ZERO_RESP = 0x65
    } volz_response_t;

    struct Command {
        hrt_abstime last_send_time{0};

        uint8_t cmd[VOLZ_CMD_LEN];  // command bytes
        uint8_t exp_resp[VOLZ_CMD_LEN];  // expected response bytes
        bool resp_unknown[VOLZ_CMD_LEN] = {false, false, false, false, false, false};  // true if we don't know what to expect

        // TODO: maybe remove repetition feature
        int num_repetitions{0};
        bool valid() const { return num_repetitions > 0; }
        void clear() { num_repetitions = 0; }

        // check if a given set of response bytes is valid for this command
        // TODO: make this check whether the checksum of the received command makes sense
        bool valid_response(uint8_t resp[]) {
            for (int i = 0; i < VOLZ_CMD_LEN; i ++) {
                if (!resp_unknown[i] && (resp[i] != exp_resp[i])) {
                    return false;
                }
            }
            return true;
        }
    };

	/**
	 * Send a Volz command to one or all servos
	 * This is expected to be called from another thread.
	 * @param command is the command to be sent
	 * @return 0 on success, <0 error otherwise
	 */
	int sendCommandThreadSafe(Command command);

private:

	void Run() override;

	static constexpr uint16_t DISARMED_VALUE = 0;
    static constexpr uint16_t MIN_VALUE = DISARMED_VALUE + 1;
    static constexpr uint16_t MAX_VALUE = MIN_VALUE + 1000;
    uint16_t _idle_value[MAX_ACTUATORS] {};

    static constexpr int VOLZ_POS_MIN = 0x0060;  // value that corresponds to "minimum" position in Volz protocol
    static constexpr int VOLZ_POS_CENTER = 0x1000;  // value that corresponds to center position in Volz protocol

    static constexpr int RESP_QUEUE_LEN = MAX_ACTUATORS + 1;  // we store expected responses for each servo and an additional channel (for unknown ID)

	int _fd{-1};
	const char *_port = "/dev/ttyS2";  // TELEM2 port

	MixingOutput _mixing_output{DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
    int update_rate{50};  // limit update rate, which seems to reduce the times the driver crashes

	uORB::Subscription _param_sub{ORB_ID(parameter_update)};

	// fields used to send command-line commands thread-safe
    Command _current_command;
    px4::atomic<Command *> _new_command{nullptr};

	Command sent_commands[RESP_QUEUE_LEN];  // store sent commands so that incoming responses can be related to them
	bool waiting_for_resp[RESP_QUEUE_LEN] = {false};

    uint8_t _resp_buffer[VOLZ_CMD_LEN];  // TODO: Should this really be kept between loops or emptied every time?
    int _resp_position{0};

	unsigned	_num_outputs{0};
	int		_class_instance{-1}; // TODO: what does this do?

	perf_counter_t	_cycle_perf;

	void		update_params();

    int			pwm_ioctl(file *filp, int cmd, unsigned long arg);  // taken from dshot.cpp
    int		capture_ioctl(file *filp, int cmd, unsigned long arg);

    // helper functions to generate Volz commands
    static int generate_crc(int cmd, int actuator_id, int arg_1, int arg_2);
    static int highbyte(int value);
    static int lowbyte(int value);

    // functions to generate Volz commands
    static Command pos_cmd(int pos, int id = VOLZ_ID_UNKNOWN, int num_repetitions = 1);
    static Command set_actuator_id(int new_id, int id = VOLZ_ID_UNKNOWN, int num_repetitions = 1);
    static Command set_failsafe_timeout(float timeout, int id = VOLZ_ID_UNKNOWN, int num_repetitions = 1);
    static Command set_current_pos_as_failsafe(int id = VOLZ_ID_UNKNOWN, int num_repetitions = 1);
    static Command set_current_pos_as_zero(int id = VOLZ_ID_UNKNOWN, int num_repetitions = 1);

    // write a command to the open port
    bool write_command(Command command);

    // check for response signals. Currently never called
    bool update_telemetry();

	VolzOutput(const VolzOutput &) = delete;
	VolzOutput operator=(const VolzOutput &) = delete;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VOLZ_CONFIG>) _param_volz_config // TODO: replace with actual parameters
	)
};

VolzOutput::VolzOutput() :
	CDev("/dev/volz"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_mixing_output.setAllDisarmedValues(DISARMED_VALUE);
	_mixing_output.setAllMinValues(MIN_VALUE);
	_mixing_output.setAllMaxValues(MAX_VALUE);

    _mixing_output.setMaxTopicUpdateRate(1000000 / update_rate);

    for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
        _idle_value[i] = (MAX_VALUE + MIN_VALUE) / 2;
    }
}

VolzOutput::~VolzOutput()
{
	/* make sure outputs are off */
	// TODO: Disarm all servos

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance); // TODO: what does this do?

	if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
	}

	perf_free(_cycle_perf);
}

int
VolzOutput::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	// TODO: what does this do?
	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	// Getting initial parameter values
	update_params();

	// TODO: Find out how this code works and what the flags mean (taken from TFMINI.cpp). Check out dshot's telemetry.cpp

    do { // create a scope to handle exit conditions using break
        // open fd
        _fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (_fd < 0) {
            PX4_ERR("Error opening fd");
            return -1;
        }

        // baudrate 115200, 8 bits, no parity, 1 stop bit
        unsigned speed = B115200;
        termios uart_config{};
        int termios_state{};

        tcgetattr(_fd, &uart_config);

        // clear ONLCR flag (which appends a CR for every LF)
        uart_config.c_oflag &= ~ONLCR;

        // set baud rate
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d ISPD", termios_state);
            ret = -1;
            break;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d OSPD\n", termios_state);
            ret = -1;
            break;
        }

        if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
            PX4_ERR("baud %d ATTR", termios_state);
            ret = -1;
            break;
        }

        uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
        uart_config.c_cflag &= ~CSIZE;
        uart_config.c_cflag |= CS8;			// 8-bit characters
        uart_config.c_cflag &= ~PARENB;			// no parity bit
        uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
        uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

        // setup for non-canonical mode
        uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        uart_config.c_oflag &= ~OPOST;

        // fetch bytes as they become available
        uart_config.c_cc[VMIN] = 1;
        uart_config.c_cc[VTIME] = 1;

        if (_fd < 0) {
            PX4_ERR("FAIL: fd");
            ret = -1;
            break;
        }
    } while (0);

    // close the fd
    ::close(_fd);
    _fd = -1;

    PX4_INFO("started");

	ScheduleNow();

	return 0;
}

int
VolzOutput::task_spawn(int argc, char *argv[])
{
	VolzOutput *instance = new VolzOutput();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VolzOutput::sendCommandThreadSafe(Command command)
{
	_new_command.store(&command);

	PX4_INFO("Starting command upload");
	// wait until main thread processed it
	while (_new_command.load()) {
		px4_usleep(1000);
	}
    PX4_INFO("Finished command upload");
	return 0;
}

void VolzOutput::mixerChanged()
{
	// This shouldn't happen. Do nothing
}

// TODO: Make sure that the servos are set to failsafe positions when disarmed, and not to zero
bool VolzOutput::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				unsigned num_outputs, unsigned num_control_groups_updated)
{
    if (_fd < 0) {
        return false;
    }

	if (!_mixing_output.armed().prearmed && !_mixing_output.armed().armed) {
		for (unsigned i = 0; i < num_outputs; i++) {
            Command command = pos_cmd(_idle_value[i], i + 1);  // Set actuators to idle positions
            write_command(command);
		}

        // check if we have other commands to send
        if (_current_command.valid()) {
            write_command(_current_command);
        }

	} else {
		for (unsigned i = 0; i < num_outputs; i++) {
		    Command command;
			if (outputs[i] == DISARMED_VALUE) {
			    command = pos_cmd(_idle_value[i], i + 1); // set disarmed servos to idle position

			} else {

                command = pos_cmd(outputs[i], i + 1); // set non-disarmed servos to their correct positions
			}
            write_command(command);  // write the command to the port
		}

		// clear CLI commands when motors are running
		_current_command.clear();
	}

	return true;
}

int VolzOutput::highbyte(int value) {
    return (value >> 8) & 0xff;
}

int VolzOutput::lowbyte(int value) {
    return value & 0xff;
}

// generate command checksum for Volz protocol
int VolzOutput::generate_crc(int cmd, int actuator_id, int arg_1, int arg_2)	{
    unsigned short int crc=0xFFFF; // init value of result
    int command[4]={cmd,actuator_id,arg_1,arg_2}; // command, ID, argument1, argument 2
    int x,y;
    for(x=0; x<4; x++)	{
        crc= ( ( command[x] <<8 ) ^ crc);

        for ( y=0; y<8; y++ )	{

            if ( crc & 0x8000 )
                crc = (crc << 1) ^ 0x8005;

            else
                crc = crc << 1;

        }
    }

    return crc;
}

VolzOutput::Command VolzOutput::pos_cmd(int pos, int id, int num_repetitions) {
    int arg = VOLZ_POS_CENTER + (pos - (MAX_VALUE + MIN_VALUE) / 2) * 2 / (MAX_VALUE - MIN_VALUE) * (VOLZ_POS_CENTER - VOLZ_POS_MIN);
    uint8_t arg_1 = highbyte(arg);
    uint8_t arg_2 = lowbyte(arg);
    int crc_cmd = generate_crc(volz_command_t::POS_CMD, id, arg_1, arg_2);
    int crc_resp = generate_crc(volz_response_t::POS_CMD_RESP, id, arg_1, arg_2);

    Command command;
    command.cmd[0] = volz_command_t::POS_CMD;
    command.cmd[1] = id;
    command.cmd[2] = arg_1;
    command.cmd[3] = arg_2;
    command.cmd[4] = highbyte(crc_cmd);
    command.cmd[5] = lowbyte(crc_cmd);

    command.exp_resp[0] = volz_response_t::POS_CMD_RESP;
    command.exp_resp[1] = id;
    command.exp_resp[2] = arg_1;
    command.exp_resp[3] = arg_2;
    command.exp_resp[4] = highbyte(crc_resp);
    command.exp_resp[5] = lowbyte(crc_resp);

    if (id == VOLZ_ID_UNKNOWN) {
        command.resp_unknown[1] = true;
        command.resp_unknown[4] = true;
        command.resp_unknown[5] = true;
    }

    command.num_repetitions = num_repetitions;

    return command;
}
VolzOutput::Command VolzOutput::set_actuator_id(int new_id, int id, int num_repetitions) {
    int crc_cmd = generate_crc(volz_command_t::SET_ACTUATOR_ID, id, new_id, new_id);
    int crc_resp = generate_crc(volz_response_t::SET_ACTUATOR_ID_RESP, new_id, new_id, new_id);

    Command command;
    command.cmd[0] = volz_command_t::SET_ACTUATOR_ID;
    command.cmd[1] = id;
    command.cmd[2] = new_id;
    command.cmd[3] = new_id;
    command.cmd[4] = highbyte(crc_cmd);
    command.cmd[5] = lowbyte(crc_cmd);

    command.exp_resp[0] = volz_response_t::SET_ACTUATOR_ID_RESP;
    command.exp_resp[1] = new_id;
    command.exp_resp[2] = new_id;
    command.exp_resp[3] = new_id;
    command.exp_resp[4] = highbyte(crc_resp);
    command.exp_resp[5] = lowbyte(crc_resp);

    command.num_repetitions = num_repetitions;

    return command;
}
VolzOutput::Command VolzOutput::set_failsafe_timeout(float timeout, int id, int num_repetitions) {
    int arg = (int)(timeout * 10); // convert from seconds to x100 ms
    int crc_cmd = generate_crc(volz_command_t::SET_FAILSAFE_TIMEOUT, id, arg, arg);
    int crc_resp = generate_crc(volz_response_t::SET_FAILSAFE_TIMEOUT_RESP, id, arg, arg);

    Command command;
    command.cmd[0] = volz_command_t::SET_FAILSAFE_TIMEOUT;
    command.cmd[1] = id;
    command.cmd[2] = arg;
    command.cmd[3] = arg;
    command.cmd[4] = highbyte(crc_cmd);
    command.cmd[5] = lowbyte(crc_cmd);

    command.exp_resp[0] = volz_response_t::SET_FAILSAFE_TIMEOUT_RESP;
    command.exp_resp[1] = id;
    command.exp_resp[2] = arg;
    command.exp_resp[3] = arg;
    command.exp_resp[4] = highbyte(crc_resp);
    command.exp_resp[5] = lowbyte(crc_resp);

    if (id == VOLZ_ID_UNKNOWN) {
        command.resp_unknown[1] = true;
        command.resp_unknown[4] = true;
        command.resp_unknown[5] = true;
    }

    command.num_repetitions = num_repetitions;

    return command;
}
VolzOutput::Command VolzOutput::set_current_pos_as_failsafe(int id, int num_repetitions) {
    int crc_cmd = generate_crc(volz_command_t::SET_CURRENT_POS_AS_FAILSAFE_POS, id, 0x00, 0x00);
    int crc_resp = generate_crc(volz_response_t::SET_CURRENT_POS_AS_FAILSAFE_POS_RESP, id, 0x00, 0x00);

    Command command;
    command.cmd[0] = volz_command_t::SET_FAILSAFE_TIMEOUT;
    command.cmd[1] = id;
    command.cmd[2] = 0x00;
    command.cmd[3] = 0x00;
    command.cmd[4] = highbyte(crc_cmd);
    command.cmd[5] = lowbyte(crc_cmd);

    command.exp_resp[0] = volz_response_t::SET_FAILSAFE_TIMEOUT_RESP;
    command.exp_resp[1] = id;
    command.exp_resp[2] = 0x00;
    command.exp_resp[3] = 0x00;
    command.exp_resp[4] = highbyte(crc_resp);
    command.exp_resp[5] = lowbyte(crc_resp);

    command.resp_unknown[1] = id == VOLZ_ID_UNKNOWN;
    command.resp_unknown[2] = true;
    command.resp_unknown[3] = true;
    command.resp_unknown[4] = true;
    command.resp_unknown[5] = true;

    command.num_repetitions = num_repetitions;

    return command;
}
VolzOutput::Command VolzOutput::set_current_pos_as_zero(int id, int num_repetitions) {
    int crc_cmd = generate_crc(volz_command_t::SET_CURRENT_POS_AS_ZERO, id, 0x00, 0x00);
    int crc_resp = generate_crc(volz_response_t::SET_CURRENT_POS_AS_ZERO_RESP, id, 0x00, 0x00);

    Command command;
    command.cmd[0] = volz_command_t::SET_CURRENT_POS_AS_ZERO;
    command.cmd[1] = id;
    command.cmd[2] = 0x00;
    command.cmd[3] = 0x00;
    command.cmd[4] = highbyte(crc_cmd);
    command.cmd[5] = lowbyte(crc_cmd);

    command.exp_resp[0] = volz_response_t::SET_CURRENT_POS_AS_ZERO_RESP;
    command.exp_resp[1] = id;
    command.exp_resp[2] = highbyte(VOLZ_POS_CENTER);
    command.exp_resp[3] = lowbyte(VOLZ_POS_CENTER);
    command.exp_resp[4] = highbyte(crc_resp);
    command.exp_resp[5] = lowbyte(crc_resp);

    if (id == VOLZ_ID_UNKNOWN) {
        command.resp_unknown[1] = true;
        command.resp_unknown[4] = true;
        command.resp_unknown[5] = true;
    }

    command.num_repetitions = num_repetitions;

    return command;
}

// TODO: do something with the return value of this function
bool VolzOutput::write_command(Command command) {
    if (_fd < 0) {
        return false;
    }

    --command.num_repetitions;
    command.last_send_time = hrt_absolute_time();
    int ret = ::write(_fd, command.cmd, sizeof(command.cmd));

    if (ret < 0) {
        return false;
    }

    int idx = command.cmd[1];

    if (idx == VOLZ_ID_UNKNOWN) {
        idx = RESP_QUEUE_LEN - 1;
    }

    // currently not used
//    if (waiting_for_resp[idx]) {
//        PX4_ERR("Received no response to command on servo %d before sending new command", idx);
//    }
//
//    sent_commands[idx] = command;
//    waiting_for_resp[idx] = true;

    return true;
}

// inspired by dshot's telemetry.cpp. Currently not used
bool VolzOutput::update_telemetry() {
    if (_fd < 0) {
        return false;
    }

    // TODO: the ioctl call here causes conflicts with the ioctl defined in this file. Fix this
    // read from the uart. This must be non-blocking, so check first if there is data available
//    int bytes_available = 0;
//    int ret = ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
//
//    if (ret != 0 || bytes_available <= 0) {
//        // no data available. TODO: Check for a timeout
//        return false;
//    }

    const int buf_length = VOLZ_CMD_LEN;
    uint8_t buf[buf_length];
    bool response_matched = false;

    int num_read = ::read(_fd, buf, buf_length);

    for (int i = 0; i < num_read; i++) {
        _resp_buffer[_resp_position++] = buf[i];

        if (_resp_position == VOLZ_CMD_LEN) {
            for (int j = 0; j < RESP_QUEUE_LEN; j++) {
                // TODO: this will almost certainly throw an error due to not being initialised
                if (waiting_for_resp[j] && sent_commands[j].valid_response(_resp_buffer)) {
                    waiting_for_resp[j] = false;
                    _resp_position = 0;
                    response_matched = true;
                }
            }
            if (!response_matched) {
                // TODO: do something
                PX4_ERR("Received unexpected response");
            }
        }
    }
    return true;
}

void
VolzOutput::Run()
{
    // fds initialized?
    if (_fd < 0) {
        // open fd
        _fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK); // TODO: Check if flags are right
    }

	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		::close(_fd);
		_fd = -1;

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	// update_telemetry();

	if (_param_sub.updated()) {
		update_params();
	}

	// new command?
	// TODO: Make this if statement work somehow
	// if (!_current_command.valid()) {
	    Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
			// TODO: don't write it here, but in updateOutputs instead
			write_command(_current_command);
		}
	// }

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void VolzOutput::update_params()
{
    // TODO: Make this function actually do something useful, i.e. come up with parameters for this driver
	parameter_update_s pupdate;
	_param_sub.update(&pupdate);

	updateParams();
}


int
VolzOutput::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	/* try it as a Capture ioctl next */
	ret = capture_ioctl(filp, cmd, arg);
    if (ret != -ENOTTY) {
        return ret;
    }

    ret = pwm_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
VolzOutput::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("volz ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_GET_COUNT:
        *(unsigned *)arg = 5;
		break;

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters.
			 * TODO: Implement this if you want
			 */
			break;
		}

	case PWM_SERVO_SET_MODE: {
            // TODO: Implement this if you want
			break;
		}

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

int
VolzOutput::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = -ENOTTY;
	// TODO: Find out what this method is good for in dshot
	return ret;
}

int VolzOutput::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	int actuator_id = VOLZ_ID_UNKNOWN;
	float pos = DISARMED_VALUE;
	int new_id = VOLZ_ID_UNKNOWN;
	float timeout = -1;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:p:n:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			actuator_id = strtol(myoptarg, nullptr, 10);
			break;
        case 'p':
            pos = strtol(myoptarg, nullptr, 10);
            break;
        case 'n':
            new_id = strtol(myoptarg, nullptr, 10);
            break;
        case 't':
            timeout = strtol(myoptarg, nullptr, 10);
            break;

		default:
			return print_usage("unrecognized flag");
		}
	}

    if (!is_running()) {
        PX4_ERR("module not running");
        return -1;
    }

    if (!strcmp(verb, "pos_cmd")) {
        if (pos < MIN_VALUE || pos > MAX_VALUE) {
            PX4_ERR("enter a position between %i and %i", MIN_VALUE, MAX_VALUE);
            return -1;
        } else {
            return get_instance()->sendCommandThreadSafe(pos_cmd(pos, actuator_id, 1));
        }
    } else if (!strcmp(verb, "set_id")) {
        if (new_id == VOLZ_ID_UNKNOWN) {
            PX4_ERR("set an ID between 0x01 and 0x1E");
            return -1;
        } else {
            return get_instance()->sendCommandThreadSafe(set_actuator_id(new_id, actuator_id, 1));
        }
    } else if (!strcmp(verb, "set_fs_timeout")) {
        if (timeout < 0) {
            PX4_ERR("set a timeout between 0 and 12.7 seconds");
            return -1;
        } else {
            return get_instance()->sendCommandThreadSafe(set_failsafe_timeout(timeout, actuator_id, 1));
        }
    } else if (!strcmp(verb, "set_pos_as_fs")) {
        return get_instance()->sendCommandThreadSafe(set_current_pos_as_failsafe(actuator_id, 1));
    } else if (!strcmp(verb, "set_pos_as_zero")) {
        return get_instance()->sendCommandThreadSafe(set_current_pos_as_zero(actuator_id, 1));
    }

    if (!is_running()) { // TODO: What does this do?
		int ret = VolzOutput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int VolzOutput::print_status()
{
    PX4_INFO("running");
	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	return 0;
}

int VolzOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the Volz output driver for Volz DA22-Series actuators. It is similar to the DShot output driver, from which it
borrows heavily.

It supports:
- sending Volz commands via CLI:
    - new position command
    - set actuator ID
    - set failsafe timeout
    - set current position as new failsafe position
    - set current position as new zero
- integrating the actuator into the control pipeline, i.e. carry out commands by the flight controller. For this to
work, the actuators should have IDs corresponding to the index of their control value in the mixer output (starting
from 1).

For actuator IDs, start counting from 0x01 upwards. The actuators should be numbered in the order in which they
occur in the mixer file.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("volz", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

    PRINT_MODULE_USAGE_COMMAND_DESCR("pos_cmd", "Command actuator to given position");
    PRINT_MODULE_USAGE_PARAM_INT('i', VOLZ_ID_UNKNOWN, 0x01, 0x1E, "Actuator ID", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('p', DISARMED_VALUE, MIN_VALUE, MAX_VALUE, "Position", false);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_id", "Set new actuator ID");
    PRINT_MODULE_USAGE_PARAM_INT('i', VOLZ_ID_UNKNOWN, 0x01, 0x1E, "Old actuator ID", true);
    PRINT_MODULE_USAGE_PARAM_INT('n', VOLZ_ID_UNKNOWN, 0x01, 0x1E, "New actuator ID", false);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_fs_timeout", "Set amount of seconds after which the actuator "
                                                       "goes into failsafe position, if it does not receive a valid"
                                                       "command");
    PRINT_MODULE_USAGE_PARAM_INT('i', VOLZ_ID_UNKNOWN, 0x01, 0x1E, "Actuator ID", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('t', -1, 0x00, 0x7F, "Timeout (s)", false);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_pos_as_fs", "Set current actuator position as new failsafe "
                                                      "positon");
    PRINT_MODULE_USAGE_PARAM_INT('i', VOLZ_ID_UNKNOWN, 0x01, 0x1E, "Actuator ID", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_pos_as_zero", "Set current actuator position as new "
                                                        "mid-position (zero)");
    PRINT_MODULE_USAGE_PARAM_INT('i', VOLZ_ID_UNKNOWN, 0x01, 0x1E, "Actuator ID", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int volz_main(int argc, char *argv[])
{
	return VolzOutput::main(argc, argv);
}
