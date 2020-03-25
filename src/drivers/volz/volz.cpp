/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "volz_ground_up.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

// using namespace time_literals; done by many modules, but what is it good for?

// BAUD RATE of TELEM2 can be set in QGroundControl, I believe. Check TFMINI.cpp how it can be set in the code as well

int Module::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

/* I don't think this will be used, but just leaving it here in case I want to implement
some custom commands in the future */
int Module::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unrecognized command");
}

// px4_task_spawn_cmd, which is called in this method, instantiates the object and calls run() to
// initiate the main loop. Once run() is done, it deletes the object
int Module::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

// Creates a new object and returns it, nullpointer otherwise
Module *Module::instantiate(int argc, char *argv[])
{
	Module *instance = new Module();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Module::Module(): OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // some parameters for the mixer. Copied from DShot, but should be fine
    _mixing_output.setAllDisarmedValues(0);
	_mixing_output.setAllMinValues(-1);
	_mixing_output.setAllMaxValues(1);
}

// main loop. Loops until should_exit() returns true
void Module::run()
{
	// replace by any uORB subscription that may be necessary. None needed at the moment
	/*
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;
	*/

	port_handle = ::open("/dev/ttyS2", O_RDWR | O_NOCTTY); // get handle to TELEM2 port

	// initialize parameters
	// parameters_update(true);

	while (!should_exit()) {

		_mixing_output.update();

		/*
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...

		}
		*/

		// parameters_update();
	}

	close(port_handle); // close the TELEM2 port when the main loop is stopped

	// orb_unsubscribe(sensor_combined_sub);
}

// No parameters yet, but that may change
/*void Module::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}*/

int Module::highbyte(int value) {
	return (value >> 8) & 0xff;
}

int Module::lowbyte(int value) {
	return value & 0xff;
}

// directly pasted from C++. Will this work?
int Module::generate_crc (int cmd, int actuator_id, int arg_1, int arg_2)	{
	unsigned short int crc=0xFFFF; // init value of result
	char command[4]={cmd,actuator_id,arg_1,arg_2}; // command, ID, argument1, argument 2
	char x,y;
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

int[] Module::new_pos_cmd(float angle, int actuator_id) {
	nt arg = POS_CENTER + (int)(2 * angle / TOT_ANGLE * (POS_CENTER - POS_MIN));
	int arg_1 = 0x16; //highbyte(arg);
	int arg_2 = 0x60; //lowbyte(arg);
	int crc = generate_crc(NEW_POS_CMD, ACTUATOR_ID, arg_1, arg_2);
	int crc_1 = highbyte(crc);
	int crc_2 = lowbyte(crc);

	int command[] = {NEW_POS_CMD, ACTUATOR_ID, arg_1, arg_2, crc_1, crc_2};
	return command;
}

// should the outputs be written in this method? Or will that cause any issues with threading?
bool Module::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated) {
	if (port_handle < 0) {  // we can't write the outputs anywhere if the port handle is not assigned
		return false;
	}

	// should all servos go to idle position when stop_motors = True?
	for (int i = 0, i < sizeof(actuator_ids), i++) {
		cmd = new_pos_cmd(outputs[i] * TOT_ANGLE / 2, actuactor_ids[i]);
		::write(port_handle, cmd[i], CMD_SIZE);
	}

}

int Module::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.
This is a template for a module running as a task in the background with start/stop/status functionality.
### Implementation
Section describing the high-level implementation of this module.
### Examples
CLI usage example:
$ module start -f -p 42
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// Main entry point to the module. Calls "main" from ModuleBase which handles any commands passed to
// the class, such as custom commands or help, info, status, start, stop etc. The appropriate methods
// such as custom_command, print_usage etc. are called accordingly
int module_main(int argc, char *argv[])
{
	return Module::main(argc, argv);
}