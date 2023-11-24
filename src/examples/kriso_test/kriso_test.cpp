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

#include "kriso_test.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int KrisoTest::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int KrisoTest::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "1")) {
		get_instance()->sendStatus();
		return 0;
	}
	else if(!strcmp(argv[0], "2")) {
		get_instance()->sendVoltage();
		return 0;
	} else if(!strcmp(argv[0], "3")) {
		get_instance()->sendControlCmdVcc();
		return 0;
	}else if(!strcmp(argv[0], "4")) {
		get_instance()->sendLoggingStatus();
		return 0;
	}else if(!strcmp(argv[0], "5")) {
		get_instance()->sendAisStatus1();
		return 0;
	}else if(!strcmp(argv[0], "6")) {
		get_instance()->sendAisStatus2();
		return 0;
	}else if(!strcmp(argv[0], "7")) {
		get_instance()->sendAisStatus3();
		return 0;
	}else if(!strcmp(argv[0], "8")) {
		get_instance()->sendDPtoVcc();
		return 0;
	}else if(!strcmp(argv[0], "9")) {
		get_instance()->sendPLCtoVcc();
		return 0;
	}

	return 0;
}
/*
uORB::Publication<kriso_catovcc_s> _kriso_catovcc_topic{ORB_ID(kriso_catovcc)};
	uORB::Publication<kriso_cktovcc_s> _kriso_cktovcc_topic{ORB_ID(kriso_cktovcc)};
	uORB::Publication<kriso_controlcmdtovcc_s> _kriso_controlcmdtovcc_topic{ORB_ID(kriso_controlcmdtovcc)};
	uORB::Publication<kriso_dptovcc_s> _kriso_dptovcc_topic{ORB_ID(kriso_dptovcc)};
	uORB::Publication<kriso_status_s> _kriso_status_topic{ORB_ID(kriso_status)};
	uORB::Publication<kriso_wttovcc_s> _kriso_wttovcc_topic{ORB_ID(kriso_wttovcc)};
*/

void KrisoTest::sendStatus()
{
	kriso_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.nav_longitude = lon;
	status.nav_latitude = lat;
	_kriso_status_topic.publish(status);
	PX4_ERR("send Kriso Status!");
	lat = lat + 0.02;
	lon = lon + 0.03;
}

void KrisoTest::sendVoltage()
{
	kriso_voltage_s status{};
	status.timestamp = hrt_absolute_time();
	_kriso_voltage_topic.publish(status);
	PX4_ERR("send voltage status!");
}

void KrisoTest::sendControlCmdVcc()
{
	kriso_controlcmdtovcc_s status{};
	status.timestamp = hrt_absolute_time();

	status.t1_rpm = 1.1;
	status.t2_rpm = 1.2;
	status.t3_rpm = 1.3;
	status.t3_angle = 1.4;
	status.t4_rpm = 1.5;
	status.t4_angle = 1.6;
	status.oper_mode = 1;
	status.mission_mode = 2;
	_kriso_controlcmdtovcc_topic.publish(status);
	PX4_ERR("send ControlCmdToVcc status!");
}

void KrisoTest::sendLoggingStatus()
{
	kriso_loggingstatus_s status{};
	status.timestamp = hrt_absolute_time();
	status.logging_status = 1;
	_kriso_loggingstatus_topic.publish(status);
	PX4_ERR("send logging status!");
}

void KrisoTest::sendAisStatus1()
{
	kriso_aisstatus_s status{};

	status.msg_type  = 2;
	status.repeat = 3;
	status.mmsi  = 4;
	status.reserved_1  = 5;
	status.speed       = 2.1;
	status.accuracy      = 2;
	status.lon         = 126.6346511;
	status.lat         = 37.1818288;
	status.course      = 3.3;
	status.heading    = 10;
	status.second        = 11;
	status.reserved_2    = 12;
	status.cs             = 1;
	status. display        = 2;
	status.dsc            = 3;
	status. band           = 4;
	status.msg22          = 5;
	status.assigned       = 6;
	status.raim           = 7;
	status.radio         = 8;

	_kriso_aisstatus_topic.publish(status);
	PX4_ERR("send ais status!");

}

void KrisoTest::sendAisStatus2()
{
	kriso_aisstatus_s status{};

	status.msg_type  = 2;
	status.repeat = 3;
	status.mmsi  = 4;
	status.reserved_1  = 5;
	status.speed       = 2.1;
	status.accuracy      = 2;
	status.lon         = 126.6348923;
	status.lat         = 37.1831738;
	status.course      = 3.3;
	status.heading    = 10;
	status.second        = 11;
	status.reserved_2    = 12;
	status.cs             = 1;
	status. display        = 2;
	status.dsc            = 3;
	status. band           = 4;
	status.msg22          = 5;
	status.assigned       = 6;
	status.raim           = 7;
	status.radio         = 8;

	_kriso_aisstatus_topic.publish(status);
	PX4_ERR("send ais status!");

}

void KrisoTest::sendAisStatus3()
{
	kriso_aisstatus_s status{};

	status.msg_type  = 2;
	status.repeat = 3;
	status.mmsi  = 5;
	status.reserved_1  = 5;
	status.speed       = 2.1;
	status.accuracy      = 2;
	status.lon         = 126.6348923;
	status.lat         = 37.1831738;
	status.course      = 3.3;
	status.heading    = 10;
	status.second        = 11;
	status.reserved_2    = 12;
	status.cs             = 1;
	status. display        = 2;
	status.dsc            = 3;
	status. band           = 4;
	status.msg22          = 5;
	status.assigned       = 6;
	status.raim           = 7;
	status.radio         = 8;

	_kriso_aisstatus_topic.publish(status);
	PX4_ERR("send ais status!");

}

void KrisoTest::sendDPtoVcc()
{
	kriso_dptovcc_s dptovcc{};

	dptovcc.timestamp = hrt_absolute_time();
	dptovcc.surge_error = 5.1;
	dptovcc.sway_error = 1.2;
	dptovcc.yaw_error = 1.3;
	dptovcc.dp_start_stop = 1;
	dptovcc._padding0[0] = 1;
	dptovcc._padding0[1] = 2;
	dptovcc._padding0[2] = 3;
	dptovcc._padding0[3] = 4;

	_kriso_dptovcc_topic.publish(dptovcc);
	PX4_ERR("send dptovcc status!");
}

void KrisoTest::sendPLCtoVcc()
{
	kriso_plctovcc_s plctovcc{};

	plctovcc.timestamp = hrt_absolute_time();
	plctovcc.mr_mtr_sta = 1;
	plctovcc.mr_flt_msg_err1 = 2;
	plctovcc.mr_flt_msg_err2 = 3;
	plctovcc.mr_flt_msg_warn1 = 4;
	plctovcc.mr_flt_msg_warn2 = 5;
	plctovcc.mr_mtr_curr_real = 6;
	plctovcc.mr_temp = 7;
	plctovcc.mr_mtr_rpm_real = 8;
	plctovcc.mr_mtr_rot_real = 9;
	plctovcc.ml_mtr_sta = 10;
	plctovcc.ml_flt_msg_err1 = 11;
	plctovcc.ml_flt_msg_err2 = 12;
	plctovcc.ml_flt_msg_warn1 = 13;
	plctovcc.ml_flt_msg_warn2 = 14;
	plctovcc.ml_mtr_curr_real = 15;
	plctovcc.ml_temp = 16;
	plctovcc.ml_mtr_rpm_real = 17;
	plctovcc.ml_mtr_rot_real = 18;
	plctovcc.br_mtr_sta = 19;
	plctovcc.br_flt_msg = 20;
	plctovcc.br_mtr_curr_real = 21;
	plctovcc.br_temp = 22;
	plctovcc.br_mtr_rpm = 23;
	plctovcc.br_mtr_rot_sta = 24;
	plctovcc.bl_mtr_sta = 25;
	plctovcc.bl_flt_msg = 26;
	plctovcc.bl_mtr_curr_real = 27;
	plctovcc.bl_temp = 28;
	plctovcc.bl_mtr_rpm = 29;
	plctovcc.bl_mtr_rot_sta = 30;
	plctovcc.sr_str_rpm = 31;
	plctovcc.sr_str_ang = 32;
	plctovcc.sl_str_rpm = 33;
	plctovcc.sl_str_ang = 34;
	plctovcc.batt400vdc = 35;
	plctovcc.batt24vdc_1 = 36;
	plctovcc.batt24vdc_2 = 37;
	plctovcc._padding0[0] = 1;
	plctovcc._padding0[1] = 2;
	plctovcc._padding0[2] = 3;
	plctovcc._padding0[3] = 4;
	plctovcc._padding0[4] = 5;
	plctovcc._padding0[5] = 6;

	_kriso_plctovcc_topic.publish(plctovcc);
	PX4_ERR("send plctovcc status!");

}

int KrisoTest::task_spawn(int argc, char *argv[])
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

KrisoTest *KrisoTest::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	KrisoTest *instance = new KrisoTest(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

KrisoTest::KrisoTest(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void KrisoTest::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

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

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
}

void KrisoTest::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void KrisoTest::publishMsg(uint8_t type)
{
	switch(type){
		case 0: {
			kriso_catovcc_s msg{};
			_kriso_catovcc_topic.publish(msg); }
			break;
		case 1: {
			kriso_cktovcc_s msg{};
			_kriso_cktovcc_topic.publish(msg); }
			break;
		case 2: {
			kriso_controlcmdtovcc_s msg{};
			_kriso_controlcmdtovcc_topic.publish(msg);}
			break;
		case 3: {
			kriso_dptovcc_s msg{};
			_kriso_dptovcc_topic.publish(msg); }
			break;
		case 4: {
			kriso_status_s msg{};
			_kriso_status_topic.publish(msg); }
			break;
		case 5: {
			kriso_wttovcc_s msg{};
			_kriso_wttovcc_topic.publish(msg); }
			break;
		default :
			break;
	}
}
int KrisoTest::print_usage(const char *reason)
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

int kriso_test_main(int argc, char *argv[])
{
	return KrisoTest::main(argc, argv);
}
