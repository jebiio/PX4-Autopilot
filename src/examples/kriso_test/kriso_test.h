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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/kriso_catovcc.h>
#include <uORB/topics/kriso_cktovcc.h>
#include <uORB/topics/kriso_controlcmdtovcc.h>
#include <uORB/topics/kriso_dptovcc.h>
#include <uORB/topics/kriso_status.h>
#include <uORB/topics/kriso_wttovcc.h>
#include <uORB/topics/kriso_voltage.h>
#include <uORB/topics/kriso_controlcmdvcc.h>
#include <uORB/topics/kriso_loggingstatus.h>

using namespace time_literals;

extern "C" __EXPORT int kriso_test_main(int argc, char *argv[]);


class KrisoTest : public ModuleBase<KrisoTest>, public ModuleParams
{
public:
	KrisoTest(int example_param, bool example_flag);

	virtual ~KrisoTest() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static KrisoTest *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void sendStatus();
	void sendVoltage();
	void sendControlCmdVcc();
	void sendLoggingStatus();
private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	void publishMsg(uint8_t type);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Publish
	uORB::Publication<kriso_catovcc_s> _kriso_catovcc_topic{ORB_ID(kriso_catovcc)};
	uORB::Publication<kriso_cktovcc_s> _kriso_cktovcc_topic{ORB_ID(kriso_cktovcc)};
	uORB::Publication<kriso_controlcmdtovcc_s> _kriso_controlcmdtovcc_topic{ORB_ID(kriso_controlcmdtovcc)};
	uORB::Publication<kriso_dptovcc_s> _kriso_dptovcc_topic{ORB_ID(kriso_dptovcc)};
	uORB::Publication<kriso_status_s> _kriso_status_topic{ORB_ID(kriso_status)};
	uORB::Publication<kriso_wttovcc_s> _kriso_wttovcc_topic{ORB_ID(kriso_wttovcc)};
	uORB::Publication<kriso_voltage_s> _kriso_voltage_topic{ORB_ID(kriso_voltage)};
	uORB::Publication<kriso_loggingstatus_s> _kriso_loggingstatus_topic{ORB_ID(kriso_loggingstatus)};
};

