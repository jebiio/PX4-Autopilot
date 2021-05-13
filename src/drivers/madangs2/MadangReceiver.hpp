/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file madang_receiver.h
 * madang receiver thread
 *
 * @author Jeyong Shin
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_odometry.h>

using namespace time_literals;

class Madang;

enum MadangState {
	tx_state = 0,
	rx_state
};

class MadangReceiver : public ModuleParams
{
public:
	MadangReceiver(MadangSerial *parent);
	~MadangReceiver() override;

	/**
	 * Start the receiver thread
	 */
	static void receive_start(pthread_t *thread, MadangSerial *parent);

	static void *start_helper(void *context);

private:

	void Run();

	void schedule_tune(const char *tune);

	void update_params();
	void rx_task();
	void tx_task();
	void send_localposition(int x, int y, int z);
	MadangSerial				*_madang;
	enum MadangState state  {tx_state};
	hrt_abstime before_rx_now;
	int nread;
	uORB::Publication<vehicle_odometry_s>		_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	// Disallow copy construction and move assignment.
	MadangReceiver(const MadangReceiver &) = delete;
	MadangReceiver operator=(const MadangReceiver &) = delete;
};
