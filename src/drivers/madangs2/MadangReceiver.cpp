#include <systemlib/px4_macros.h>

#include <math.h>
#include <poll.h>

#include "Madang.hpp"
#include "MadangReceiver.hpp"
#include "dwm_api.h"


MadangReceiver::~MadangReceiver()
{

}

MadangReceiver::MadangReceiver(MadangSerial *parent) :
	ModuleParams(nullptr),
	_madang(parent)
{

}
void *
MadangReceiver::start_helper(void *context)
{
	MadangReceiver rcv{(MadangSerial *)context};
	rcv.Run();

	return nullptr;
}

void
MadangReceiver::receive_start(pthread_t *thread, MadangSerial *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr,
				  PX4_STACK_ADJUSTED(sizeof(MadangReceiver) + 1500));

	pthread_create(thread, &receiveloop_attr, MadangReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
}
/*
void
MadangReceiver::Run()
{
	const int timeout = 20;
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "madang_rcv_if%d", _madang->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}
	uint8_t rx_data[255];
	ssize_t nread = 0;
	dwm_pos_t p_pos;
	//hrt_abstime last_send_update = 0;

	struct pollfd fds[1] = {};

	fds[0].fd = _madang->get_uart_fd();
	fds[0].events = POLLIN;

	while (!_madang->_task_should_exit) {
		int ret = poll(&fds[0], 1, timeout);

		if (ret > 0) {
			// non-blocking read. read may return negative values
			nread = ::read(fds[0].fd, rx_data, 255);
			PX4_ERR("Got Postion Data!!! ------------- ");
			if (nread == -1 && errno == ENOTCONN) { // Not connected (can happen for USB)
				usleep(100000);
			}
			else if (nread == 18) { //(nread == 18) {
				uint8_t data_cnt = RESP_DAT_VALUE_OFFSET;
				p_pos.x = rx_data[data_cnt]
					+ (rx_data[data_cnt+1]<<8)
					+ (rx_data[data_cnt+2]<<16)
					+ (rx_data[data_cnt+3]<<24);
				data_cnt += 4;
				p_pos.y = rx_data[data_cnt]
					+ (rx_data[data_cnt+1]<<8)
					+ (rx_data[data_cnt+2]<<16)
					+ (rx_data[data_cnt+3]<<24);
				data_cnt += 4;
				p_pos.z = rx_data[data_cnt]
					+ (rx_data[data_cnt+1]<<8)
					+ (rx_data[data_cnt+2]<<16)
					+ (rx_data[data_cnt+3]<<24);
				data_cnt += 4;
				p_pos.qf = rx_data[data_cnt];
				PX4_INFO("Local Position : x: %d, y: %d, z: %d -------", p_pos.x, p_pos.y, p_pos.z);
				_madang->position_requesting = false;
				tcflush(_madang->get_uart_fd(), TCIOFLUSH);
				memset(rx_data, 1, 255);
			}
		} else {
			PX4_ERR("madang receiver timeout! ");
		}
	}
}
*/

void
MadangReceiver::Run() {
	const int timeout = 20;
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "madang_rcv_if%d", _madang->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	struct pollfd fds[1] = {};

	fds[0].fd = _madang->get_uart_fd();
	fds[0].events = POLLIN;

	while (!_madang->_task_should_exit) {
		int ret = poll(&fds[0], 1, timeout);
		PX4_INFO("madang receiver loop----------");
		if (ret >= 0) {
			if (state == tx_state) {
				tx_task();
			} else if(state == rx_state) {
				rx_task();
			}
		} else {
			//PX4_ERR("madang receiver timeout! ");
		}
	}
}

void
MadangReceiver::tx_task() {
	nread = 0;
	int bytes = 0;
   	uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   	tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_POS_GET;
   	tx_data[tx_len++] = 0;
	bytes = ::write(_madang->get_uart_fd(), tx_data, tx_len);
	if (bytes > 0) {
		PX4_INFO("Local Position ready?");
		before_rx_now = hrt_absolute_time();
		state = rx_state;
	} else {
		PX4_ERR("madang tx fail!!!");
	}
}

void
MadangReceiver::rx_task(){
	int bytes = 0;
	uint8_t rx_data[255];
	if(hrt_elapsed_time(&before_rx_now) > 500000) {
		//time out
		state = tx_state;
		PX4_ERR("madang rx Timeout!!!");
		return ;
	}
	bytes = ::read(_madang->get_uart_fd(), rx_data, 255);
	nread = nread + bytes;
	if (bytes < 3) {
		state = tx_state;
		PX4_ERR("error : rx bytes < 3 !!!");
		return ;
	}
	else if(nread < 18) {
		nread = nread + bytes;
		PX4_ERR("continue : rx bytes + nread !!!");
	} else if (nread == 18) {
		//PX4_ERR("nread ==== 18 !!!");
		dwm_pos_t p_pos;
		uint8_t data_cnt = RESP_DAT_VALUE_OFFSET;
		p_pos.x = rx_data[data_cnt]
			+ (rx_data[data_cnt+1]<<8)
			+ (rx_data[data_cnt+2]<<16)
			+ (rx_data[data_cnt+3]<<24);
		data_cnt += 4;
		p_pos.y = rx_data[data_cnt]
			+ (rx_data[data_cnt+1]<<8)
			+ (rx_data[data_cnt+2]<<16)
			+ (rx_data[data_cnt+3]<<24);
		data_cnt += 4;
		p_pos.z = rx_data[data_cnt]
			+ (rx_data[data_cnt+1]<<8)
			+ (rx_data[data_cnt+2]<<16)
			+ (rx_data[data_cnt+3]<<24);
		data_cnt += 4;
		p_pos.qf = rx_data[data_cnt];
		PX4_INFO("Local Position : x: %d, y: %d, z: %d -------", p_pos.x, p_pos.y, p_pos.z);
		_madang->position_requesting = false;
		send_localposition(p_pos.x, p_pos.y, p_pos.z);
		tcflush(_madang->get_uart_fd(), TCIOFLUSH);
		memset(rx_data, 1, 255);
		nread = 0;
		state = tx_state;

	} else if (nread > 18) {
		PX4_ERR("nread > 18 ------ !!!");
		nread = 0;
		state = tx_state;
	}else {
		nread = 0;
		state = tx_state;
		//over !!!
	}
}

void
MadangReceiver::send_localposition(int x, int y, int z) {
	struct vehicle_odometry_s odom;
	/* The position in the local NED frame */
	// mm(Decawave) -> m()
	float xf = (float) x/1000;
	float yf = (float) y/1000;
	float zf = (float) z/1000;

	odom.x = xf;
	odom.y = yf;
	odom.z = -zf;

	/* The euler angles of the VISUAL_POSITION_ESTIMATE msg represent a
		* rotation from NED earth/local frame to XYZ body frame */
	matrix::Quatf q(matrix::Eulerf(NAN, NAN, NAN));

	odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;


	odom.pose_covariance[0] = NAN;

	odom.vx = NAN;
	odom.vy = NAN;
	odom.vz = NAN;

	odom.rollspeed = NAN;
	odom.pitchspeed = NAN;
	odom.yawspeed = NAN;

	odom.velocity_covariance[0] = NAN;

	PX4_INFO("vision publish!!!!!");
	odom.timestamp = hrt_absolute_time();
	/* Publish the odometry */
	_visual_odometry_pub.publish(odom);
}
