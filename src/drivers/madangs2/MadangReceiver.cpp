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

		if (ret > 0) {
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
	//send_command();
	state = rx_state;
	before_rx_now = hrt_absolute_time();
	nread = 0;
}

void
MadangReceiver::rx_task(){
	if(hrt_elapsed_time(&before_rx_now) > 500000) {
		//time out
		state = tx_state;
		return ;
	}
	if (nread < 3) {
		state = tx_state;
		return ;
	}
	else if(nread < 18) {
		//더 돌아야
	} else if (nread == 18) {
		state = tx_state;
	} else {
		//over !!!
	}
}
