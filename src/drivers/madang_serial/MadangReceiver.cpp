#include <systemlib/px4_macros.h>

#include <math.h>
#include <poll.h>

#include "Madang.hpp"
#include "MadangReceiver.hpp"



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

void
MadangReceiver::Run()
{
	const int timeout = 20;
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "madang_rcv_if%d", _madang->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}
	uint8_t buf[64];
	ssize_t nread = 0;
	//hrt_abstime last_send_update = 0;

	struct pollfd fds[1] = {};

	fds[0].fd = _madang->get_uart_fd();
	fds[0].events = POLLIN;

	while (!_madang->_task_should_exit) {
		int ret = poll(&fds[0], 1, timeout);

		if (ret > 0) {
			/* non-blocking read. read may return negative values */
			nread = ::read(fds[0].fd, buf, sizeof(buf));

			if (nread == -1 && errno == ENOTCONN) { // Not connected (can happen for USB)
				usleep(100000);
			}
		} else {
			PX4_ERR("madang receiver timeout! ");
		}
	}
}
