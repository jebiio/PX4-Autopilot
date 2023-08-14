#ifndef KRISO_LOGGINGSTATUS_HPP
#define KRISO_LOGGINGSTATUS_HPP

#include <uORB/topics/kriso_loggingstatus.h>

class MavlinkStreamKrisoLoggingStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoLoggingStatus(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_LOGGINGSTATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_ROS_LOG_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return status_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_ROS_LOG_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoLoggingStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription status_sub{ORB_ID(kriso_loggingstatus)};

	bool send() override
	{
		kriso_loggingstatus_s status{};
		if (status_sub.update(&status)) {
			mavlink_kriso_ros_log_status_t msg{};
			msg.logging_status = status.logging_status;
			mavlink_msg_kriso_ros_log_status_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso ros log status sent!!!");
			return true;
		}
		return false;
	}
};

#endif // KRISO_STATUS_HPP
