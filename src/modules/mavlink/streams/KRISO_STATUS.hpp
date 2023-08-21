#ifndef KRISO_STATUS_HPP
#define KRISO_STATUS_HPP

#include <uORB/topics/kriso_status.h>

class MavlinkStreamKrisoStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoStatus(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return status_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription status_sub{ORB_ID(kriso_status)};

	bool send() override
	{
		kriso_status_s status{};
		if (status_sub.update(&status)) {
			mavlink_kriso_status_t msg{};
			msg.nav_latitude = status.nav_latitude;
			msg.nav_longitude = status.nav_longitude;
			msg.nav_mode = 2;
			msg.nav_yaw = 1.0;
			msg.nav_yaw_rate = 1.8;
			msg.wea_visibiran = 2.2;
			PX4_ERR("lat: %lf, lon: %lf", status.nav_latitude, status.nav_longitude);
			mavlink_msg_kriso_status_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso status sent!!!");
			return true;
		}
		return false;
	}
};

#endif // KRISO_STATUS_HPP
