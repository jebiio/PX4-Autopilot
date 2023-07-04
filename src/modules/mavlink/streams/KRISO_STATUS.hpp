#ifndef KRISO_STATUS_HPP
#define KRISO_STATUS_HPP

#include <uORB/topics/kriso_status.h>

class MavlinkStreamKrisoStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoStatus(mavlink); }

	static constexpr const char *get_name_static() { return "STATUS"; }
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
		PX4_ERR("mavlink kriso status sent!!!");

		if (status_sub.copy(&status)) {
			mavlink_kriso_status_t msg{};
			mavlink_msg_kriso_status_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso status sent!!!");
			return true;
		}
		return false;
	}
};

#endif // KRISO_STATUS_HPP
