#ifndef KRISO_STATUS_HPP
#define KRISO_STATUS_HPP

#include <uORB/topics/kriso_status.h>

class MavlinkStreamKrisoStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoStatus(mavlink); }

	static constexpr const char *get_name_static() { return "DPTOVCC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HOME_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _home_sub.advertised() ? (MAVLINK_MSG_ID_HOME_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _home_sub{ORB_ID(kriso_status)};

	bool send() override
	{
		// we're sending the GPS home periodically to ensure the
		// the GCS does pick it up at one point
		kriso_status_s home;

		if (kriso_cktovcc.advertised() && kriso_cktovcc.copy(&home)) {
			if (home.valid_hpos) {
				mavlink_home_position_t msg{};

				msg.latitude  = home.lat * 1e7;
				msg.longitude = home.lon * 1e7;
				msg.altitude  = home.alt * 1e3f;

				msg.x = home.x;
				msg.y = home.y;
				msg.z = home.z;

				msg.approach_x = 0.f;
				msg.approach_y = 0.f;
				msg.approach_z = 0.f;

				msg.time_usec = home.timestamp;

				mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif // KRISO_STATUS_HPP
