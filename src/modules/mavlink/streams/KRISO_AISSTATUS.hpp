#ifndef KRISO_AISSTATUS_HPP
#define KRISO_AISSTATUS_HPP

#include <uORB/topics/kriso_aisstatus.h>

class MavlinkStreamKrisoAisStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoAisStatus(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_AISSTATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_AIS_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return status_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_AIS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoAisStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription status_sub{ORB_ID(kriso_aisstatus)};

	bool send() override
	{
		kriso_aisstatus_s status{};
		if (status_sub.update(&status)) {
			mavlink_kriso_ais_status_t msg{};

			msg.msg_type = status.msg_type;
			msg.repeat = status.repeat;
			msg.mmsi = status.mmsi;
			msg.reserved_1 = status.reserved_1;
			msg.speed = status.speed;
			msg.accuracy = status.accuracy;
			msg.lon = status.lon;
			msg.lat = status.lat;
			msg.course = status.course;
			msg.heading = status.heading;
			msg.second  = status.second;
			msg.reserved_2  = status.reserved_2;
			msg.cs    = status.cs;
			msg.display  = status.display;
			msg.dsc      = status.dsc;
			msg.band   = status.band;
			msg.msg22    = status.msg22;
			msg.assigned   = status.assigned;
			msg.raim   = status.raim;
			msg.radio      = status.radio;

			mavlink_msg_kriso_ais_status_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso ais status sent!!!");
			return true;
		}
		return false;
	}
};

#endif // KRISO_STATUS_HPP
