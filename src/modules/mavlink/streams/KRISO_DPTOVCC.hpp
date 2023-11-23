#ifndef KRISO_DPTOVCC_HPP
#define KRISO_DPTOVCC_HPP

#include <uORB/topics/kriso_dptovcc.h>

class MavlinkStreamKrisoDpToVcc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoDpToVcc(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_DPTOVCC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_DP_TO_VCC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return dptovcc_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_DP_TO_VCC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoDpToVcc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription dptovcc_sub{ORB_ID(kriso_dptovcc)};

	bool send() override
	{
		kriso_dptovcc_s dptovcc{};
		if (dptovcc_sub.update(&dptovcc)) {
			mavlink_kriso_dp_to_vcc_t msg{};

			msg.surge_error = dptovcc.surge_error;
			msg.sway_error = dptovcc.sway_error;
			msg.yaw_error = dptovcc.yaw_error;
			msg.dp_start_stop = dptovcc.dp_start_stop;

			mavlink_msg_kriso_dp_to_vcc_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso dptovcc sent!!!");
			return true;
		}
		return false;
	}
};

#endif // KRISO_DPTOVCC_HPP
