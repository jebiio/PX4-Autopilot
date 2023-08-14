

#ifndef KRISO_CONTROLCMDTOVCC_HPP
#define KRISO_CONTROLCMDTOVCC_HPP

#include <uORB/topics/kriso_controlcmdtovcc.h>

class MavlinkStreamKrisoControlCmdToVcc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoControlCmdToVcc(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_CONTROLCMDTOVCC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_CONTROL_COMMAND_TO_VCC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return status_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_CONTROL_COMMAND_TO_VCC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoControlCmdToVcc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription status_sub{ORB_ID(kriso_controlcmdtovcc)};

	bool send() override
	{
		// we're sending the GPS home periodically to ensure the
		// the GCS does pick it up at one point
		kriso_controlcmdtovcc_s home;

		if (status_sub.update(&home)) {
			mavlink_kriso_control_command_to_vcc_t msg{};
			msg.t1_rpm = home.t1_rpm;
			msg.t2_rpm = home.t2_rpm;
			msg.t3_rpm = home.t3_rpm;
			msg.t3_angle = home.t3_angle;
			msg.t4_rpm = home.t4_rpm;
			msg.t4_angle = home.t4_angle;
			msg.oper_mode = home.oper_mode;
			msg.mission_mode = home.mission_mode;
			mavlink_msg_kriso_control_command_to_vcc_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink  control cmd to vcc sent!!!");
			return true;
		}

		return false;
	}
};

#endif // KRISO_CONTROLCMDTOVCC_HPP
