#ifndef KRISO_VOLTAGE_HPP
#define KRISO_VOLTAGE_HPP

#include <uORB/topics/kriso_voltage.h>

class MavlinkStreamKrisoVoltage : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoVoltage(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_VOLTAGE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_VOL_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return status_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_VOL_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoVoltage(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription status_sub{ORB_ID(kriso_voltage)};

	bool send() override
	{
		kriso_voltage_s status{};
		if (status_sub.update(&status)) {
			mavlink_kriso_vol_status_t msg{};
			msg.ch1_volt = 10.11;
			msg.ch2_volt = 11.22;
			msg.ch3_volt = 12.33;
			msg.ch4_volt = 13.44;
			msg.invertor_volt = 14.55;
			mavlink_msg_kriso_vol_status_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso voltage sent!!!");
			return true;;
		}
		return false;
	}
};

#endif // KRISO_VOLTAGE_HPP
