#ifndef KRISO_PLCTOVCC_HPP
#define KRISO_PLCTOVCC_HPP

#include <uORB/topics/kriso_plctovcc.h>

class MavlinkStreamKrisoPlcToVcc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKrisoPlcToVcc(mavlink); }

	static constexpr const char *get_name_static() { return "KRISO_PLCTOVCC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_KRISO_PLC_TO_VCC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return plctovcc_sub.advertised() ? (MAVLINK_MSG_ID_KRISO_PLC_TO_VCC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamKrisoPlcToVcc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription plctovcc_sub{ORB_ID(kriso_plctovcc)};

	bool send() override
	{
		kriso_plctovcc_s plctovcc{};

		if (plctovcc_sub.update(&plctovcc)) {

			mavlink_kriso_plc_to_vcc_t msg{};

			msg.mr_mtr_sta = plctovcc.mr_mtr_sta;
			msg.mr_flt_msg_err1 = plctovcc.mr_flt_msg_err1;
			msg.mr_flt_msg_err2 = plctovcc.mr_flt_msg_err2;
			msg.mr_flt_msg_warn1 = plctovcc.mr_flt_msg_warn1;
			msg.mr_flt_msg_warn2 = plctovcc.mr_flt_msg_warn2;
			msg.mr_mtr_curr_real = plctovcc.mr_mtr_curr_real;
			msg.mr_temp = plctovcc.mr_temp;
			msg.mr_mtr_rpm_real = plctovcc.mr_mtr_rpm_real;
			msg.mr_mtr_rot_real = plctovcc.mr_mtr_rot_real;
			msg.ml_mtr_sta = plctovcc.ml_mtr_sta;
			msg.ml_flt_msg_err1 = plctovcc.ml_flt_msg_err1;
			msg.ml_flt_msg_err2 = plctovcc.ml_flt_msg_err2;
			msg.ml_flt_msg_warn1 = plctovcc.ml_flt_msg_warn1;
			msg.ml_flt_msg_warn2 = plctovcc.ml_flt_msg_warn2;
			msg.ml_mtr_curr_real = plctovcc.ml_mtr_curr_real;
			msg.ml_temp = plctovcc.ml_temp;
			msg.ml_mtr_rpm_real = plctovcc.ml_mtr_rpm_real;
			msg.ml_mtr_rot_real = plctovcc.ml_mtr_rot_real;
			msg.br_mtr_sta = plctovcc.br_mtr_sta;
			msg.br_flt_msg = plctovcc.br_flt_msg;
			msg.br_mtr_curr_real = plctovcc.br_mtr_curr_real;
			msg.br_temp = plctovcc.br_temp;
			msg.br_mtr_rpm = plctovcc.br_mtr_rpm;
			msg.br_mtr_rot_sta = plctovcc.br_mtr_rot_sta;
			msg.bl_mtr_sta = plctovcc.bl_mtr_sta;
			msg.bl_flt_msg = plctovcc.bl_flt_msg;
			msg.bl_mtr_curr_real = plctovcc.bl_mtr_curr_real;
			msg.bl_temp = plctovcc.bl_temp;
			msg.bl_mtr_rpm = plctovcc.bl_mtr_rpm;
			msg.bl_mtr_rot_sta = plctovcc.bl_mtr_rot_sta;
			msg.sr_str_rpm = plctovcc.sr_str_rpm;
			msg.sr_str_ang = plctovcc.sr_str_ang;
			msg.sl_str_rpm = plctovcc.sl_str_rpm;
			msg.sl_str_ang = plctovcc.sl_str_ang;
			msg.batt400vdc = plctovcc.batt400vdc;
			msg.batt24vdc_1 = plctovcc.batt24vdc_1;
			msg.batt24vdc_2 = plctovcc.batt24vdc_2;

			mavlink_msg_kriso_plc_to_vcc_send_struct(_mavlink->get_channel(), &msg);
			PX4_ERR("mavlink kriso PLCtoVCC sent!!!");
			return true;
		}
		return false;
	}
};

#endif // KRISO_PLCTOVCC_HPP
