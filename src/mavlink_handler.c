
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>

#include "params.h"
#include "network.h"
#include "mavlink.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

#define MAV_DEBUG 0

uint8_t target_system;
uint8_t target_component;

uint16_t chan[10];
int16_t ichan[10];

int armed = 0;

uint32_t chan_UpdateTime_ms = 0;

uint16_t mavlink_getChan(int n){
	return chan[n];
}
// uint32_t mavlink_getChanUpdateTimer(void){
// 	uint32_t delay = system_getTime_ms() - chan_UpdateTime_ms;
// 	return delay;
// }

void mavlink_send_msg(mavlink_message_t * msg){
	int i;
	uint8_t buf[BUFFER_LENGTH];
	int len = mavlink_msg_to_send_buffer(buf, msg);

	network_send(buf, len);
}

void mavlink_send_param(int n){
	struct param_struct p;
	int find_fl = 0;
	if (params_getParam(n, &p) == 0) find_fl = 1;	// if param exist

	mavlink_message_t msg;
	// if param exist - set param to msg
	if (find_fl) {
		float fvalue = 0;
		if ((p.param_type == MAV_PARAM_TYPE_REAL32) || p.param_type == MAV_PARAM_TYPE_REAL64)
			fvalue = p.param_value.FLOAT;
		else fvalue = (float)p.param_value.INT;

		mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, p.param_id, fvalue, p.param_type, params_getSize(), n);}
	// if param none - set none param
	else mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, NULL, 0, MAV_PARAM_TYPE_REAL32, params_getSize(), -1);

	mavlink_send_msg(&msg);

	// for debug
	if (find_fl == 1) {if (MAV_DEBUG) printf("param send ans %d: %d\n\r", n, (int)p.param_value.FLOAT);}
	else if (MAV_DEBUG) printf("param send ans -1 (%d)\n\r", n);
}

void mavlink_send_heartbeat_server(void){	// package for transit system
	mavlink_message_t msg;

	uint8_t system_status = MAV_STATE_BOOT;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	system_status = MAV_STATE_STANDBY;
	if (armed == 1) system_status = MAV_STATE_ACTIVE;
    // mavlink_msg_heartbeat_pack(100, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, base_mode, 0, system_status);
    // mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, base_mode, 0, system_status);
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, base_mode, 0, system_status);
	
	//
	// int armed = MAV_MODE_FLAG_SAFETY_ARMED;
	// #define SYSTEM_ID 100
	// 	mavlink_msg_heartbeat_pack(SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_GENERIC,
	// 		MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0),
	// 		0, MAV_STATE_STANDBY);
	mavlink_send_msg(&msg);
}

int mavlink_send_reboot_system(char * buf){	// package for transit system
	mavlink_message_t msg;

	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	
    mavlink_msg_command_long_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 0, 0, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 3,3,3,3,3,0,0);
	int len = mavlink_msg_to_send_buffer(buf, &msg);

	return len;
}

void mavlink_send_heartbeat(void){
	mavlink_message_t msg;

	uint8_t system_status = MAV_STATE_BOOT;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_PAIRING_MANAGER, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_INVALID, base_mode, 0, system_status);
	mavlink_send_msg(&msg);
}

void mavlink_send_status(void){
	mavlink_message_t msg;
	int voltage = 4150;	//Battery_getVoltage();
	int bat = 43;//Battery_getBatPercent();
	mavlink_msg_sys_status_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 0, 0, 0, 500, voltage, -1, bat, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	mavlink_send_msg(&msg);
}


void mavlink_send_attitude(void){
	mavlink_message_t msg;
	static int timer_us = 0;//system_getTime_ms()*1000;
	timer_us++;

	// float pitch = imu_getPitch()*M_PI/180.0f;
	// float roll = imu_getRoll()*M_PI/180.0f;
	// float yaw = imu_getYaw()*M_PI/180.0f;
	static float pitch = 0, roll = 0, yaw = 0;
    static int pitch_sig = 1, roll_sig = 1, yaw_sig = 1;
    pitch+=pitch_sig*1*3.14f/180.0f;
    roll+=roll_sig*1*3.14f/180.0f;
    yaw+=yaw_sig*1*3.14f/180.0f;

    #define LIMIT_VALUE_DEG 45
    if (pitch>=(LIMIT_VALUE_DEG*3.14/180))pitch_sig = -1;
    else if (pitch<=-(LIMIT_VALUE_DEG*3.14/180))pitch_sig = 1;
    if (roll>=(LIMIT_VALUE_DEG*3.14/180))roll_sig = -1;
    else if (roll<=-(LIMIT_VALUE_DEG*3.14/180))roll_sig = 1;
    if (yaw>=(LIMIT_VALUE_DEG*3.14/180))yaw_sig = -1;
    else if (yaw<=-(LIMIT_VALUE_DEG*3.14/180))yaw_sig = 1;

	mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, timer_us*1000, roll, pitch, yaw, 0, 0, 0);
	mavlink_send_msg(&msg);
#define SYSTEM_ID 1

		// const float zeroQuat[] = {0, 0, 0, 0};
		// mavlink_msg_attitude_quaternion_pack(SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg,
		// 	time, 0.8, 0.1, 0.1, 0.1, 0, 0, 0, zeroQuat);
		// mavlink_send_msg(&msg);

		// mavlink_msg_rc_channels_scaled_pack(SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg, time, 0,
		// 	ichan[0] * 10000, ichan[1] * 10000, ichan[2] * 10000,
		// 	ichan[3] * 10000, ichan[4] * 10000, ichan[5] * 10000,
		// 	INT16_MAX, INT16_MAX, UINT8_MAX);
		// mavlink_send_msg(&msg);

		// float actuator[32] = {1,2,3,4};
		// // memcpy(actuator, motors, sizeof(motors));
		// mavlink_msg_actuator_output_status_pack(SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg, time, 4, actuator);
		// mavlink_send_msg(&msg);

		// float ax=10, ay=20, az=-1000;
		// float gx = 0, gy=-1, gz=1;
		// mavlink_msg_scaled_imu_pack(SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, &msg, time,
		// 	ax * 1000, ay * 1000, az * 1000,
		// 	gx * 1000, gy * 1000, gz * 1000,
		// 	0, 0, 0, 0);
		// mavlink_send_msg(&msg);

}

void mavlink_send_battery_status(void){
	mavlink_message_t msg;
	uint16_t voltages[1] = {4200};
	mavlink_msg_battery_status_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 0, 0, 0, 0, voltages, 0, 0, 0, 45, 0, 0, 0, 0, 0);
	mavlink_send_msg(&msg);
}

void mavlink_send_time(void){
	mavlink_message_t msg;
    static uint64_t time_us=1675674266;
    static uint32_t time_ms=0;
    time_ms+=1000; time_us+=1;
    mavlink_msg_system_time_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, time_us, time_ms);
	mavlink_send_msg(&msg);
}

// result MAV_RESULT_ACCEPTED
void mavlink_send_cmd_ack(uint16_t command, uint8_t result, uint8_t progress){
	mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, command, result, progress, 0, target_system, target_component);
	mavlink_send_msg(&msg);
}

void mavlink_receive(char rxdata){

	mavlink_message_t msg;

	float t_param[10];

	mavlink_status_t status;

	if (mavlink_parse_char(MAVLINK_COMM_0, rxdata, &msg, &status))
	{
		// Packet received
		switch (msg.msgid){
			case MAVLINK_MSG_ID_MANUAL_CONTROL:
				ichan[0] = mavlink_msg_manual_control_get_z(&msg);	// th 0-1000
				ichan[1] = mavlink_msg_manual_control_get_x(&msg);	// p -1000 - 1000
				ichan[2] = mavlink_msg_manual_control_get_y(&msg);	// r -1000 - 1000
				ichan[3] = mavlink_msg_manual_control_get_r(&msg);	// y -1000 - 1000
				printf("MC T:%d, P:%d, R:%d, Y:%d\n\r", ichan[0], ichan[1], ichan[2], ichan[3]);

			case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
				ichan[0] = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);	// p -1000 - 1000
				ichan[1] = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);	// r -1000 - 1000
				ichan[2] = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);	// th 0-1000
				ichan[3] = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);	// y -1000 - 1000
				ichan[4] = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);

				printf("CO P:%d, R:%d, T:%d, Y:%d\n\r", ichan[0], ichan[1], ichan[2], ichan[3]);

				//chan_UpdateTime_ms = system_getTime_ms();
				//if (MAV_DEBUG) printf("ch: %d, %d, %d, %d, %d, \n\r", chan[0], chan[1], chan[2], chan[3], chan[4]);
				break;
			case MAVLINK_MSG_ID_PARAM_SET:
			{
				mavlink_param_set_t param_set;
				char this_param_id[16];

				target_system = mavlink_msg_param_set_get_target_system(&msg);
				target_component = mavlink_msg_param_set_get_target_component(&msg);
				uint16_t param_id_len = mavlink_msg_param_set_get_param_id(&msg, this_param_id);	// name of rx param
				this_param_id[strlen(this_param_id)-1] = '\0';										// correction name
				float this_param_value = mavlink_msg_param_set_get_param_value(&msg);
				uint8_t param_type = 0;
				// if (msg.magic != MAVLINK_STX_MAVLINK1)
				param_type = _MAV_RETURN_uint8_t(&msg,  msg.len-1);

				if (param_type == MAV_PARAM_TYPE_REAL32){
					if (MAV_DEBUG) printf("set param REAL32: %s, %d, %f\n\r", this_param_id, param_type, this_param_value);
				}
				else {
					if (MAV_DEBUG) printf("set param nan: %s, %d, %f\n\r", this_param_id, param_type, this_param_value);
				}

				int pos = params_getIndexById(this_param_id);
				union param_value pv = {.FLOAT=this_param_value};	//FIXME!!!!
				if (pos != 0) params_setValue(pos, pv, param_type);
				mavlink_send_param(pos);

				if (MAV_DEBUG) printf("param set %d: %d\n\r", pos, (int)this_param_value);
				break;
			}
			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			{
				mavlink_param_set_t param_set;
				char this_param_id[16];


				target_system = mavlink_msg_param_set_get_target_system(&msg);
				target_component = mavlink_msg_param_set_get_target_component(&msg);
				uint16_t param_id_len = mavlink_msg_param_request_read_get_param_id(&msg, this_param_id);	// name of rx param
				this_param_id[strlen(this_param_id)-1] = '\0';										// correction name
				int16_t this_index = mavlink_msg_param_request_read_get_param_index(&msg);

				int pos = params_getIndexById(this_param_id);
				if (pos != 0) mavlink_send_param(pos);
				else  mavlink_send_param(this_index);

				if (MAV_DEBUG)printf("reuest read \n\r");
				break;
			}
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			{
				if (MAV_DEBUG) printf("reuest list\n\r");

				target_system = mavlink_msg_param_set_get_target_system(&msg);
				target_component = mavlink_msg_param_set_get_target_component(&msg);

				int p;
				for (p=0;p<params_getSize(); p++){
					mavlink_send_param(p);
					if (MAV_DEBUG) printf("reuest read ans %d\n\r", p);
				}
				break;
			}
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				mavlink_send_heartbeat();
				// if (MAV_DEBUG) printf(".");;//printf("rx HEARTBEAT\n\r");
				break;
			}
			case MAVLINK_MSG_ID_COMMAND_LONG:
			{
				uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
						printf("MAVLINK_MSG_ID_COMMAND_LONG (%d)\n\r", cmd);
				switch(cmd){
					case(MAV_CMD_COMPONENT_ARM_DISARM):
						t_param[1] = mavlink_msg_command_long_get_param1(&msg);
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						if (MAV_DEBUG) printf("rx cmd ARM_DISARM %d\n\r", (int)t_param[1]);
						if ((int)t_param[1]) armed = 1;
						else armed = 0;
						//MotorControl_setArm((int)t_param[1]);
						break;
					case(MAV_CMD_PREFLIGHT_CALIBRATION):
						t_param[1] = mavlink_msg_command_long_get_param1(&msg);
						t_param[5] = mavlink_msg_command_long_get_param5(&msg);
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						if (MAV_DEBUG) printf("rx calibrate\n\r");
						if (t_param[1] != 0)if (MAV_DEBUG) printf("rx calibrate gyro %d\n\r", (int)t_param[1]);
						if (t_param[5] != 0)if (MAV_DEBUG) printf("rx calibrate acc %d\n\r", (int)t_param[5]);

						//if (t_param[5] != 0) imu_AccCalibrate_run();

						break;
					case MAV_CMD_REQUEST_PROTOCOL_VERSION:
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						{
							mavlink_message_t msg;
							mavlink_msg_protocol_version_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 200, 200,200, NULL, NULL);
							mavlink_send_msg(&msg);
						}
						break;
					case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						{
							mavlink_message_t msg;
							mavlink_msg_autopilot_version_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 0,0,0,0,0,NULL, NULL, NULL, 0,0,0,NULL);
							// static inline uint16_t mavlink_msg_autopilot_version_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                            //    uint64_t capabilities, 
							//    uint32_t flight_sw_version, 
							//    uint32_t middleware_sw_version, 
							//    uint32_t os_sw_version, 
							//    uint32_t board_version, 
							//    const uint8_t *flight_custom_version, 
							//    const uint8_t *middleware_custom_version, 
							//    const uint8_t *os_custom_version, 
							//    uint16_t vendor_id, 
							//    uint16_t product_id, 
							//    uint64_t uid, 
							//    const uint8_t *uid2)
							mavlink_send_msg(&msg);
						}
						break;
					default:
						printf("undefined cmd (%d)!!!!!!!!!!!!!!!!!!!!!\n\r", cmd);
					break;
				}
				break;
			}
			default:
				//if (MAV_DEBUG) printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
		}
	}

}


char * mavId2str(int id){
	static char thisId[255] = "";
	if (id == MAVLINK_MSG_ID_STATUSTEXT) sprintf(thisId, "%s","STATUSTEXT");
	else if (id == MAVLINK_MSG_ID_HEARTBEAT) sprintf(thisId, "%s","HEARTBEAT");
	else if (id == MAVLINK_MSG_ID_ATTITUDE) sprintf(thisId, "%s","ATTITUDE");
	else if (id == MAVLINK_MSG_ID_SYS_STATUS) sprintf(thisId, "%s","SYS_STATUS");
	else if (id == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) sprintf(thisId, "%s","CHANNELS_OVERRIDE");
	else if (id == MAVLINK_MSG_ID_BATTERY_STATUS) sprintf(thisId, "%s","BATTERY_STATUS");
	else if (id == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) sprintf(thisId, "%s","PARAM_REQUEST_LIST");
	else if (id == MAVLINK_MSG_ID_PARAM_REQUEST_READ) sprintf(thisId, "%s","PARAM_REQUEST_READ");
	else if (id == MAVLINK_MSG_ID_PARAM_SET) sprintf(thisId, "%s","PARAM_SET");
	else if (id == MAVLINK_MSG_ID_PARAM_VALUE) sprintf(thisId, "%s","PARAM_VALUE");
	else {
		sprintf(thisId, "id %d", id);
	}
	return thisId;
}

