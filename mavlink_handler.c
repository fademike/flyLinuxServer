
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

#define MAV_DEBUG 1

uint8_t target_system;
uint8_t target_component;

uint16_t chan[10];

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
	// if (ModemControl_getStatus < 0) return;
	// for (i=0; i<len;i++) ModemControl_SendSymbol(buf[i]);
	// //printf("send mav\n\r");
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

		mavlink_msg_param_value_pack(1, 200, &msg, p.param_id, fvalue, p.param_type, params_getSize(), n);}
	// if param none - set none param
	else mavlink_msg_param_value_pack(1, 200, &msg, NULL, 0, MAV_PARAM_TYPE_REAL32, params_getSize(), -1);

	mavlink_send_msg(&msg);

	// for debug
	if (find_fl == 1) {if (MAV_DEBUG) printf("param send ans %d: %d\n\r", n, (int)p.param_value.FLOAT);}
	else if (MAV_DEBUG) printf("param send ans -1\n\r");
}

void mavlink_send_heartbeat_server(void){	// package for transit system
	mavlink_message_t msg;

	uint8_t system_status = MAV_STATE_BOOT;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_heartbeat_pack(100, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, base_mode, 0, system_status);
	mavlink_send_msg(&msg);
}

void mavlink_send_heartbeat(void){
	mavlink_message_t msg;

	uint8_t system_status = MAV_STATE_BOOT;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_INVALID, base_mode, 0, system_status);
	mavlink_send_msg(&msg);
}

void mavlink_send_status(void){
	mavlink_message_t msg;
	int voltage = 4150;	//Battery_getVoltage();
	int bat = 43;//Battery_getBatPercent();
	mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, voltage, -1, bat, 0, 0, 0, 0, 0, 0, 0, 0, 0);
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

	mavlink_msg_attitude_pack(1, 200, &msg, timer_us*1000, roll, pitch, yaw, 0, 0, 0);
	mavlink_send_msg(&msg);
}

void mavlink_send_battery_status(void){
	mavlink_message_t msg;
	uint16_t voltages[1] = {4200};
	mavlink_msg_battery_status_pack(1, 200, &msg, 0, 0, 0, 0, voltages, 0, 0, 0, 45, 0, 0, 0, 0, 0);
	mavlink_send_msg(&msg);
}

void mavlink_send_time(void){
	mavlink_message_t msg;
    static uint64_t time_us=1675674266;
    static uint32_t time_ms=0;
    time_ms+=1000; time_us+=1;
    mavlink_msg_system_time_pack(1, 200, &msg, time_us, time_ms);
	mavlink_send_msg(&msg);
}

// result MAV_RESULT_ACCEPTED
void mavlink_send_cmd_ack(uint16_t command, uint8_t result, uint8_t progress){
	mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, 200, &msg, command, result, progress, 0, target_system, target_component);
	mavlink_send_msg(&msg);
}

void mavlink_receive(char rxdata){

	mavlink_message_t msg;

	float t_param[10];
	uint16_t t_cmd;

	mavlink_status_t status;

	if (mavlink_parse_char(MAVLINK_COMM_0, rxdata, &msg, &status))
	{
		// Packet received
		switch (msg.msgid){
			case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
				chan[0] = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);
				chan[1] = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);
				chan[2] = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);
				chan[3] = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);
				chan[4] = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);

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
				for (p=1;p<=params_getSize(); p++){
					mavlink_send_param(p);
					if (MAV_DEBUG) printf("reuest read ans %d\n\r", p);
				}
				break;
			}
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				mavlink_send_heartbeat();
				if (MAV_DEBUG) printf("rx HEARTBEAT\n\r");
				break;
			}
			case MAVLINK_MSG_ID_COMMAND_LONG:
			{
				uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
				switch(cmd){
					case(MAV_CMD_COMPONENT_ARM_DISARM):
						t_param[1] = mavlink_msg_command_long_get_param1(&msg);
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						if (MAV_DEBUG) printf("rx cmd ARM_DISARM %d\n\r", (int)t_param[1]);
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
				}
				break;
			}
			default:
				if (MAV_DEBUG) printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
		}
	}

}



// /* This assumes you have the mavlink headers on your include path
//  or in the same folder as this source file */
// #include <mavlink.h>

// #define PARAM_ALL 5
// struct param_struct {
// 	char param_id[16];
// 	union param_value{
// 		float FLOAT;
// 		int INT;
// 	} param_value;
// 	char param_type;
// } t_param[PARAM_ALL] = 	{	{"param1", 0.1f, MAV_PARAM_TYPE_REAL32}, 
// 							{"param2", 111, MAV_PARAM_TYPE_INT8}, 
// 							{"pid_p", 0.3f, MAV_PARAM_TYPE_REAL32}, 
// 							{"pid_d", 0.4f, MAV_PARAM_TYPE_REAL32}, 
// 							{"pid_i", 0.5f, MAV_PARAM_TYPE_REAL32} };

// #define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

// static int timer_us = 0;


// int mavlink_setTime(int t_us){
//     timer_us = t_us;
// }

// int mavlink_loop_tx(int sock, struct  sockaddr_in * gcAddr){


// 	mavlink_message_t msg;
// 	uint16_t len;
// 	uint8_t buf[BUFFER_LENGTH];
// 	int bytes_sent;

//     memset(buf, 0, BUFFER_LENGTH);

//     /*Send Heartbeat */
//     //mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
//     mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_INVALID, 
//     MAV_MODE_FLAG_SAFETY_ARMED |
//     MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
//     MAV_MODE_FLAG_STABILIZE_ENABLED |
//     MAV_MODE_FLAG_GUIDED_ENABLED |
//     MAV_MODE_FLAG_AUTO_ENABLED |
//     MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
//     len = mavlink_msg_to_send_buffer(buf, &msg);
//     bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
    
//     /* Send Status */
//     //mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//     mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 4200, -1, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//     len = mavlink_msg_to_send_buffer(buf, &msg);
//     bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof (struct sockaddr_in));
    
//     // /* Send Local Position */
//     // mavlink_msg_local_position_ned_pack(1, 200, &msg, microsSinceEpoch(), 
//     // 								position[0], position[1], position[2],
//     // 								position[3], position[4], position[5]);
//     // len = mavlink_msg_to_send_buffer(buf, &msg);
//     // bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
    
//     /* Send attitude */
//     static float pitch = 0, roll = 0, yaw = 0;
//     static int pitch_sig = 1, roll_sig = 1, yaw_sig = 1;
//     pitch+=pitch_sig*1*3.14f/180.0f;
//     roll+=roll_sig*1*3.14f/180.0f;
//     yaw+=yaw_sig*1*3.14f/180.0f;

//     #define LIMIT_VALUE_DEG 45
//     if (pitch>=(LIMIT_VALUE_DEG*3.14/180))pitch_sig = -1;
//     else if (pitch<=-(LIMIT_VALUE_DEG*3.14/180))pitch_sig = 1;
//     if (roll>=(LIMIT_VALUE_DEG*3.14/180))roll_sig = -1;
//     else if (roll<=-(LIMIT_VALUE_DEG*3.14/180))roll_sig = 1;
//     if (yaw>=(LIMIT_VALUE_DEG*3.14/180))yaw_sig = -1;
//     else if (yaw<=-(LIMIT_VALUE_DEG*3.14/180))yaw_sig = 1;

//     mavlink_msg_attitude_pack(1, 200, &msg, timer_us, pitch, roll, yaw, 0, 0, 0);
//     len = mavlink_msg_to_send_buffer(buf, &msg);
//     bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
    
//     /* Send battery status */
//     uint16_t voltages[1] = {4200};
//     mavlink_msg_battery_status_pack(1, 200, &msg, 0, 0, 0, 0, voltages, 0, 0, 0, 45, 0, 0, 0, 0, 0);
//     len = mavlink_msg_to_send_buffer(buf, &msg);
//     bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));


//     /*Send Time */
//     //static int time_us=0, time_ms=0; time_ms+=1000; time_us+=1000000;
//     static uint64_t time_us=1675674266;
//     static uint32_t time_ms=0; 
//     time_ms+=1000; time_us+=1;
//     mavlink_msg_system_time_pack(1, 200, &msg, time_us, time_ms);
//     len = mavlink_msg_to_send_buffer(buf, &msg);
//     bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));

// }



// int mavlink_loop_rx(char s){


// 	// mavlink_message_t msg;
// 	uint16_t len;
// 	uint8_t buf[BUFFER_LENGTH];
// 	// int bytes_sent;
//     // ssize_t recsize;
//     // int i = 0;
//     // unsigned int temp = 0;
//     // //socklen_t fromlen = sizeof(gcAddr);
//      uint16_t chan[10];

// 	// 	memset(buf, 0, BUFFER_LENGTH);
// 	// 	recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)gcAddr, fromlen);
// 	// 	if (recsize > 0)
//     //   	{
// 			// Something received - print out all bytes and parse packet
// 			mavlink_message_t msg;
// 			mavlink_status_t status;
			
// 			//printf("Bytes Received: %d\nDatagram: ", (int)recsize);
// 			// for (i = 0; i < recsize; ++i)
// 			// {
// 			// 	temp = buf[i];
// 				//printf("%02x ", (unsigned char)temp);
// 				if (mavlink_parse_char(MAVLINK_COMM_0, s, &msg, &status))
// 				{

// 					int u=0;
// 					uint8_t tbuf[BUFFER_LENGTH];
// 					int tlen = mavlink_msg_to_send_buffer(tbuf, &msg);
// 					for (u=0;u<tlen;u++){
// 						ModemControl_SendSymbol(tbuf[u]);
// 					}
// 					return 0;

// 					// Packet received
// 					//		printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d, mag %.2x\n", msg.sysid, msg.compid, msg.len, msg.msgid, msg.magic);
// 					//		printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d, mag %.2x\n", msg.sysid, msg.compid, msg.len, msg.msgid, msg.magic);
// 					switch (msg.msgid){
// 						case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
// 							chan[0] = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);
// 							chan[1] = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);
// 							chan[2] = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);
// 							chan[3] = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);
// 							chan[4] = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);
// 							printf("ch: %d, %d, %d, %d, %d, \n\r", chan[0], chan[1], chan[2], chan[3], chan[4]);
// 							break;
// 						case MAVLINK_MSG_ID_PARAM_SET:
// 						{
// 							mavlink_param_set_t param_set;
// 							char this_param_id[16];
// 							printf("param set\n\r");

// 							uint8_t target_system = mavlink_msg_param_set_get_target_system(&msg);
// 							uint8_t target_component = mavlink_msg_param_set_get_target_component(&msg);
// 							uint16_t param_id_len = mavlink_msg_param_set_get_param_id(&msg, this_param_id);	// name of rx param
// 							this_param_id[strlen(this_param_id)-1] = '\0';										// correction name 
// 							float this_param_value = mavlink_msg_param_set_get_param_value(&msg);
// 							uint8_t param_type = 0;
// 							// if (msg.magic != MAVLINK_STX_MAVLINK1) 
// 							param_type = _MAV_RETURN_uint8_t(&msg,  msg.len-1);
							
// 							if (param_type == MAV_PARAM_TYPE_REAL32){
// 								printf("set param REAL32: %s, %d, %f\n\r", this_param_id, param_type, this_param_value);
// 							}
// 							else {
// 								printf("set param nan: %s, %d, %f\n\r", this_param_id, param_type, this_param_value);
// 							}

// 							int i=0;
// 							int find = 0;												// flag of existence param
// 							for (i=0;i<PARAM_ALL; i++){									// find rx param in param base
// 								if (strcmp(this_param_id, t_param[i].param_id)==0){				// if rx param == param base
// 									printf("strcmp %s,  %d \n\r", t_param[i].param_id, i);
// 									find = 1;
// 									t_param[i].param_value.FLOAT = this_param_value;					// save value of param
// 									// reply with the parameter set
// 									mavlink_msg_param_value_pack(1, 200, &msg, t_param[i].param_id, t_param[i].param_value.FLOAT, param_type, PARAM_ALL, i+1);
// 									len = mavlink_msg_to_send_buffer(buf, &msg);
// 									bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
// 									printf("param set ans %d\n\r", i+1);
// 								}
// 							}
// 							if (find == 0){
// 								mavlink_msg_param_value_pack(1, 200, &msg, NULL, 0, MAV_PARAM_TYPE_REAL32, PARAM_ALL, -1);
// 								len = mavlink_msg_to_send_buffer(buf, &msg);
// 								bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
// 								printf("param set ans -1\n\r");
// 							}
// 							break;
// 						}
// 						case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
// 						{
// 							mavlink_param_set_t param_set;
// 							char this_param_id[16];

// 							printf("reuest read\n\r");

// 							uint8_t target_system = mavlink_msg_param_set_get_target_system(&msg);
// 							uint8_t target_component = mavlink_msg_param_set_get_target_component(&msg);
// 							uint16_t param_id_len = mavlink_msg_param_request_read_get_param_id(&msg, this_param_id);	// name of rx param
// 							this_param_id[strlen(this_param_id)-1] = '\0';										// correction name 
// 							int16_t this_index = mavlink_msg_param_request_read_get_param_index(&msg);
							
// 							int i=0;
// 							int find = 0;												// flag of existence param
// 							for (i=0;i<PARAM_ALL; i++){									// find rx param in param base
// 								if (strcmp(this_param_id, t_param[i].param_id)==0){				// if rx param == param base
// 									printf("strcmp %s,  %d \n\r", t_param[i].param_id, i);
// 									find = 1;
// 									// reply with the parameter set
// 									mavlink_msg_param_value_pack(1, 200, &msg, t_param[i].param_id, t_param[i].param_value.FLOAT, MAV_PARAM_TYPE_REAL32, PARAM_ALL, i+1);
// 									len = mavlink_msg_to_send_buffer(buf, &msg);
// 									bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
// 									printf("reuest read ans\n\r");
// 								}
// 							}
// 							if (find == 0){
// 								if ((0 <= (this_index - 1)) && ((this_index-1) < PARAM_ALL)){
// 									mavlink_msg_param_value_pack(1, 200, &msg, t_param[this_index-1].param_id, t_param[this_index-1].param_value.FLOAT, MAV_PARAM_TYPE_REAL32, PARAM_ALL, this_index);
// 									len = mavlink_msg_to_send_buffer(buf, &msg);
// 									bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
// 									printf("reuest read ans\n\r");
// 								}
// 								else {
// 									mavlink_msg_param_value_pack(1, 200, &msg, NULL, 0, MAV_PARAM_TYPE_REAL32, PARAM_ALL, -1);
// 									len = mavlink_msg_to_send_buffer(buf, &msg);
// 									bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
// 									printf("reuest read ans\n\r");
// 								}
// 							}
// 							break;
// 						}
// 						case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
// 						{
// 							mavlink_param_set_t param_set;
// 							char this_param_id[16];

// 							printf("reuest list\n\r");

// 							uint8_t target_system = mavlink_msg_param_set_get_target_system(&msg);
// 							uint8_t target_component = mavlink_msg_param_set_get_target_component(&msg);
							
// 							int i=0;
// 							for (i=0;i<PARAM_ALL; i++){
// 									mavlink_msg_param_value_pack(1, 200, &msg, t_param[i].param_id, t_param[i].param_value.FLOAT, t_param[i].param_type, PARAM_ALL, i+1);
// 									len = mavlink_msg_to_send_buffer(buf, &msg);
// 									bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)gcAddr, sizeof(struct sockaddr_in));
// 									usleep(100*1000);
// 									//sleep(1);
// 									printf("reuest read ans %d\n\r", i+1);
// 							}
// 							break;
// 						}
// 						case MAVLINK_MSG_ID_HEARTBEAT:
// 						{
// 							printf("rx HEARTBEAT\n\r");
// 							break;
// 						}
// 						default:
// 							printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
// 					}
// 				}
// 		//	}
// 			//printf("\n");
// 		// }
// 		// memset(buf, 0, BUFFER_LENGTH);
// 		return 0;
// }