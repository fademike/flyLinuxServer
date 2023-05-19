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

/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */

#include <mavlink.h>

#include "network.h"
#include "mavlink_handler.h"
#include "ModemControl.h"
#include "nRF24.h"


#define BUFFER_LENGTH 2041 	// minimum buffer size that can be used with qnx (I don't know why)
#define MODEM_RF_ENABLE 1	// Enable/Disable rf part
#define LOCAL_SERVER_EN 0	// Enable Mavlink Server


mavlink_message_t msg;		// for send parsed package to network
mavlink_status_t status;
uint8_t buf[BUFFER_LENGTH];


int main(int argc, char* argv[])
{
	// setting up network
	if (network_init() < 0) return -1;

	setvbuf(stdout, NULL, _IONBF, 0);




#if MODEM_RF_ENABLE
	int ret = -1;
	while(ret < 0){
		ret = ModemControl_init();	// init RF part
		if (ret <0) printf("nRF_init false\n\r");
		else printf("nRF_init OK\n\r");
		//sleep(1);
		usleep(1000*1000);
	}
#endif

	while (1){
		//	receive data from network
		int n = network_receive(buf, BUFFER_LENGTH);
		int i=0;
#if LOCAL_SERVER_EN
		// Send to Mavlink Server from network
		for (i=0;i<n;i++) mavlink_receive(buf[i]);
#endif
#if MODEM_RF_ENABLE
		// Send data to RF from network
		for (i=0;i<n;i++) ModemControl_SendSymbol(buf[i]);
#endif

		// Sending some packages after a while
		struct timeval tv;
		gettimeofday(&tv, NULL);  
		uint64_t uS =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		static uint64_t l_uS = 0;
#if LOCAL_SERVER_EN
		if (((uint64_t)(uS - l_uS)) > 100*1000){	// every 100 ms
			l_uS = uS;
			mavlink_send_attitude();
		}
#else 
		// if this system is retranslator:
		// need send mavlink hert bit to host for connection detect
		if (((uint64_t)(uS - l_uS)) > 1000*1000){	// every 1 s
			l_uS = uS;
			mavlink_send_heartbeat_server();	// send data to network by this system
		}
#endif

#if MODEM_RF_ENABLE
		// Packet reception cycle. and sending if necessary
		ModemControl_Loop();
		char b = 0;
		while (ModemControl_GetByte(&b) > 0) {	// if a byte is received by RF
			if (mavlink_parse_char(MAVLINK_COMM_0, b, &msg, &status)){	//parse rx bytes
				
				// Preparing data and sending to network
				uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
				network_send(buf, len);
				printf("send id: %d, len: %d\n\r", msg.msgid, len);
			}

		}
#endif

	}
	return 0;

}
