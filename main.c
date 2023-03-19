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



#include "network.h"
#include "mavlink_handler.h"
#include "ModemControl.h"
#include "nRF24.h"



#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

#include <mavlink.h>
mavlink_message_t msg;
mavlink_status_t status;
uint8_t buf[BUFFER_LENGTH];


int main(int argc, char* argv[])
{

	if (network_init() < 0) return -1;

	setvbuf(stdout, NULL, _IONBF, 0);

	int ret = -1;

#define MODEM_RF_ENABLE 1
#define LOCAL_SERVER_EN 0

#if MODEM_RF_ENABLE
	while(ret < 0){
		ret = ModemControl_init();
		if (ret <0) printf("nRF_init false\n\r");
		sleep(1);
	}

	printf("nRF_init OK\n\r");
#endif

	while (1){
		//mavlink_loop_rx(sock, &gcAddr, &fromlen);
		int n = network_receive(buf, BUFFER_LENGTH);
		int i=0;
#if LOCAL_SERVER_EN
		for (i=0;i<n;i++) mavlink_receive(buf[i]);
#endif
#if MODEM_RF_ENABLE
		for (i=0;i<n;i++) ModemControl_SendSymbol(buf[i]);
#endif

		struct timeval tv;
		gettimeofday(&tv, NULL);  
		uint64_t uS =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		static uint64_t l_uS = 0;
		if (((uint64_t)(uS - l_uS)) > 100*1000){
			l_uS = uS;
			if (LOCAL_SERVER_EN) mavlink_send_attitude();
		}

#if MODEM_RF_ENABLE
		ModemControl_Work();
		char b = 0;
		while (ModemControl_GetByte(&b) > 0) {
			if (mavlink_parse_char(MAVLINK_COMM_0, b, &msg, &status)){
				
				uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
				network_send(buf, len);
				printf("send id: %d, len: %d\n\r", msg.msgid, len);
			}

		}
#endif

	}

	printf("exit\n\r");
	return 0;

}
