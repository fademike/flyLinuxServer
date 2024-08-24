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

/// thread
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>

#include <mavlink.h>

#include "network.h"
#include "mavlink.h"
#include "mavlink_handler.h"
#include "program_pack.h"

#include "tty.h"
#include "cb.h"

#define BUFFER_LENGTH 2041 	// minimum buffer size that can be used with qnx (I don't know why)
// #define LOCAL_SERVER_EN	// Enable Mavlink Server
#define MODEM_TTY_ENABLE	// connect to fly via tty

#ifdef MODEM_TTY_ENABLE //tty init

// for circular buffer
#define BUF_SIZE 1000
#define BUF_PACK_LEN 255

#endif
circular_buffer cb_ttyRead;	// data from tty

bool quiet = false;

char * portname = "/dev/ttyUSB0";

unsigned long GetTime_ms(int reset){
	struct timeval tv;
	gettimeofday(&tv, NULL);  
	uint64_t uS =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	static uint64_t l_uS = 0;
	if (reset) l_uS = uS;
	if (uS < l_uS) l_uS = uS;	// if overflow
	return (uint64_t)(uS - l_uS)/1000;	// return dt ms
}

int cmdOptionExists(int argc, char *argv[], const char * option)
{
	int f=0;
	while(++f < argc){
		if (strcmp(argv[f], option) == 0) return f;
	}
	return 0;
}

int cmdOptionGetVal(int argc, char *argv[], const char * option, int * val)
{
    int f = cmdOptionExists(argc, argv, option);
	if ((f <= 0) || ((f+1) >= argc)) return 0;
	if (++f <= argc) *val = atoi(argv[f]);
	return f;
}
int cmdOptionGet(int argc, char *argv[], const char * option, char ** val)
{
    int f = cmdOptionExists(argc, argv, option);
	if ((f <= 0) || ((f+1) >= argc)) return 0;
	*val = (char *)argv[f+1];
	// if (++f <= argc) memcpy(*val, argv[f], strlen(argv[f])+1);
	return f;
}


int main(int argc, char* argv[])
{
	char * program_name;		// file name for program

	if (cmdOptionExists(argc, argv, "--help") || cmdOptionExists(argc, argv, "-h")) {
		printf("use: %s -D ttydev -P file\n\r", argv[0]);
		printf("-D - tty device: %s default\n\r", portname);
		printf("-P - update program on fly\n\r");
		printf("-q - quite mode (only msg to stdout)\n\r");

		return 0;
	}
	// cmdOptionGetVal(argc, argv, "--delay", &param_delay_us);
	if (cmdOptionGet(argc, argv, "-D", &portname)) printf("tty is: %s\n\r", portname);
	if (cmdOptionExists(argc, argv, "-q")) quiet = true;
	if (cmdOptionGet(argc, argv, "-P", &program_name)) {
		int program_file = open (program_name, O_RDONLY);
		if (program_file < 0)
		{
			printf("program name fail\n\r");
		}
		else {
			pthread_t tid;
			int err = pthread_create(&tid, NULL, &thread_program, &program_file);
			if (err != 0) {printf("\ncan't create thread :[%s]", strerror(err)); return -1;}
		}
	}


	// setting up network
	if (network_init() < 0) return -1;

	// no buffer for stdout
	setvbuf(stdout, NULL, _IONBF, 0);	


#ifdef MODEM_TTY_ENABLE

	// init circular buffer for tty
	if (cb_init(&cb_ttyRead, BUF_SIZE, BUF_PACK_LEN) < 0) {
		printf("cb init cb_ttyRead error\n\r");
		return 0;
	}
	// tty init
	if (tty_init(portname) < 0){printf("error tty %s\n\r", portname); return -1;}

	pthread_t tid;
    int err = pthread_create(&tid, NULL, &thread_for_read_from_tty, NULL);
	if (err != 0) {printf("\ncan't create thread :[%s]", strerror(err));return -1;}

#endif

	while (1){
		uint8_t nbuf[BUFFER_LENGTH];
		//	receive data from network
		int n = network_receive(nbuf, BUFFER_LENGTH);

#ifdef LOCAL_SERVER_EN
		// Send to Mavlink Server from network
		for (int i=0;i<n;i++) mavlink_receive(nbuf[i]);

		if (GetTime_ms(0) > 100){	// every 100 ms
			GetTime_ms(1);
			mavlink_send_heartbeat_server();
			mavlink_send_attitude();
		}
#else 
		// if this system is retranslator:
		// need send mavlink hert bit to host for connection detect
		if (GetTime_ms(0) > 1000){	// every 1 s
			GetTime_ms(1);
			mavlink_send_heartbeat();	// send data to network by this system

			if (program_getStatus() == PROGRAM_NONE){
				printf("send reboot              \n\r");
				uint8_t buf[BUFFER_LENGTH];
				int len = mavlink_send_reboot_system((char *)buf);
				tty_write(buf, len);
			}
		}
#endif

#ifdef MODEM_TTY_ENABLE

		if (n>0){	// send from network to tty
			if (n>=BUF_PACK_LEN) n = BUF_PACK_LEN - 1;
			if (program_getStatus() == PROGRAM_FINISH) {
				tty_write(nbuf, n);//cb_write(&cb_sendToRF, buf, n);
				// printf("TX: %2x, %2x, %2x, %2x, %2x, \n\r", nbuf[0], nbuf[1],nbuf[2],nbuf[3],nbuf[4]);

				static mavlink_message_t msg;		// for send parsed package to network
				static mavlink_status_t status;
				static uint8_t tbuf[BUFFER_LENGTH];	//tty buf

				for (int x=0;x<n;x++) if (mavlink_parse_char(MAVLINK_COMM_0, nbuf[x], &msg, &status)){	//parse rx bytes
					// Preparing data and sending to network
					// uint16_t len = mavlink_msg_to_send_buffer(tbuf, &msg);
					printf("TX: %s\n\r", mavId2str(msg.msgid));

				}
			}
		}
		while (!cb_isEmpty(&cb_ttyRead)){	// read tty!!!
			
			uint8_t * b = cb_getBuf(&cb_ttyRead, cb_RD);
			int n_rx = cb_getBufLen(&cb_ttyRead);

			if (program_getStatus() != PROGRAM_FINISH){		// for firmware load
				pack_read_raw_buffer(b, n_rx);	// parse program package
			}

			// network_send((char *)b, n_rx);
			for (int x=0;x<n_rx;x++) {	// parse mavlink to find msg commands
				static mavlink_message_t msg;		// for send parsed package to network
				static mavlink_status_t status;
				static uint8_t tbuf[BUFFER_LENGTH];	//tty buf

				if (mavlink_parse_char(MAVLINK_COMM_0, b[x], &msg, &status)){	//parse rx bytes
					// Preparing data and sending to network
					uint16_t len = mavlink_msg_to_send_buffer(tbuf, &msg);
					network_send((char *)tbuf, len);
					if (!quiet)printf("send id: %s, len: %d, ", mavId2str(msg.msgid), len);
					//if (msg2.msgid == 22){printf ("--id %d",mavlink_msg_param_value_get_param_index(&msg2));}
					if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT){
						char text[50];
						mavlink_msg_statustext_get_text(&msg, text);
						int len = mavlink_msg_statustext_get_id(&msg);
						
						for (int i=0;i<len;i++)if((text[i] == '\n') || (text[i] == '\r'))text[i] = ' ';
						if (len>=sizeof(text)) len = sizeof(text)-1;
						text[len] = '\0';
						if (quiet) printf("%s\n\r", text);
						else printf("[msg(%d): %s]", len, text);
					}
					if (!quiet)printf("\n\r");
				}
			}
			cb_pass(&cb_ttyRead, cb_RD);
		}
		usleep(100);

#endif

	}
	return 0;

}
