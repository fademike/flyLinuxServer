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

#include "tty.h"

#define BUFFER_LENGTH 2041 	// minimum buffer size that can be used with qnx (I don't know why)
#define MODEM_RF_ENABLE 0	// Enable/Disable rf part
#define LOCAL_SERVER_EN 0	// Enable Mavlink Server
#define MODEM_TTY_ENABLE 1

#ifdef MODEM_TTY_ENABLE //tty init
// // #include <errno.h>
// // #include <fcntl.h> 
// // #include <string.h>
// #include <termios.h>
// // #include <unistd.h>



#define CMD_LIST_SIZE  1000

#define DataEx(a) (a##_RD != a##_WR)
#define DataExAny(f, a) (f##_RD != a##_WR)  // for send block
#define DataArrayPP(a) if (++a>=CMD_LIST_SIZE) a=0
#define DataArrayMM(a) if (--a<0) a=(CMD_LIST_SIZE-1)


unsigned char myCyrcleTxBuff[CMD_LIST_SIZE][BUFFER_LENGTH];  // buffer for send to RF
int myCyrcleTxBuff_len[CMD_LIST_SIZE];  // buffer for send to RF
int myCyrcleTxBuff_RD; // Read pointer 
int myCyrcleTxBuff_WR; // Write pointer

unsigned char myCyrcleRxBuff[CMD_LIST_SIZE][BUFFER_LENGTH];  // buffer for send to RF
int myCyrcleRxBuff_len[CMD_LIST_SIZE];  // buffer for send to RF
int myCyrcleRxBuff_RD; // Read pointer 
int myCyrcleRxBuff_WR; // Write pointer

/// thread
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

pthread_t tid;
void* thread_for_write_to_tty(void *arg)
{
	while(1){
		if (DataEx(myCyrcleTxBuff)){
			unsigned char *buf = myCyrcleTxBuff[myCyrcleTxBuff_RD];
			int n = myCyrcleTxBuff_len[myCyrcleTxBuff_RD];
			printf("TX(%d): %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n\r", n, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
			// write(tty, myCyrcleTxBuff[myCyrcleTxBuff_RD], myCyrcleTxBuff_len[myCyrcleTxBuff_RD]);
			tty_write(myCyrcleTxBuff[myCyrcleTxBuff_RD], myCyrcleTxBuff_len[myCyrcleTxBuff_RD]);
			DataArrayPP(myCyrcleTxBuff_RD);
		}
		usleep(100);
	};
    return NULL;
}
void* thread_for_read_from_tty(void *arg)
{
	while(1){

		char b [100];
		int n_rx = tty_read(b, sizeof(b));//read (tty, b, sizeof(b));  // read up to 100 characters if ready to read
		if (n_rx>0) {

			memcpy(myCyrcleRxBuff[myCyrcleRxBuff_WR], b, n_rx);
			myCyrcleRxBuff_len[myCyrcleRxBuff_WR] = n_rx;
			DataArrayPP(myCyrcleRxBuff_WR);
			// printf("Rx tty(%d): ", n_rx);
			// int x=0;
			// for (x=0;x<n_rx;x++)printf("%d,", (unsigned char)b[x]);
			// // for (x=0;x<n_rx;x++) if (mavlink_parse_char(MAVLINK_COMM_0, b[x], &msg, &status)){	//parse rx bytes
					
			// // 		// Preparing data and sending to network
			// // 		uint16_t len = mavlink_msg_to_send_buffer(b, &msg);
			// // 		network_send(b, len);
			// // 		printf("send id: %d, len: %d\n\r", msg.msgid, len);
			// // 	}

			// printf("\n\r");


			// static char bb[200];
			// static int ptr=0;
			// int x0=0;

			// for (x0=0;x0<n_rx;x0++){
			// 	if (((unsigned char)b[x0]) == 253){
			// 		printf("Rx tty(%d): ", ptr);
			// 		int x;
			// 		for (x=0;x<ptr;x++)printf("%d,", (unsigned char)bb[x]);
			// 		printf("\n\r");
			// 		ptr=0;
			// 		bb[ptr++] = 253;
					
			// 	}else {
			// 		bb[ptr++] = b[x0];
			// 		if (ptr>=200)ptr=0;
			// 	}
			// }


		}

		usleep(1000);
	};
    return NULL;
}
////

#endif
// char *portname = "/dev/ttyUSB0";
char *portname = "/dev/ttyUSB1";
// char *portname = "/dev/ttyACM0";
// char *portname = "/dev/ttyACM1";
////

mavlink_message_t msg;		// for send parsed package to network
mavlink_status_t status;
uint8_t buf[BUFFER_LENGTH];

unsigned long GetTime_ms(void){	//FIXME no test
	struct timeval tv;
	gettimeofday(&tv, NULL);  
	uint64_t uS =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return uS/1000;
}

int main(int argc, char* argv[])
{


	// setting up network
	if (network_init() < 0) return -1;

	setvbuf(stdout, NULL, _IONBF, 0);

	if (argc>=2){
		portname = argv[1];
		printf("tty is: %s\n\r", argv[1]);
	}

#if MODEM_TTY_ENABLE

	if (tty_init(portname)<0){printf("error tty %s\n\r", portname); return -1;}

    int err;
	err = pthread_create(&tid, NULL, &thread_for_write_to_tty, NULL);
	if (err != 0) {printf("\ncan't create thread :[%s]", strerror(err)); return -1;}
	//else printf("\n Thread created successfully\n");
	err = pthread_create(&tid, NULL, &thread_for_read_from_tty, NULL);
	if (err != 0) {printf("\ncan't create thread :[%s]", strerror(err));return -1;}


#endif


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
		usleep(100);
		//	receive data from network
		int n = network_receive(buf, BUFFER_LENGTH);

		// char buf_clear[BUFFER_LENGTH];
		// while(network_receive(buf_clear, BUFFER_LENGTH)>0){};
		int i=0;
#if LOCAL_SERVER_EN
		// Send to Mavlink Server from network
		for (i=0;i<n;i++) mavlink_receive(buf[i]);
#endif
#if MODEM_RF_ENABLE
		// Send data to RF from network
		for (i=0;i<n;i++) ModemControl_SendSymbol(buf[i]);
#endif

	// if (n>0){
	// 	printf("TX(%d): %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n\r", n, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
	// }

#if MODEM_TTY_ENABLE

	//for (i=0;i<n;i++) write (fd, buf[i], 1); //mavlink_receive(buf[i]);
	//write(tty, myCyrcleBuff[myCyrcleBuff_RD], myCyrcleBuff_len[myCyrcleBuff_RD]);
	if (n>0){
		memcpy(myCyrcleTxBuff[myCyrcleTxBuff_WR], buf, n);
		myCyrcleTxBuff_len[myCyrcleTxBuff_WR] = n;
		DataArrayPP(myCyrcleTxBuff_WR);
		// if (n>0){
		// 	write (tty, buf, n); 
		// 	printf("TX(%d): %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n\r", n, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
		// }
	}
	if (DataEx(myCyrcleRxBuff)){
		
		int x=0;
		unsigned char * b = myCyrcleRxBuff[myCyrcleRxBuff_RD];
		int n_rx = myCyrcleRxBuff_len[myCyrcleRxBuff_RD];
		//for (x=0;x<n_rx;x++)printf("%d,", buf[x]);

		for (x=0;x<n_rx;x++) {
			static mavlink_message_t msg2;		// for send parsed package to network
			static mavlink_status_t status2;
			static uint8_t buf2[BUFFER_LENGTH];




			if (mavlink_parse_char(MAVLINK_COMM_0, b[x], &msg2, &status2)){	//parse rx bytes
				// Preparing data and sending to network
				uint16_t len = mavlink_msg_to_send_buffer(buf2, &msg2);
				network_send(buf2, len);
				//printf("send id: %d, len: %d\n\r", msg.msgid, len);
				// printf("send id: %d, len: %d [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\n\r", msg.msgid, len, b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12],b[13],b[14]);
				printf("send id: %d, len: %d, ", msg2.msgid, len);
				// int x=0;
				// for (x=0;x<len;x++)printf("%d,", b[x]);}//
				if (msg2.msgid == 22){printf ("--id %d",mavlink_msg_param_value_get_param_index(&msg2));}
				printf("\n\r");
			}
		}
		DataArrayPP(myCyrcleRxBuff_RD);


	}

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
		ModemControl_Work();
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
