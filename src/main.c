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
#include "ModemControl.h"
#include "nRF24.h"
#include "program_pack.h"

#include "tty.h"
#include "cb.h"

#define BUFFER_LENGTH 2041 	// minimum buffer size that can be used with qnx (I don't know why)
#define MODEM_RF_ENABLE 0	// Enable/Disable rf part (via spi)
#define LOCAL_SERVER_EN 0	// Enable Mavlink Server
#define MODEM_TTY_ENABLE 1	// connect to fly via tty

#ifdef MODEM_TTY_ENABLE //tty init

enum {	// RF commands for program mode
	CMD_DATA = 0,
	CMD_REPLY_OK = 1,
	CMD_REPLY_FAIL = 2,
};


circular_buffer cb_sendToRF;
circular_buffer cb_sendFromRF;


bool program_set = false;	// set program mode
char program_name[256];		// file name for program
int program_file = 0;
int program_answer = -1;	// for reboot fly if no answer


pthread_t tid;


#define BLOCKSIZE 50	// split the firmware file into 50 bytes in package


void program_block (uint16_t send_block, uint16_t total_block){
	uint8_t package[64];
	uint8_t * data = (uint8_t *)&package[sizeof(rf_pack_header) + sizeof(rf_pack_program_header)];
	
	lseek(program_file, (size_t)send_block*BLOCKSIZE, SEEK_SET);//CUR);

	int len = read (program_file, data, BLOCKSIZE);

	rf_pack_header * cPack_header = (rf_pack_header *)package;
	cPack_header->st = '*';
	cPack_header->len = len + sizeof(rf_pack_program_header);

	rf_pack_program_header * cPack_program_header = (rf_pack_program_header *)&package[sizeof(rf_pack_header)];
	cPack_program_header->status = 0;

	cPack_program_header->current_block = send_block;//ntohs(send_block);
	cPack_program_header->total_block = total_block;//ntohs(total_block);

	pack_setCRC(package);

	//total length:
	int t_len = sizeof(rf_pack_header) + sizeof(rf_pack_program_header) + len + sizeof(rf_pack_crc) + sizeof(uint8_t);
	uint8_t end_ptr = t_len - sizeof(uint8_t);

	package[end_ptr] = '#';

	printf("send %d, total %d, ans %d      \r", send_block, total_block, program_answer);

	cb_write(&cb_sendToRF, package, t_len);
}

void* thread_program(void *arg)	// thread programming fly
{
	int total_timeout = 100;
	int size = lseek(program_file, (size_t)0, SEEK_END);
	uint16_t total_block = size/BLOCKSIZE;
	if (size%BLOCKSIZE) total_block+=1;
	uint16_t last_block = total_block -1;
	int send_block = 0;

	while(1){

		if (program_set){		// Program device

			program_answer = pack_getRxBlock();
			uint16_t total = pack_getRxTotal();

			if (program_answer >= total_block) {printf("\n\rprogram finish\n\r"); program_set = false; break;}
			if (program_answer == send_block) send_block++;

			if (send_block >= total_block){
				if (total_timeout>0)total_timeout--;
				else {printf("\n\rprogram finish\n\r"); program_set = false; break;}
			}

			program_block(send_block, total_block);
		}
		usleep(10*1000);
	};
    return NULL;
}
void* thread_for_write_to_tty(void *arg)
{
	while(1){

		if (!cb_isEmpty(&cb_sendToRF)){
			unsigned char *buf = cb_getBuf(&cb_sendToRF, cb_RD);
			int n = cb_getBufLen(&cb_sendToRF);
			//printf("TX(%d): %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n\r", n, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
			if (!program_set){
				printf("TX(%d): ");
				for (int i=0;i<12;i++) printf("%x,", buf[i]);
				printf("\n\r");
			}
			tty_write(buf, n);
			cb_pass(&cb_sendToRF, cb_RD);
		}

		usleep(100);
	};
    return NULL;
}
void* thread_for_read_from_tty(void *arg)
{
	while(1){

		char b [100];
		int n_rx = tty_read(b, sizeof(b)); // read up to 100 characters if ready to read
		if (n_rx>0) {
			cb_write(&cb_sendFromRF, b, n_rx);
			if (program_set){
				pack_read_raw_buffer(b, n_rx);	// parse program package
			}
		}

		usleep(1000);
	};
    return NULL;
}
////

#endif
char *portname = "/dev/ttyUSB0";
// char *portname = "/dev/ttyUSB1";
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


int cmdOptionExists(int argc, char *argv[], const char * option)
{
	int f=1;
	while(f<argc){
		if (strcmp(argv[f], option) == 0) return f;
		f++;
	}
	return 0;
}

int cmdOptionGetVal(int argc, char *argv[], const char * option, int * val)
{
    int f=cmdOptionExists(argc, argv, option);
	if (f <= 0) return 0;
	if (++f <= argc) *val = atoi(argv[f]);
	return f;
}
int cmdOptionGet(int argc, char *argv[], const char * option, char * val)
{
    int f=cmdOptionExists(argc, argv, option);
	if (f <= 0) return 0;
	if (++f <= argc) memcpy(val, argv[f],strlen(argv[f])+1);
	return f;
}


int main(int argc, char* argv[])
{
	int tty_dev_set = 0;
	char tty_dev[256];

	if (cmdOptionExists(argc, argv, "--help") || cmdOptionExists(argc, argv, "-h")) {
		printf("use: %s -D ttydev -P file\n\r", argv[0]);
		printf("-D - tty device: %s default\n\r", portname);
		printf("-P - update program on fly\n\r");
		return 0;
	}
	// cmdOptionGetVal(argc, argv, "--delay", &param_delay_us);
	if (cmdOptionGet(argc, argv, "-D", tty_dev)) tty_dev_set = 1;
	if (cmdOptionGet(argc, argv, "-P", program_name)) {
		program_file = open (program_name, O_RDONLY);
		if (program_file < 0)
		{
			printf("program name fail\n\r");
		}
		else {
			program_set = true;
			    int err;
				err = pthread_create(&tid, NULL, &thread_program, NULL);
				if (err != 0) {printf("\ncan't create thread :[%s]", strerror(err)); return -1;}
			}
	}

	// init circular buffer for tty
	if (cb_init(&cb_sendToRF, 1000, 255) < 0) {
		printf("cb init cb_sendToRF error\n\r");
		return 0;
	}
	if (cb_init(&cb_sendFromRF, 1000, 255) < 0) {
		printf("cb init cb_sendFromRF error\n\r");
		return 0;
	}

	// setting up network
	if (network_init() < 0) return -1;

	setvbuf(stdout, NULL, _IONBF, 0);

	if (tty_dev_set){
		portname = tty_dev;
		printf("tty is: %s\n\r", tty_dev);
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

		int i=0;
#if LOCAL_SERVER_EN
		// Send to Mavlink Server from network
		for (i=0;i<n;i++) mavlink_receive(buf[i]);
#endif
#if MODEM_RF_ENABLE
		// Send data to RF from network
		for (i=0;i<n;i++) ModemControl_SendSymbol(buf[i]);
#endif


#if MODEM_TTY_ENABLE

		if (n>0){
			cb_write(&cb_sendToRF, buf, n);
		}
		if (!cb_isEmpty(&cb_sendFromRF)){	// read tty!!!
			
			int x=0;
			unsigned char * b = cb_getBuf(&cb_sendFromRF, cb_RD);
			int n_rx = cb_getBufLen(&cb_sendFromRF);
			// if (!program_set){
			// 	printf("  raw:");
			// 	for (int t=0;t<n_rx;t++){printf("%x,", b[t]);}
			// 	printf("\n\r");
			// }

			for (x=0;x<n_rx;x++) {
				static mavlink_message_t msg2;		// for send parsed package to network
				static mavlink_status_t status2;
				static uint8_t buf2[BUFFER_LENGTH];

				if (mavlink_parse_char(MAVLINK_COMM_0, b[x], &msg2, &status2)){	//parse rx bytes
					// Preparing data and sending to network
					uint16_t len = mavlink_msg_to_send_buffer(buf2, &msg2);
					network_send(buf2, len);
					printf("send id: %d, len: %d, ", msg2.msgid, len);
					//if (msg2.msgid == 22){printf ("--id %d",mavlink_msg_param_value_get_param_index(&msg2));}
					if (msg2.msgid == MAVLINK_MSG_ID_STATUSTEXT){
						char text[50];
						int ret = mavlink_msg_statustext_get_text(&msg2, text);
						int len = mavlink_msg_statustext_get_id(&msg2);
						int ii=0;
						for (ii=0;ii<len;ii++)if((text[ii] == '\n') || (text[ii] == '\r'))text[ii] = ' ';
						if (len>49) len = 49;
						text[len] = '\0';
						printf("[msg(%d): %s]", len, text);
						}
					printf("\n\r");
				}
			}
			cb_pass(&cb_sendFromRF, cb_RD);
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



			if (program_set){
				if (program_answer == -1){
					printf("send reboot              \n\r");
					// mavlink_send_reboot_system();

					char buf[2*1024];
					int len = mavlink_send_reboot_system(buf);
					
					cb_write(&cb_sendToRF, buf, len);

				}
			}
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
