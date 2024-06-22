
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "program_pack.h"
#include "tty.h"

uint16_t pack_rx_block = -1;
uint16_t pack_rx_total = -1;


int pack_getLen(uint8_t * pack){
	int len = pack[1];
	if (len >= 60) return 0;
	return len;
}
uint16_t pack_returnCRC(uint8_t * pack, int len){
	int i;
	uint16_t crc = 0;
	for (i=0;i<(len+sizeof(rf_pack_header));i++) crc+= pack[i]*87;	// calculate crc
	return crc;
}
// return total pack len
void pack_setCRC(uint8_t * pack){
	int len = pack_getLen(pack);
	uint16_t crc = pack_returnCRC(pack, len);
	rf_pack_crc * pack_crc = (rf_pack_crc *)&pack[sizeof(rf_pack_header) + len];
	pack_crc->crc = crc;//htons(crc);
}
bool pack_checkCRC(uint8_t * pack){
	int len = pack_getLen(pack);
	uint16_t crc = pack_returnCRC(pack, len);
	rf_pack_crc * pack_crc = (rf_pack_crc *)&pack[sizeof(rf_pack_header) + len];
	if (pack_crc->crc == crc) return true;//htons(crc)) return true;
	return false;
}

void pack_read_single(uint8_t * pack){

	if (!pack_checkCRC(pack)) return;

	uint16_t rx_block = 0;
	uint16_t rx_total = 0;
	rf_pack_program_header * p_header = (rf_pack_program_header *) &pack[sizeof(rf_pack_header)]; 
	rx_block = p_header->current_block;
	rx_total = p_header->total_block;

    pack_rx_block = rx_block;
    pack_rx_total = rx_total;
}
int16_t pack_getRxBlock(void){
    return pack_rx_block;
}
int16_t pack_getRxTotal(void){
    return pack_rx_total;
}

void pack_read_raw_byte(uint8_t data){
	static int pos = 0;
	static uint8_t len = 0;	// length of the data in the packet
	static uint8_t clen = 0;	//pointer to the current byte in the data
	static uint8_t rx_pack[64];

	rx_pack[pos + clen] = data;

	if ((pos == 0) && (data == '*')) {pos++;}									//*
	else if (pos == 1) {len = data; if (len >= 60)len = 0; clen = 0; pos++;}	//len
	else if ((pos == 2) && (len > clen)) {clen++;}								//data
	else if ((pos == 2) && (len == clen)) {pos++;}								//crc1
	else if ((pos == 3)) pos++;													//crc2
	else if ((pos == 4) && (data == '#')) {pack_read_single(rx_pack); pos=0; clen=0;}	//#
	else pos = 0;
}
void pack_read_raw_buffer(uint8_t * data, int len){
	for (int i=0;i<len; i++) pack_read_raw_byte(data[i]);
}


#define BLOCKSIZE 50	// split the firmware file into 50 bytes in package

int program_block (int file, uint16_t send_block, uint16_t total_block, uint8_t * package){
	// uint8_t package[64];
	uint8_t * data = (uint8_t *)&package[sizeof(rf_pack_header) + sizeof(rf_pack_program_header)];
	
	lseek(file, (size_t)send_block*BLOCKSIZE, SEEK_SET);

	int len = read (file, data, BLOCKSIZE);

	rf_pack_header * cPack_header = (rf_pack_header *)package;
	cPack_header->st = '*';
	cPack_header->len = len + sizeof(rf_pack_program_header);

	rf_pack_program_header * cPack_program_header = (rf_pack_program_header *)&package[sizeof(rf_pack_header)];
	cPack_program_header->status = 0;

	cPack_program_header->current_block = send_block;
	cPack_program_header->total_block = total_block;

	pack_setCRC(package);

	//total length:
	int t_len = sizeof(rf_pack_header) + sizeof(rf_pack_program_header) + len + sizeof(rf_pack_crc) + sizeof(uint8_t);
	uint8_t end_ptr = t_len - sizeof(uint8_t);

	package[end_ptr] = '#';

	return t_len;
}

int program_getCntBlock(int file){
	int size = lseek(file, (size_t)0, SEEK_END);
	int total_block = size/BLOCKSIZE;
	if (size%BLOCKSIZE) total_block+=1;
	return total_block;
}

static int program_status = PROGRAM_FINISH;//PROGRAM_ERROR;

int program_getStatus(){
	return program_status;
}

void* thread_program(void *arg)	// thread programming fly
{
	int timeout = 100;
	int program_file = *(int *)arg;
	uint16_t total_block = program_getCntBlock(program_file);
	int send_block = 0;
	int program_answer = -1;

	program_status = PROGRAM_NONE;

	while(program_status != PROGRAM_FINISH){

		program_answer = pack_getRxBlock();
		uint16_t total = pack_getRxTotal();

		if (program_answer != -1) program_status = PROGRAM_RUN;
		if (program_answer == send_block) send_block++;
		if ((send_block >= total_block) || (program_answer >= total_block)){
			if (timeout-- < 0){
				printf("\n\rprogram finish\n\r"); 
				program_status = PROGRAM_FINISH; 
				return;
			}
		}
		uint8_t package[64];
		int len = program_block(program_file, send_block, total_block, package);
		tty_write(package, len);

		printf("send %d, total %d, ans %d      \r", send_block, total_block, program_answer);

		usleep(10*1000);
	};
    return NULL;
}