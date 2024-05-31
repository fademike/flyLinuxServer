
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <arpa/inet.h>


#include "program_pack.h"


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
	int i;
	int len = pack_getLen(pack);
	uint16_t crc = pack_returnCRC(pack, len);
	rf_pack_crc * pack_crc = (rf_pack_crc *)&pack[sizeof(rf_pack_header) + len];
	pack_crc->crc = crc;//htons(crc);
}
bool pack_checkCRC(uint8_t * pack){
	int i=0;
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
	rx_block = p_header->current_block;//htons(p_header->current_block);//pack[3]<<8 | (pack[4]&0xFF);
	rx_total = p_header->total_block;//htons(p_header->total_block);//pack[5]<<8 | (pack[6]&0xFF);
	// printf("rx pack %d, %d\n\r", rx_block, rx_total);


    pack_rx_block = rx_block;
    pack_rx_total = rx_total;

	// if (p_header->status != CMD_REPLY_OK) return 0;
	// if (rx_block > rx_total) program_set = false;	// error
	// if (rx_block == rx_total) program_set = false;	//finish
	// program_answer = rx_block;
    // return 0;
}
int16_t pack_getRxBlock(void){
    return pack_rx_block;
}
int16_t pack_getRxTotal(void){
    return pack_rx_total;
}

void pack_read_raw_byte(uint8_t data){
	static int pos = 0;
	static uint8_t len = 0;
	static uint8_t clen = 0;
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
	int i=0;
	for (i=0;i<len; i++) pack_read_raw_byte(data[i]);
}

