
#include <string.h>
#include <stdio.h>

#define nRF24 1

#ifdef nRF24

#include "nRF24.h"
#define PACKET_LEN 32	      // package Length
#define PACKET_LEN_CRC 2	      // package Length
#define PACKET_DATALEN 28   // the size of the data in the package
#define CMD_LIST_SIZE 10    // number of packages

#pragma pack(push, 1)
struct nrf_pack {
  unsigned char type;
  unsigned char len;
  unsigned char data[PACKET_DATALEN];
  unsigned char crc[2];
};
#pragma pack(pop)

#endif

#define DataEx(a) (a##_RD != a##_WR)    //Is there any data to send or read
#define DataArrayPP(a) if (++a>=CMD_LIST_SIZE) a=0      // increase pointer
#define DataArrayMM(a) if (--a<0) a=(CMD_LIST_SIZE-1)   // decrease pointer


static int ModemControl_status = -1;

unsigned char toSendCmdbuf[CMD_LIST_SIZE][PACKET_DATALEN];  // buffer for send to RF
int toSendCmd_RD; // Read pointer 
int toSendCmd_WR; // Write pointer

unsigned char toRxCmdbuf[CMD_LIST_SIZE][PACKET_DATALEN];
int toRxCmd_RD; // Read pointer 
int toRxCmd_WR; // Write pointer

typedef enum  {
	PACKET_NONE 		= 0,
	PACKET_DATA			= 1,    // data pccket
	PACKET_ASK			= 2,    // it is ask
	PACKET_ANSWER		= 3,    // it is answer
	PACKET_DATA_PACK	= 4,  // Part of block packet. Need wait other data
} PACKET_TYPE;


#define SYNCHRO_MODE 1	// 1- synchronization mode; 0- simple mode. More in stm8+si4463 project
#if SYNCHRO_MODE

#define SYNCHRO_TIME 500                     // sync time
#define SYNCHRO_TWAIT 24//6*4               // TIMEOUT of ANSWER = (TIME of TX 1 packet) + SYNCHRO_TWAIT    // 10bps - 100 TWAIT(57us); 100kbps - 6TWAIT(6us)

volatile int delay_to_wait_answer=0;  
volatile int delay_measure_tx=0;
volatile int delay_synchro=0;
char NeedAnswer=0;

#endif 

// Copies the packet to the buffer for sending
void ModemControl_SendPacket(char * buff)
{
	memcpy(toSendCmdbuf[toSendCmd_WR], buff, PACKET_DATALEN); // copy packet for send
	DataArrayPP(toSendCmd_WR);  // increase painter for send
}
void ModemControl_SendSymbol(char data)
{
	static int ptr=0; // pointer to fill in the block package
	static char buff[PACKET_DATALEN]; // buffer of the filled package
	buff[ptr++] = data; // add new data to buffer
	if (ptr>=PACKET_DATALEN){ // if the buffer is full
    memcpy(toSendCmdbuf[toSendCmd_WR], buff, PACKET_DATALEN); // copy full buffer for send
    ptr=0; 
    DataArrayPP(toSendCmd_WR);
  }
}
// If there is a package, puts the package in the pointer (buf)
// Returned 1 if there is packet. 0 no packet
int ModemControl_GetPacket(char * buf){
    if (!DataEx(toRxCmd)) return 0;

    memcpy(buf, toRxCmdbuf[toRxCmd_RD], 32);
    DataArrayPP(toRxCmd_RD);
    return 1;
}
// Reads received packets and outputs one byte to pointer (buf)
// Returned 1 if there is data. 0 no data
int ModemControl_GetByte(char * buf){     //FIXME !!!
  struct nrf_pack * pack =  (struct nrf_pack *)toRxCmdbuf[toRxCmd_RD];
    static int pos = 0;
    int read = 0; // flag of return data
    if (!DataEx(toRxCmd)) {pos=0; return 0;}

    char cnt = pack->len&0x3F;
    if (cnt > PACKET_DATALEN) cnt=PACKET_LEN;
    if (cnt>pos){
        *buf = toRxCmdbuf[toRxCmd_RD][2+pos]; // 2+pos, because datain pack start from second byte
        pos++;
        read = 1;
    }
    if (pos >= cnt){  // all bytes in this package geting 
        pos = 0;
        DataArrayPP(toRxCmd_RD);  //increase pointer
    }
    return read;
}


void CRC_PacketCalculate(unsigned char * buff){
	unsigned char mCRC = 87;
	int x=0;
	for (x=0;x<(PACKET_LEN-PACKET_LEN_CRC);x++)mCRC += buff[x]*3; // algorithm of calculation
	buff[PACKET_LEN-PACKET_LEN_CRC] = mCRC;
}

int CRC_PacketCheck(unsigned char * buff){
	unsigned char mCRC = 87;  // start byte for calculate my crc
	int x=0;
	for (x=0;x<(PACKET_LEN-PACKET_LEN_CRC);x++)mCRC += buff[x]*3; // algorithm of calculation
	return  (buff[PACKET_LEN-PACKET_LEN_CRC] == mCRC);
}

int ModemControl_getStatus(void){
	return ModemControl_status;
}

int ModemControl_init(void){
	ModemControl_status = nRF24_init();
	if (ModemControl_status < 0) printf("nRF_init false\n\r");
	else printf("nRF_init OK\n\r");

	return ModemControl_status;
}

int ModemControl_Read(void){

  unsigned char status = SPI_Read_Reg(STATUS);

 	if(status & 0x40){         // if there is new package

 		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);

 		unsigned char fifo_status = 0;
 		do{
 		  unsigned char rbuff[PACKET_LEN];
      struct nrf_pack * pack = (struct nrf_pack *) rbuff;

     	SPI_Read_Buf(RD_RX_PLOAD,rbuff,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

 #if SYNCHRO_MODE
      if (pack->type == PACKET_NONE ){
        NeedAnswer = 0; delay_to_wait_answer = 0;
      }
      else if (pack->type == PACKET_ASK ){
        NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
      }
      else if (pack->type == PACKET_ANSWER ){
        NeedAnswer = 0; delay_to_wait_answer = 0;
      }
      else
 #endif
      if ((pack->type == PACKET_DATA) || (pack->type == PACKET_DATA_PACK)){                                   // if Packet is PACKET_DATA

 #if SYNCHRO_MODE
        if (pack->type == PACKET_DATA) {NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;}
        else {NeedAnswer = 0; delay_to_wait_answer = SYNCHRO_TWAIT;}
 #endif
        char len = pack->len&0x3F; // datalen of package
        if (len){ // if no zero len
          if 	(CRC_PacketCheck(rbuff)){
            memcpy(toRxCmdbuf[toRxCmd_WR], rbuff, PACKET_LEN); 
            DataArrayPP(toRxCmd_WR);
          }
        }
      }

			fifo_status=SPI_RW_Reg(FIFO_STATUS, 0);
		}while((fifo_status&0x01) != 0x01);

 		RX_Mode(); // Switch to RX mode

    return 1;
  }

 	if(status&TX_DS)
 	{
 		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, TX_DS);  	// Reset bit TX_DS
 	}
 	if(status&MAX_RT)
 	{
 		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, MAX_RT); 	// Reset bit MAX_RT
 	}

  return 0;
}




void ModemControl_Loop(void)
{

  if (ModemControl_Read() != 0) return; // if there was a data reception, need to repeat

  // prepare to tx data:
#if SYNCHRO_MODE
  if (delay_to_wait_answer != 0){return;}                          //The timer between the packets being sent. Otherwise, heap data was not accepted.
#endif
  unsigned char DataToSend = 0;

  if (DataEx(toSendCmd)){

    delay_measure_tx = 0;

    DataToSend=PACKET_DATALEN;  // type of packet

    unsigned char buf[PACKET_LEN];
    buf[0] = PACKET_DATA;
    buf[1] = (DataToSend)&0x3F;
    memcpy(&buf[2], toSendCmdbuf[toSendCmd_RD], PACKET_DATALEN);
    DataArrayPP(toSendCmd_RD);
    TX_Mode(buf);//NRF24L01_Send(buf);

    // calculate rx delay:
    delay_measure_tx++;

    int delay = 10000;
    delay=0;
    int t=0;
    for (t=0;t<delay;t++){}		//TODO change to normal view
#if SYNCHRO_MODE
    delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //50; //Set timer between send the packets
    delay_synchro = SYNCHRO_TIME;    //else delay_to_wait_answer = 10;	//delay_to_wait_answer = 10;  //50;
    NeedAnswer=0;	//FIXME
#endif

    RX_Mode();

  }

#if SYNCHRO_MODE

  if ((DataToSend == 0) && ((NeedAnswer>0) || (delay_synchro>0))){      //Maintain communication by sync pulses.

    //Delay_ms(delay_measure_tx_middle);                                  //So as not to foul the broadcast.
    unsigned char wbuff[64];
    if (NeedAnswer) {wbuff[0] = NeedAnswer; NeedAnswer=0;}              // Pulse is Answer
    else if (delay_synchro>0) wbuff[0] = PACKET_ASK;                    // Pulse is Ask
    else wbuff[0] = PACKET_NONE;                                        // Pulse is None

    delay_measure_tx = 0;

    //if (SPI_Read_Reg(CD) != 0) {return -1;}

    TX_Mode(wbuff);//NRF24L01_Send(buf);

    delay_measure_tx++;
    delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //delay_to_wait_answer = delay_measure_tx*4;  //50;


    RX_Mode();

  }
#endif

}