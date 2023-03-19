
#include <string.h>
#include <stdio.h>

#define nRF24 1

#define READCMDSTREAM_ARRAY 10

#define SYNCHRO_MODE 1	// 1- synchronization mode; 0- simple mode. More in stm8+si4463 project
////////////////////////////For SYNCHRO_MODE :
#define SYNCHRO_TIME 500                     // sync time
#define SYNCHRO_TWAIT 6*4//6*4               // TIMEOUT of ANSWER = (TIME of TX 1 packet) + SYNCHRO_TWAIT    // 10bps - 100 TWAIT(57us); 100kbps - 6TWAIT(6us)



#ifdef nRF24

#include "nRF24.h"
#define PACKET_DATALEN 28

//#define CMD_BUF_SIZE 32     // max buffer nRF32L01
#define CMD_LIST_SIZE 10    // any num RAM

#define PACKET_LEN 32	// length receive buffer of modem

#endif

#define DataEx(a) (a##_RD != a##_WR)
#define DataArrayPP(a) if (++a>=CMD_LIST_SIZE) a=0
#define DataArrayMM(a) if (--a<0) a=(CMD_LIST_SIZE-1)


static int ModemControl_status = -1;

unsigned char toSendCmdbuf[CMD_LIST_SIZE][PACKET_DATALEN];
int toSendCmd_RD;
int toSendCmd_WR;

unsigned char toRxCmdbuf[CMD_LIST_SIZE][PACKET_DATALEN];
int toRxCmd_RD;
int toRxCmd_WR;


typedef enum  {
	PACKET_NONE 		= 0,
	PACKET_DATA			= 1,
	PACKET_ASK			= 2,
	PACKET_ANSWER		= 3,
	PACKET_DATA_PACK	= 4,
} PACKET_TYPE;

volatile int delay_to_wait_answer=0;
volatile int delay_measure_tx=0;
volatile int delay_synchro=0;
char NeedAnswer=0;
//int delay_measure_tx_middle=0;


void ModemControl_SendPacket(char * buff)
{
	memcpy(toSendCmdbuf[toSendCmd_WR], buff, PACKET_DATALEN);
	DataArrayPP(toSendCmd_WR);
}
void ModemControl_SendSymbol(char data)
{
	static int i=0;
	static char buff[PACKET_DATALEN];
	buff[i++] = data;
	if (i>=PACKET_DATALEN){memcpy(toSendCmdbuf[toSendCmd_WR], buff, PACKET_DATALEN); i=0; DataArrayPP(toSendCmd_WR);}
}

int ModemControl_GetPacket(char * buf){
    if (!DataEx(toRxCmd)) return 0;

    memcpy(buf, toRxCmdbuf[toRxCmd_RD], 32);
    DataArrayPP(toRxCmd_RD);
    return 1;
}
int ModemControl_GetByte(char * buf){
    static int pos = 0;
    int read = 0;
    if (!DataEx(toRxCmd)) {pos=0; return 0;}

    char cnt = toRxCmdbuf[toRxCmd_RD][1]&0x3F;
    if (cnt > 32) cnt=32;
    if (cnt>pos){
        *buf = toRxCmdbuf[toRxCmd_RD][2+pos];
        pos++;
        read = 1;
    }
    if (pos >= cnt){
        //printf("read %d, %d\n\r", cnt, toRxCmd_RD);
        pos = 0;
        DataArrayPP(toRxCmd_RD);
    }
    //printf("(%d) ", pos);
    return read;
}


void CRC_PacketCalculate(unsigned char * buff){
	unsigned char mCRC = 87;
	int x=0;
	for (x=0;x<30;x++){mCRC += buff[x]*3;}
	buff[30] = mCRC;
}

int CRC_PacketCheck(unsigned char * buff){
	unsigned char mCRC = 87;
	int x=0;
	for (x=0;x<30;x++){mCRC += buff[x]*3;}
	//Printf("CRC 0x%x, 0x%x ",buff[30], mCRC);
	return  (buff[30] == mCRC);
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

int ModemControl_ReadOnly(void){

  unsigned char status=0x01;

  status=SPI_Read_Reg(STATUS);



 	if(status & 0x40){//if ((xStatus&(1<<4))!=0){	                                                //PACKET_RX

 		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);

 		unsigned char fifo_status = 0;
 		do{
 		  unsigned char rbuff[PACKET_LEN];

     	SPI_Read_Buf(RD_RX_PLOAD,rbuff,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer


 #if SYNCHRO_MODE
              if (rbuff[0] == PACKET_NONE ){
                NeedAnswer = 0; delay_to_wait_answer = 0;
              }
              else if (rbuff[0] == PACKET_ASK ){
                
                NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
              }
              else if (rbuff[0] == PACKET_ANSWER ){
                NeedAnswer = 0; delay_to_wait_answer = 0;
              }
              else
 #endif
             	 if ((rbuff[0] == PACKET_DATA) || (rbuff[0] == PACKET_DATA_PACK)){                                   // if Packet is PACKET_DATA

                      if (rbuff[0] == PACKET_DATA) {NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;}
                      else {NeedAnswer = 0; delay_to_wait_answer = SYNCHRO_TWAIT;}

                char cnt = rbuff[1]&0x3F;
                int i=2;
                if (cnt >=1){

					if 	(CRC_PacketCheck(rbuff)){
						memcpy(toRxCmdbuf[toRxCmd_WR], rbuff, 32);
						DataArrayPP(toRxCmd_WR);
                        
					}
					else {
					}

             	//    ModemControl_ReadMSGRfomRF_Callback(rbuff);
                //   do{
                //  	 ModemControl_ReadSymbolRfomRF_Callback(rbuff[i++]);

                //   }while(--cnt>0);
                }
        }

				fifo_status=SPI_RW_Reg(FIFO_STATUS, 0);//SPI_Read_Reg(FIFO_STATUS);
			}while((fifo_status&0x01) != 0x01);

 		    RX_Mode(); // Switch to RX mode//changeState(STATE_RX);

        return 1;//continue;
      }


 	if(status&TX_DS)
 	{
 		//Printf("data sended INRX\r\n");
 		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);
 	}
 	if(status&MAX_RT)
 	{
 	//Printf("NO SEND..INRX\n\r");
 		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);
 	}


  return 0;

}




void ModemControl_Work(void)
{


    if (ModemControl_ReadOnly() != 0) return;



     if (1){   //tx data

       if (delay_to_wait_answer != 0){return;}                          //The timer between the packets being sent. Otherwise, heap data was not accepted.

         unsigned char DataToSend = 0;
         //if(UartRxBuffer_rd!=UartRxBuffer_wr){

        	if (DataEx(toSendCmd)){


           delay_measure_tx = 0;

           DataToSend=PACKET_DATALEN;

            unsigned char buf[PACKET_LEN];
            buf[0] = PACKET_DATA;
            buf[1] = (DataToSend)&0x3F;
            memcpy(&buf[2], toSendCmdbuf[toSendCmd_RD], PACKET_DATALEN);
            DataArrayPP(toSendCmd_RD);
            TX_Mode(buf);//NRF24L01_Send(buf);


           delay_measure_tx++;

           int delay = 10000;
           //if (BuffToTx_ptr_Rd == BuffToTx_ptr_Wr)
        	   delay=0;

           int t=0;
           for (t=0;t<delay;t++){}		//TODO change to normal view
           //Printf("Data sended %d\n\r", UartRxBuffer_rd/60);

           if (SYNCHRO_MODE) delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //50; //Set timer between send the packets
           delay_synchro = SYNCHRO_TIME;    //else delay_to_wait_answer = 10;	//delay_to_wait_answer = 10;  //50;

           NeedAnswer=0;	//FIXME


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
           //delay_measure_tx_middle = delay_measure_tx;
           delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //delay_to_wait_answer = delay_measure_tx*4;  //50;


         RX_Mode();

         }
#endif

       }
}