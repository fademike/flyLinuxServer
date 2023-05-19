/*
 * ModemControl.c
 *
 *  Created on: 06 ���. 2020 �.
 *      Author: NASA
 */

#include <string.h>

#include "main.h"

#include "ModemControl.h"


//#define PACKET_DATALEN 0


#ifdef SI4463
#include "si4463.h"
#include "radio_config_Si4463.h"

#define PACKET_LEN 64
#define PACKET_LEN_CRC 2	      // package Length
#define PACKET_DATALEN 60
//#define CMD_BUF_SIZE 64     // max buffer nRF32L01
#define CMD_LIST_SIZE 5    // any num RAM
#endif


#ifdef nRF24

#include "nRF24.h"
#define PACKET_DATALEN 28

#define PACKET_LEN 32	      // package Length
#define PACKET_LEN_CRC 2	      // package Length
#define PACKET_DATALEN 28   // the size of the data in the package
#define CMD_LIST_SIZE 10    // number of packages


#endif


#pragma pack(push, 1)
struct nrf_pack {
  unsigned char type;
  unsigned char len;
  unsigned char data[PACKET_DATALEN];
  unsigned char crc[2];
};
#pragma pack(pop)

#define DataEx(a) (a##_RD != a##_WR)
#define DataArrayPP(a) if (++a>=CMD_LIST_SIZE) a=0
#define DataArrayMM(a) if (--a<0) a=(CMD_LIST_SIZE-1)


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

////////////////
//circular buffer for receive commands
//#define READCMDSTREAM_ARRAY 10

unsigned char ReadCommandStreamArray[READCMDSTREAM_ARRAY][100];
unsigned char ReadCommandStreamArrayCnt[READCMDSTREAM_ARRAY];
int ReadCommandStreamArray_rd=0;
int ReadCommandStreamArray_wr=0;

#if SYNCHRO_MODE

//  Variables of modem
//volatile int time_delay=0;
//volatile int time_last_tx=0;
volatile int delay_to_wait_answer=0;
volatile int delay_measure_tx=0;
volatile int delay_synchro=0;
char NeedAnswer=0;
//int delay_measure_tx_middle=0;
#endif 

//DELETE									//circular buffer for send out by RF
//#define UART_BUF_SIZE 255			// buffer lenght
char UartRxBuffer[UART_BUF_SIZE];	// buffer
volatile int UartRxBuffer_rd = 0;	// read array
volatile int UartRxBuffer_wr = 0;	//	write array


//#define BUFFER_TX_RF 64
char BufferTxRf[UART_BUF_SIZE];	// buffer
volatile int BufferTxRf_len = 0;	// read array
char BufferTxTextRf[UART_BUF_SIZE];	// buffer
volatile int BufferTxTextRf_len = 0;	// read array



//unsigned char rbuff[PACKET_LEN],  wbuff[PACKET_LEN];	// in/out buffer of modem



#define MODEM_PINCONTROL 0	// For debug pin output



// int ModemControl_init(void){

// #ifdef SI4463

// 	    RFinit();			//Set Settings Config to Si4463
// 	    //setFrequency(433.3*1000*1000);
// 	    setPower(0x20);//(0x08);	//(0x7F);

// 	    SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR

// 	    changeState(STATE_RX);			// Set state to RX



// 	#if 0	// carrier mode
// 			  setCarrier(1);
// 			  char wbuff[64];
// 			  RFwrite(wbuff, PACKET_LEN);

// 			  while(1){RFwrite(wbuff, PACKET_LEN);};
// 	#endif

// #endif

// #ifdef nRF24


//     CSN(GPIO_PIN_RESET);
//     CE(GPIO_PIN_SET);

//     HAL_Delay(20);


// 	int ret = -1;

// 	while(ret < 0){
// 		ret = nRF24_init();
// 		if (ret <0) Printf("nRF_init false\n\r");
// 		HAL_Delay(1000);
// 	}

// 	Printf("nRF_init OK\n\r");


// 	CE(0);
// 	HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);
// 	unsigned char fifo_status=SPI_Read_Reg(FIFO_STATUS);
// 	Printf("Fifo status is: 0x%x\n\r", fifo_status);
// 	fifo_status=SPI_Read_Reg(STATUS);
// 	Printf("status is: 0x%x\n\r", fifo_status);


// 	  unsigned char RX_BUF[TX_PLOAD_WIDTH];

// 	SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
// 	SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
// 	SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

// 	CE(1);

// #endif

// return 0;
// }



#ifdef STM32


__weak void ModemControl_ReadSymbolRfomRF_Callback(char data)
{
}

__weak void ModemControl_ReadMSGRfomRF_Callback(char * data)
{
}

#else 
__attribute__((weak)) void ModemControl_ReadSymbolRfomRF_Callback(char data)
{
}

__attribute__((weak)) void ModemControl_ReadMSGRfomRF_Callback(char * data)
{
}
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

//void ModemControl_SendPacket(char * buff)
//{
//#ifdef SI4463
//	memcpy(BufferTxTextRf, buff, 64);
//	BufferTxTextRf_len = 64;
//#endif
//#ifdef nRF24
//		memcpy(toSendCmdbuf[toSendCmd_WR], buff, TX_PLOAD_WIDTH);
//		DataArrayPP(toSendCmd_WR);
//#endif
//}
//
//#define BUFF_TO_TX 10
//char BuffToTx[BUFF_TO_TX][64];
//int BuffToTx_len[BUFF_TO_TX];
//volatile int BuffToTx_ptr_Wr = 0;
//volatile int BuffToTx_ptr_Rd = 0;
//
//
//void ModemControl_SendSymbol(char buff)
//{
//	static int ptr = 0;
//	BuffToTx[BuffToTx_ptr_Wr][ptr++] = buff;
//	if (ptr>=60){
//		BuffToTx_len[BuffToTx_ptr_Wr]=60;
//		ptr=0;
//
//		int ptrWr = BuffToTx_ptr_Wr;
//		if (++ptrWr >= BUFF_TO_TX) ptrWr = 0;
//		while (ptrWr == BuffToTx_ptr_Rd){
//			//ModemControl_Work();
//		}
//
//
//		BuffToTx_ptr_Wr = ptrWr;//while(BufferTxTextRf_len!=0){ModemControl_Work();}
//	}
//
//	//if ((ptr&0x3) == 0)ModemControl_Work();
//
//
////	static int ptr = 0;
////	BufferTxTextRf[ptr++] = buff;
////	if (ptr>=60){
////		ptr=0;
////		BufferTxTextRf_len = 60;
////		while(BufferTxTextRf_len!=0){}//ModemControl_Work();}
////	}
//}

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


#ifdef SI4463

	    ModemControl_status = RFinit();			//Set Settings Config to Si4463
	    ModemControl_status = RFinit();			//Set Settings Config to Si4463
	    ModemControl_status = RFinit();			//Set Settings Config to Si4463
      if (ModemControl_status < 0) return -1;
	    //setFrequency(433.3*1000*1000);
	    setPower(0x20);//(0x08);	//(0x7F);

	    SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR

	    changeState(STATE_RX);			// Set state to RX



	#if 0	// carrier mode
			  setCarrier(1);
			  char wbuff[64];
			  RFwrite(wbuff, PACKET_LEN);

			  while(1){RFwrite(wbuff, PACKET_LEN);};
	#endif

#endif

#ifdef nRF24


	ModemControl_status = nRF24_init();
	if (ModemControl_status < 0) printf("nRF_init false\n\r");
	else printf("nRF_init OK\n\r");
#endif
	return ModemControl_status;
}

int ptr[10];
int ModemControl_Read(void){

#ifdef SI4463

    uint8_t buffer[6];
    SI446X_INT_STATUS(buffer);
    uint8_t xStatus = buffer[3];

    unsigned char rbuff[PACKET_LEN];

#if 0		//Add Carrier receive handle to future
    if ((xStatus&(1<<4))==0){	// GET carrier!?!? defect! need to clear bit.
    	if ((buffer[6]&0x3) != 0) { return 1;}//thread_ModemControl.t_counter = 0; return;}
    }
#endif
     if ((xStatus&(1<<4))!=0){	 // if there is new package

       //if ((xStatus&(1<<3))!=0){GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);Delay_ms(1);GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);changeState(STATE_RX); continue;}
    	 //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);	//HAL_GPIO_WritePin(PIN_LED, PIN_TEST1_Pin, GPIO_PIN_SET);
             RFread(rbuff, PACKET_LEN);
             //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);	//HAL_GPIO_WritePin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin, GPIO_PIN_RESET);
             SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR


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

               //NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
               char cnt = rbuff[1]&0x3F;
               if (((rbuff[1]>>6)&(0x3)) == 1) {cnt=0; ModemControl_ReadMSGRfomRF_Callback(rbuff);}//{dataCoded[0][0] = cnt; cnt = 0;}
               int i=2;
               if (cnt >=1){
            	   ModemControl_ReadMSGRfomRF_Callback(rbuff);
                 do{
                	 ModemControl_ReadSymbolRfomRF_Callback(rbuff[i++]);
//                	 //HAL_UART_Transmit(&huart1, &rbuff[i], 1, 100);
//                	 ReadCommandStream(rbuff[i++]);

//                	 //i++;
//                   //while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
//                   //UART1_SendData8((uint8_t)rbuff[i++]);
                 }while(--cnt>0);
               }
             }
             changeState(STATE_RX);

            //  //if (MODEM_PINCONTROL)HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);
            //  modem_busy=0;
            //  //thread_ModemControl.t_counter = 0;
            //  //ThreadFly[thread_ModemControl].t_counter=0;
             return 1;//continue;
     }
     return 0;

#endif

#ifdef nRF24
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
  RX_Mode(); //FIXME
  return 0;
#endif
}


void ModemControl_Loop(void)
{

if (ModemControl_Read() != 0) return;

#if 0
     if(0)  // UART Echo
     while(UartRxBuffer_rd!=UartRxBuffer_wr){
         UART1_SendData8((uint8_t)UartRxBuffer[UartRxBuffer_rd]);
         while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
         if (++UartRxBuffer_rd>=UART_BUF_SIZE) UartRxBuffer_rd = 0;
     }
#endif
     //SPI_Read_Reg(CD);//return;
#ifdef nRF24
     //if (SPI_Read_Reg(CD) != 0) {return;}
#endif

     if (1){   //tx data

//    	 BufferTxRf_len=60;
//    	 delay_to_wait_answer=0;
//    	 memcpy(BufferTxRf, "hello from RF  \n\r", strlen("hello from RF  \n\r"));

       if (delay_to_wait_answer != 0){return;               }                          //The timer between the packets being sent. Otherwise, heap data was not accepted.

         unsigned char DataToSend = 0;
         //if(UartRxBuffer_rd!=UartRxBuffer_wr){
//#ifdef SI4463
//         if (BuffToTx_ptr_Wr != BuffToTx_ptr_Rd){//if(BufferTxRf_len>0){
//#endif
//#ifdef nRF24
        	 if (DataEx(toSendCmd)){
//#endif
           //unsigned char int_status[9];
           //SI446X_INT_STATUS(int_status);
           //if (((int_status[5]&0x03)!=0x00) || ((int_status[6]&0x03)!=0x00)) continue;
           //if ((int_status[6]&0x03)!=0x00) continue;


//           while((DataToSend<60) && (UartRxBuffer_rd!=UartRxBuffer_wr)) {wbuff[2+DataToSend++] = UartRxBuffer[UartRxBuffer_rd]; if (++UartRxBuffer_rd>=UART_BUF_SIZE) UartRxBuffer_rd = 0;}

//#ifdef SI4463
//while((DataToSend<60) && (DataToSend<BuffToTx_len[BuffToTx_ptr_Rd]))//(DataToSend<BufferTxRf_len))
//	 //while((DataToSend<60) && (DataToSend<BufferTxRf_len))
//	 {
//		wbuff[2+DataToSend] = BuffToTx[BuffToTx_ptr_Rd][DataToSend];
//		//wbuff[2+DataToSend] = BufferTxRf[DataToSend];
//		DataToSend++;
//	 }
//	 if (++BuffToTx_ptr_Rd>=BUFF_TO_TX) BuffToTx_ptr_Rd=0;
//#endif


//Printf("Data Send...\n\r");

			 //wbuff[1] = (DataToSend)&0x3F;
//			 if (BuffToTx_ptr_Rd == BuffToTx_ptr_Wr)wbuff[0] = PACKET_DATA;
//			 else wbuff[0] = PACKET_DATA_PACK;
			 //wbuff[0] = PACKET_DATA;

           delay_measure_tx = 0;

           //GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);
#ifdef SI4463
           DataToSend=PACKET_DATALEN;

			unsigned char buf[PACKET_LEN];
			buf[0] = PACKET_DATA;
			buf[1] = (DataToSend)&0x3F;
			memcpy(&buf[2], toSendCmdbuf[toSendCmd_RD], PACKET_DATALEN);
			DataArrayPP(toSendCmd_RD);
			//TX_Mode(buf);//NRF24L01_Send(buf);

			//Printf("Tx data... %s\n\r", &toSendCmdbuf[toSendCmd_RD][2]);
			//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
           RFwrite(buf, PACKET_LEN);                                         // TX Data
           //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
#endif
#ifdef nRF24
           DataToSend=PACKET_DATALEN;

           	  		//printf("send\n\r");
           //Printf("Tx data... %s\n\r", &toSendCmdbuf[toSendCmd_RD][2]);


           	  		unsigned char buf[PACKET_LEN];
                 	 buf[0] = PACKET_DATA;
          	  	buf[1] = (DataToSend)&0x3F;
           	  		memcpy(&buf[2], toSendCmdbuf[toSendCmd_RD], PACKET_DATALEN);
           	  		DataArrayPP(toSendCmd_RD);
           	  	// HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
           	  		TX_Mode(buf);//NRF24L01_Send(buf);
           	  	// HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
           		  	//RX_Mode(); // Switch to RX mode
#endif
           //GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);

           //Printf("Data sended %d\n\r", UartRxBuffer_rd/60);

           delay_measure_tx++;

           int delay = 10000;
           //if (BuffToTx_ptr_Rd == BuffToTx_ptr_Wr)
        	   delay=0;

           int t=0;
           for (t=0;t<delay;t++){}		//TODO change to normal view
           //Printf("Data sended %d\n\r", UartRxBuffer_rd/60);

           if (SYNCHRO_MODE) delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //50; //Set timer between send the packets
           delay_synchro = SYNCHRO_TIME;    //else delay_to_wait_answer = 10;	//delay_to_wait_answer = 10;  //50;
#ifdef SI4463
           if (SYNCHRO_MODE) delay_to_wait_answer = delay_measure_tx+6*4;  //50; //Set timer between send the packets
           delay_synchro = SYNCHRO_TIME;    //else delay_to_wait_answer = 10;	//delay_to_wait_answer = 10;  //50;
#endif

//           if (BuffToTx_ptr_Rd != BuffToTx_ptr_Wr){
//        	   if (SYNCHRO_MODE) delay_to_wait_answer = 0;
//           }

//#include <stdlib.h>
           //delay_synchro = 500;//400 + rand()%200;
           BufferTxRf_len=0;

           NeedAnswer=0;	//FIXME

#ifdef nRF24
         RX_Mode();
#endif
         }

#if SYNCHRO_MODE


         if ((DataToSend == 0) && ((NeedAnswer>0) || (delay_synchro>0))){      //Maintain communication by sync pulses.

           //Delay_ms(delay_measure_tx_middle);                                  //So as not to foul the broadcast.
unsigned char wbuff[64];
           if (NeedAnswer) {wbuff[0] = NeedAnswer; NeedAnswer=0;}              // Pulse is Answer
           else if (delay_synchro>0) wbuff[0] = PACKET_ASK;                    // Pulse is Ask
           else wbuff[0] = PACKET_NONE;                                        // Pulse is None
           //wbuff[0] &=0x03;
           delay_measure_tx = 0;
           //GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);
#ifdef SI4463
           //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
           RFwrite(wbuff, PACKET_LEN);                                         // TX Synchro Pulse
           //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
#endif
#ifdef nRF24
  	  		//if (SPI_Read_Reg(CD) != 0) {return -1;}

  	  		//printf("send\n\r");
  	  		//Printf("Tx data...\n\r");
  	  		//unsigned char buf[32];
  	  		//memcpy(buf, toSendCmdbuf[toSendCmd_RD], 32);
  	  		//DataArrayPP(toSendCmd_RD);
          //  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  	  		TX_Mode(wbuff);//NRF24L01_Send(buf);
  	  	// HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  		  	//RX_Mode(); // Switch to RX mode
#endif
           //GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);

           delay_measure_tx++;
           //delay_measure_tx_middle = delay_measure_tx;
           delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //delay_to_wait_answer = delay_measure_tx*4;  //50;

#ifdef nRF24
         RX_Mode();
#endif
         }
#endif

       }

}
