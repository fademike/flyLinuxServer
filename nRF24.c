

#include <stdio.h>

#include "nRF24.h"
#include "spi.h"



volatile int timer_ms;



void SYSTICK_Callback(void)
{
	timer_ms++;
}



#define TX_PLOAD_WIDTH 32

unsigned char TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};
unsigned char RX_BUF[TX_PLOAD_WIDTH];
unsigned char TX_BUF[TX_PLOAD_WIDTH];


#define CE(x) pio_ce(x)  //HAL_GPIO_WritePin(GPIOB, CE_Pin, x)                 // for rx/tx mode
#define CSN(x) spi_cs(x)  //HAL_GPIO_WritePin(GPIOB, CSN_Pin, x)  // for spi
#define IRQ 0//HAL_GPIO_ReadPin(GPIOB, IRQ_Pin)


unsigned char SPI_Receive_byte(unsigned char reg)
{
	//HAL_SPI_TransmitReceive(&hspi1, &reg, &reg, 1, 100);
  char buf [1];
  spi_readCmd(reg, buf, 1);
   return buf[0];
}

unsigned char SPI_Send_byte(unsigned char reg)
{
	//HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
  char buf [1];
	spi_writeCmd(reg, buf, 1);
   return buf[0];
}

unsigned char  SPI_Write_Buf(unsigned char  reg, unsigned char  *pBuf, unsigned char  bytes)
{
	unsigned char  status,byte_ctr,i;
	CSN(0);
	// status= SPI_Receive_byte(reg);
	// //HAL_Delay(10); //delayMicroseconds(10);//delay1us(1);
	// for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
	// 	SPI_Send_byte(*pBuf++);
	
	unsigned char buf_tx[33] = {reg, };
	unsigned char buf_rx[33] = {0,0};
	memcpy(&buf_tx[1], pBuf, bytes);
	spi_txrx(&buf_tx, &buf_rx, bytes+1);
	status = buf_rx[0];
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
	unsigned char status;//,byte_ctr;
	//CSN(0);

	
	unsigned char buf_tx[33] = {reg, };
	unsigned char buf_rx[33] = {0,0};
	spi_txrx(&buf_tx, &buf_rx, bytes+1);
	status = buf_rx[0];

	memcpy(pBuf, &buf_rx[1], bytes);
	
	// status=SPI_Read_Reg(reg);//SPI_Receive_byte(reg);
	// unsigned char * bufer[32];
	// //HAL_SPI_TransmitReceive(&hspi1, bufer, pBuf, bytes, 100);
  	// spi_txrx(bufer, pBuf, bytes);
	//CSN(1);
	return(status);
}

unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
	//unsigned char status;
	//CSN(0);
	//status=SPI_Receive_byte(reg);   //select register  and write value to it
	//SPI_Send_byte(value);
	unsigned char buf_tx[2] = {reg,value};
	unsigned char buf_rx[2] = {0,0};
	spi_txrx(buf_tx, buf_rx, 2);
	//status = buf_rx[0];
	//CSN(1);
	return(buf_rx[1]);
}

unsigned char SPI_Read_Reg(unsigned char reg)
{
	unsigned char status;
	//CSN(0);
	// SPI_Send_byte(reg);
	// status=SPI_Receive_byte(0);   //select register  and write value to it
	spi_txrx(&reg, &status, 1);
	//CSN(1);
	return(status);
}


void TX_Mode(unsigned char * tx_buf)
{
	CE(0);
	CRC_PacketCalculate(tx_buf);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // {0xb2,0xb2,0xb3,0xb4,0x01}
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // wr payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);       // Enable AA to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // Enable rx data to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // number retr to error
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);         // work channel
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // no carrier;  no encoder; 1Mbps; 0dbm
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // num width payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + NRF24_CONFIG, 0x0e);      // Enable CRC; 2byte CRC; Power UP; PTX;
	CE(1);

	while((SPI_RW_Reg(FIFO_STATUS, 0)) == 0x0){}
	
}

void RX_Mode(void)
{
	CE(0);
	SPI_Read_Reg(STATUS);

  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);               // enable ask
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // set address rx
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);                 // channel

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 0dbm 1MBps
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + NRF24_CONFIG, 0x0f);               // Enable CRC; 2byte CRC; Power UP; PRX;
  CE(1);
}


void NRF24L01_Receive(void)
{
    unsigned char status=0x01;

	CE(0);
	//HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);

	// status=SPI_RW_Reg(STATUS, 0);//SPI_Read_Reg(STATUS);
	// status=SPI_RW_Reg(STATUS, 0);//SPI_Read_Reg(STATUS);
	status = SPI_Read_Reg(STATUS);
	status = SPI_Read_Reg(STATUS);

//printf("status %x  ", status);
	if(status & 0x40)
	{
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);
		
		unsigned char fifo_status = 0;
		do{
			SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

			printf("RX --");
			//printf("data zerro is: 0x%x\r\n",RX_BUF[0]);
			int t=0; for (t=0; t<32; t++)printf("0x%x,",RX_BUF[t]); printf("\r\n");

			fifo_status=SPI_RW_Reg(FIFO_STATUS, 0);
			if ((fifo_status&0x1) != 0x1) printf("fifo_status %x ", fifo_status);
		}while((fifo_status&0x1) != 0x1);	//Read all data! (nRF24L01 Can store up to 2 payload data!)
	}
	if(status&TX_DS)
	{
		printf("data sended INRX\r\n");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);
	}
	if(status&MAX_RT)
	{
		printf("NO SEND..INRX\n\r");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);
	}
	printf("status %x  \n\r", status);

	CE(1);

}

void NRF24L01_Send(void)
{
    unsigned char status=0x00;
	// simple payload to send:
	unsigned char bbuuff[32] = {0x33,0x12,0x31,0x54,0x25,0x16,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

	TX_Mode(bbuuff);

	while(IRQ == 1);
}


#include <fcntl.h>

int f_ce;

int ce_init(){
	f_ce = open("/root/settings/nrf_ce", O_RDWR);
	if (f_ce < 0) pabort("can't open device");
}

#define CE_INVERCE 1
int pio_ce(int pos){//return 0;	// write "1" to csX means LOW LEVEL csX. write "0" into csX means Hight level cs
	char res[] = {'0','1'}; 
	
  if (CE_INVERCE) write(f_ce, &res[(~pos)&0x1],1);
	else write(f_ce, &res[pos],1);

	return 0;
}


int nRF24_init(void){
	ce_init();
	spi_init();

	CSN(0);
	CE(1);


	CE(0);

	unsigned char config =  0x0F, st1, st2;		//0x0f;


	st1 = SPI_RW_Reg(WRITE_REG_NRF24L01 + NRF24_CONFIG, config);               // Enable CRC; 2byte CRC; Power UP; PRX;
	st2 = SPI_RW_Reg(NRF24_CONFIG, 0);
	//printf("st1=%d, st2=%d\n\r", st1, st2);
  	if (st2 != config) {CE(1); return -1;}

  	RX_Mode();
  	CE(1);
  	return 0;
}


