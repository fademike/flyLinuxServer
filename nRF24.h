/*
 * nRF24.h
 *
 *  Created on: 12.04.2017.
 *      Author: fademike
 */

#ifndef INC_NRF24_H_
#define INC_NRF24_H_

#include <stdint.h>


// SPI(nRF24L01) commands
#define READ_REG_NRF24L01    	0x00 				// Define read command to register
#define WRITE_REG_NRF24L01   	0x20 				// Define write command to register
#define RD_RX_PLOAD 			0x61 				// Define RX payload register address
#define WR_TX_PLOAD 			0xA0 				// Define TX payload register address
#define FLUSH_TX    			0xE1 				// Define flush TX register command
#define FLUSH_RX    			0xE2 				// Define flush RX register command
#define REUSE_TX_PL 			0xE3 				// Define reuse TX payload register command
//#define NOP         			0xFF 				// Define No Operation, might be used to read status register
//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define NRF24_CONFIG      			0x00				// 'Config' register address
#define EN_AA       			0x01                		// 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   			0x02                		// 'Enabled RX addresses' register address
#define SETUP_AW    			0x03                		// 'Setup address width' register address
#define SETUP_RETR  			0x04                		// 'Setup Auto. Retrans' register address
#define RF_CH       			0x05                		// 'RF channel' register address
#define RF_SETUP    			0x06 				// 'RF setup' register address
#define STATUS      			0x07 				// 'Status' register address
#define OBSERVE_TX  			0x08 				// 'Observe TX' register address
#define CD          			0x09 				//'Carrier Detect' register address
#define RX_ADDR_P0  			0x0A				// 'RX address pipe0' register address
#define RX_ADDR_P1  			0x0B 				// 'RX address pipe1' register address
#define RX_ADDR_P2  			0x0C 				// 'RX address pipe2' register address
#define RX_ADDR_P3  			0x0D 				// 'RX address pipe3' register address
#define RX_ADDR_P4  			0x0E 				// 'RX address pipe4' register address
#define RX_ADDR_P5  			0x0F				// 'RX address pipe5' register address
#define TX_ADDR     			0x10 				// 'TX address' register address
#define RX_PW_P0    			0x11 				// 'RX payload width, pipe0' register address
#define RX_PW_P1    			0x12 				// 'RX payload width, pipe1' register address
#define RX_PW_P2    			0x13 				// 'RX payload width, pipe2' register address
#define RX_PW_P3    			0x14 				// 'RX payload width, pipe3' register address
#define RX_PW_P4    			0x15 				// 'RX payload width, pipe4' register address
#define RX_PW_P5    			0x16 				// 'RX payload width, pipe5' register address
#define FIFO_STATUS 			0x17 			    	// 'FIFO Status Register' register address

#define TX_ADR_WIDTH   	5
#define TX_PLOAD_WIDTH 	32



#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

#define CE(x) spi_ce(x);//HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, x)	//HAL_GPIO_WritePin(GPIOB, CE_Pin, x)
#define CSN(x) spi_cs(x);// HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, x)
// #define IRQ HAL_GPIO_ReadPin(nRF_IRQ_GPIO_Port, nRF_IRQ_Pin)



unsigned char SPI_Receive_byte(unsigned char reg);
unsigned char SPI_Send_byte(unsigned char reg);
unsigned char SPI_Write_Buf(unsigned char  reg, unsigned char  *pBuf, unsigned char  bytes);
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);

unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_Read_Reg(unsigned char reg);
void TX_Mode(unsigned char * tx_buf);
void RX_Mode(void);


#endif /* INC_NRF24_H_ */
