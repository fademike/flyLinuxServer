

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <string.h>

#include "spi.h"

#define SPI_BITRATE 1000000

static uint8_t mode = SPI_MODE_0;//SPI_MODE_3;
static uint8_t bits = 0;
static uint32_t speed = SPI_BITRATE; // for nrf - 0..8Mhz

void pabort(char *s){
    printf("err %s\n\r", s);
    exit(1);
}

#define CS_INVERCE 1
int f_cs;
int f_spi;
int spi_cs(int pos){//return 0;	// write "1" to csX means LOW LEVEL csX. write "0" into csX means Hight level cs
	char res[] = {'0','1'}; 
	int l_cs = pos;
	if ((CS_INVERCE) && (l_cs))l_cs = 0;
	else if ((CS_INVERCE) && (l_cs == 0))l_cs = 1;

	write(f_cs, &res[l_cs],1);

	return 0;
}

int spi_readCmd(char cmd, char * buf, int len)
{
	struct spi_ioc_transfer	xfer[2];
	//unsigned char		buf[32], *bp;
	int			status;

	memset(xfer, 0, sizeof xfer);

	xfer[0].tx_buf = (unsigned long)&cmd;
	xfer[0].len = 1;
	xfer[0].delay_usecs = 100;
	//xfer[0].word_delay_usecs = 0;

	xfer[1].rx_buf = (unsigned long) buf;
	xfer[1].len = len;
	xfer[1].delay_usecs = 100;
	//xfer[1].word_delay_usecs = 0;

	

	status = ioctl(f_spi, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		return -1;
	}
}


int spi_writeCmd(char cmd, char * buf, int len)
{
	struct spi_ioc_transfer	xfer[2];
	//unsigned char		buf[32], *bp;
	int			status;

	memset(xfer, 0, sizeof xfer);
	
	xfer[0].tx_buf = (unsigned long)&cmd;
	xfer[0].len = 1;
	xfer[0].delay_usecs = 100;
	//xfer[0].word_delay_usecs = 100;

	xfer[1].tx_buf = (unsigned long) buf;
	xfer[1].len = len;
	xfer[1].delay_usecs = 100;
	//xfer[1].word_delay_usecs = 100;

	

	status = ioctl(f_spi, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		return -1;
	}
}

int spi_txrx(char * buf_tx, char * buf_rx, int num){
    int ret;
	/* full-duplex transfer */
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)buf_tx,
		.rx_buf = (unsigned long)buf_rx,
		.len = num,
		.delay_usecs = 0,//100,
		//.word_delay_usecs = 100,
		.speed_hz = SPI_BITRATE,//0,
		.bits_per_word = bits,
		//.cs_change = 1,	//FIXME
	};
	
	ret = ioctl(f_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
}


int spi_init(void){

	int ret = 0;
	// uint8_t tx[] = { 0x81, 0x18 };
	// uint8_t rx[] = {0, 0 };

    
	f_cs = open("/root/settings/nrf_csn", O_RDWR);
	if (f_cs < 0) pabort("can't open device");

	spi_cs(0);

	//f_spi = open(argv[1], O_RDWR);
	f_spi = open("/dev/spidev1.0", O_RDWR);
	if (f_spi < 0)
		pabort("can't open device");

	/* spi mode */
	ret = ioctl(f_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(f_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/* bits per word */
	ret = ioctl(f_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(f_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/* max speed hz */
	ret = ioctl(f_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(f_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	// printf("spi mode: %d\n", mode);
	// printf("bits per word: %d\n", bits);
	// printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	return 0;
}

int spi_deinit(void){
	close(f_cs);
    close(f_spi);
	return 0;
}


