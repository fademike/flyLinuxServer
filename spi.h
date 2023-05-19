



int spi_init(void);
int spi_deinit(void);
int spi_cs(int pos);
int spi_ce(int pos);
int spi_readCmd(unsigned char cmd, unsigned char * buf, int len);
int spi_writeCmd(unsigned char cmd, unsigned char * buf, int len);
int spi_txrx(unsigned char * buf_tx, unsigned char * buf_rx, int num);

