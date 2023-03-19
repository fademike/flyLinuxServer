



int spi_init(void);
int spi_deinit(void);
int spi_cs(int pos);
int spi_readCmd(char cmd, char * buf, int len);
int spi_writeCmd(char cmd, char * buf, int len);
int spi_txrx(char * buf_tx, char * buf_rx, int num);

