


#define error_message printf

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

int tty_init(char * portname);
void tty_write(char * buf, int len);
int tty_read(char * buf, int len);

