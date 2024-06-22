


// int mavlink_setTime(int t_us);
// int mavlink_loop_tx(int sock, struct  sockaddr_in * gcAddr);
// int mavlink_loop_rx(char s);//(int sock, struct  sockaddr_in * gcAddr, socklen_t * fromlen);



void mavlink_receive(char rxdata);

void mavlink_send_heartbeat_server(void);	// package for transit system
int mavlink_send_reboot_system(char * buf);
void mavlink_send_status(void);
void mavlink_send_attitude(void);
void mavlink_send_battery_status(void);
void mavlink_send_time(void);
void mavlink_send_heartbeat_server(void);

//void mavlink_send_msg(mavlink_message_t * msg);