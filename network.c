#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>

/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */



#include "network.h"
#include "mavlink_handler.h"
#include "ModemControl.h"
#include "nRF24.h"


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

int sock;
struct sockaddr_in gcAddr; 
struct sockaddr_in locAddr;

socklen_t fromlen;

int network_init(void){
	
	char target_ip[100];
	
	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	strcpy(target_ip, "127.0.0.1");
	
	
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);
	
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    } 
	
	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)

    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }
	
	
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);
	
    fromlen = sizeof(&gcAddr);
	
	// struct in_addr paddr;
	// paddr.s_addr = gcAddr.sin_addr.s_addr;
	// printf("addr %s\n\r", inet_ntoa(paddr));

	printf("net init ok\n\r");
    return 0;
}



int network_receive(char * buf, int len){
    ssize_t recsize;

    memset(buf, 0, BUFFER_LENGTH);
    recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

	return recsize;
}

int network_send(char * buf, int len)
{
	//int bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
    int bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, fromlen);
    return bytes_sent;
}