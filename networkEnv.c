


#include <stdio.h>
#include <wiringPi.h>


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <pthread.h>


extern unsigned char PacketTrueRx[2000];
extern unsigned char PacketTrueTx[2000];



#define DEBUG 0

#define PORT 2000

//#define FALSE 0
//#define TRUE 1



int numth = 0;

void* threadTCPmain(void* thread_data){

//void threadTCPmain(int thread_data){


    char client_message[2000];
    //int client_sock = thread_data;//*(int *)thread_data;//client_sock_thread;
    int client_sock = *(int *)thread_data;//client_sock_thread;

  if (DEBUG)printf("start connection server %d\n\r", numth++);



  int read_size;
  int bTimedOut;


  while((read_size = recv(client_sock , client_message , 2000 , 0)) > 0 )
      {
          char buff[2000];
          if (read_size>0){ //TCP_CMD_1 = 1;
            if (client_message[0] == '1'){
              sprintf(buff, "data:");//indic_RX_block);
              memcpy(&buff[strlen("data:")+10], PacketTrueTx, 100); //10 byte - shift
              memcpy(&buff[strlen("data:")+100], PacketTrueRx, 100); //100 byte - shift
              write(client_sock , buff , strlen("data:")+(200));}

          }
          else {if (send(client_sock, NULL, 0,1) >= 0)write(client_sock , "data zero" , strlen("data zero"));}


      }
  if (DEBUG)printf("exit thread connection\n\r");
  close(client_sock);
  pthread_exit(0);
}


void* threadOpenTCPmain(void* thread_data){
	if (DEBUG)printf("start TCP main thread\n\r");

  int socket_desc , client_sock , c , read_size;
  struct sockaddr_in addr , client;
  //char client_message[2000];

  //Create socket
  socket_desc = socket(AF_INET , SOCK_STREAM , 0);	// socket(AF_INET , SOCK_STREAM , IPPROTO_TCP);
  if (socket_desc == -1)
  {
      printf("Could not create socket");
  }
  //puts("Socket created");

  //Prepare the sockaddr_in structure
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons( PORT );

  //Bind
  if( bind(socket_desc,(struct sockaddr *)&addr , sizeof(addr)) < 0)
  {
      //print the error message
	  printf("Port is 2000 ");
      perror("bind failed. Error");
      pthread_exit(0);//return 1;
  }
  //puts("bind done");

  //Listen
  listen(socket_desc , 3);
while(1){
  //Accept and incoming connection
	if (DEBUG)puts("Waiting for incoming connections...");
  c = sizeof(struct sockaddr_in);

  //accept connection from an incoming client
  client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
  if (client_sock < 0)
  {
      perror("accept failed");
      pthread_exit(0);//return 1;
  }
  if (DEBUG)puts("Connection accepted");
#if 0
  //Receive a message from client
  if (0) while( (read_size = recv(client_sock , client_message , 2000 , 0)) > 0 )
  {
      //Send the message back to client
      char buff[2000];
      if (read_size>0){ //TCP_CMD_1 = 1;
        if (client_message[0] == '1'){
        //printf("rx data is: %s\n\r", client_message);
          sprintf(buff, "data is %d.", 123);//indic_RX_block);
          write(client_sock , buff , strlen(buff));}

      }
      else {write(client_sock , "data zero" , strlen("data zero"));}
  }
#endif
	//void* thread_data = NULL;
	pthread_t thread;
	int status = pthread_create(&thread, NULL, threadTCPmain, (void *)&client_sock);//thread_data);
	//threadTCPmain(client_sock);
	if (status != 0) printf("error status=%d\n\r", status);
	pthread_detach(thread);
}
  if(read_size == 0)
  {
      puts("Client disconnected");
      fflush(stdout);
  }
  else if(read_size == -1)
  {
      perror("recv failed");
  }


  pthread_exit(0);
}




  int OpenTCP_Server(void)
{

		pthread_t thread_main;
		pthread_create(&thread_main, NULL, threadOpenTCPmain, NULL);

    return 0;
}
