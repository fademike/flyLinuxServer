


#include <stdio.h>
//#include <wiringPi.h>

/*
    C ECHO client example using sockets
*/
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include <unistd.h>		//close


#include <math.h>

#include <stdint.h>

// The client opens a TCP connection on port 2000.
// And send "1"
// Get a response with data
// converts received data to Jason format

typedef struct
{
    uint8_t startByte; // $
    uint8_t cmd;// = {0x02,0x2C};
    uint8_t len;

    uint8_t indication;	//0-indication off else - on

    uint16_t voltage;
    uint16_t current;

    uint8_t mainMotor;	// 0 - off else - on
    uint8_t mainMotorValue;	//0-255
    uint8_t secondMotor;	// 0 - off else - on
    uint8_t secondMotorValue;	//0-255
    uint8_t servo1;		// 0 - off else - on
    uint8_t servo1Value;	//0-255
    uint8_t servo2;		// 0 - off else - on
    uint8_t servo2Value;	//0-255

    int16_t pitch;		//data Acceleration
    int16_t roll;
    int16_t yaw;


    uint16_t temp1;		// from MPU
    int32_t temp2;		// from BMP
    int32_t press;		// from BMP, in Pa


	uint32_t time;		//GPS Data
	uint32_t day;
	uint32_t longer;
	uint32_t latit;
	uint8_t longer_p;
	uint8_t latit_p;
	uint16_t speed;
	uint16_t hight;
	uint8_t N_sp;
	uint8_t autent;
	uint8_t dir_deg;


    uint8_t crc;
    uint8_t endByte;
//};
}__attribute__((packed)) CMD6PacketStruct ;


typedef struct
{
    unsigned char startByte; // $
    unsigned char cmd;// = {0x02,0x2C};

    unsigned char len;

    unsigned char mainMotor;	// 0 - off else - on
    unsigned char mainMotorValue;	//0-255
    unsigned char secondMotor;	// 0 - off else - on
    unsigned char secondMotorValue;	//0-255
    unsigned char typeManage;		// 0 - simple 1- stabilization
    unsigned char servo1;		// 0 - off else - on
    unsigned char servo1Value;	//0-255
    unsigned char servo2;		// 0 - off else - on
    unsigned char servo2Value;	//0-255
    unsigned char indication;	//0-indication off else - on

    unsigned char StabRoll;
    unsigned char StabPitch;
    unsigned char StabYaw;
    unsigned char StabAngelPitch;
    unsigned char StabAngelRoll;
    unsigned char StabKoeffManage1;	//ROLL
    unsigned char StabKoeffManage2;	//PITCH
    unsigned char StabServo1;		// if 1 - inverse 0 - norm
    unsigned char StabServo2;		// if 1 - inverse 0 - norm

    unsigned char StabKoeffCh1;		//ROLL
    unsigned char StabKoeffCh2;		//PITCH

    unsigned char pos_rev;
    unsigned char pos_ang1;		//0x3;//(0x02;pitch reverse)//
    unsigned char pos_ang2;		//0x13;
    unsigned char StabServoMax1;		// max Value servo 1
    unsigned char StabServoMax2;		// max Value servo 2

    unsigned char CmdTimeout;

    unsigned char math_K_angle;
    unsigned char math_K_bias;
    unsigned char math_K_measure;
    unsigned char math_gyroRate;


    unsigned char crc;
    unsigned char endByte;
}CMD18PacketStruct;


char zerroData[2000] = {0, };

void returnData(char * cmdptr){ //converts received data to Jason format
	char jsonData[1024*10];

    //Tx CMD pointer
	CMD18PacketStruct * CMD18ptr;// = (CMD18PacketStruct *)&cmdptr[strlen("data:")+10];
    //Rx CMD pointer
	CMD6PacketStruct * CMD6ptr;// = (CMD6PacketStruct *)&cmdptr[strlen("data:")+100];

	if (cmdptr == NULL)
	{
		CMD6ptr = (CMD6PacketStruct *)&zerroData[0];
		CMD6ptr->indication = -1;
		CMD18ptr = (CMD18PacketStruct *)&zerroData[100];
		CMD18ptr->indication = -1;
	}
	else {

		CMD18ptr = (CMD18PacketStruct *)&cmdptr[strlen("data:")+10];
		CMD6ptr = (CMD6PacketStruct *)&cmdptr[strlen("data:")+100];

	}

	//sprintf(jsonData, "{");
	sprintf(jsonData, "{\"status\":\"success\",");

#define mySprintfRx(a) sprintf(jsonData, "%s\""#a"""\":\"%d\",", jsonData, CMD6ptr->a)

	sprintf(jsonData, "%s\"voltage\":\"%d\",", jsonData, CMD6ptr->voltage);
	sprintf(jsonData, "%s\"current\":\"%d\",", jsonData, CMD6ptr->current);

	sprintf(jsonData, "%s\"indication\":\"%d\",", jsonData, CMD6ptr->indication);
	sprintf(jsonData, "%s\"mainMotor\":\"%d\",", jsonData, CMD6ptr->mainMotor);
	sprintf(jsonData, "%s\"mainMotorValue\":\"%d\",", jsonData, CMD6ptr->mainMotorValue);
	sprintf(jsonData, "%s\"secondMotor\":\"%d\",", jsonData, CMD6ptr->mainMotor);
	sprintf(jsonData, "%s\"secondMotorValue\":\"%d\",", jsonData, CMD6ptr->mainMotorValue);
	sprintf(jsonData, "%s\"servo1\":\"%d\",", jsonData, CMD6ptr->servo1);
	sprintf(jsonData, "%s\"servo1Value\":\"%d\",", jsonData, CMD6ptr->servo1Value);
	sprintf(jsonData, "%s\"servo2\":\"%d\",", jsonData, CMD6ptr->servo2);
	sprintf(jsonData, "%s\"servo2Value\":\"%d\",", jsonData, CMD6ptr->servo2Value);

	sprintf(jsonData, "%s\"pitch\":\"%d\",", jsonData, CMD6ptr->pitch);
	sprintf(jsonData, "%s\"roll\":\"%d\",", jsonData, CMD6ptr->roll);
	sprintf(jsonData, "%s\"yaw\":\"%d\",", jsonData, CMD6ptr->yaw);

mySprintfRx(temp1);
mySprintfRx(temp2);
mySprintfRx(press);

	sprintf(jsonData, "%s\"time\":\"%d\",", jsonData, CMD6ptr->time);
	sprintf(jsonData, "%s\"day\":\"%d\",", jsonData, CMD6ptr->day);

	sprintf(jsonData, "%s\"longer\":\"%d\",", jsonData, CMD6ptr->longer);
	sprintf(jsonData, "%s\"latit\":\"%d\",", jsonData, CMD6ptr->latit);

	sprintf(jsonData, "%s\"longer_p\":\"%d\",", jsonData, CMD6ptr->longer_p);
	sprintf(jsonData, "%s\"latit_p\":\"%d\",", jsonData, CMD6ptr->latit_p);

	sprintf(jsonData, "%s\"speed\":\"%d\",", jsonData, CMD6ptr->speed);
	sprintf(jsonData, "%s\"hight\":\"%d\",", jsonData, CMD6ptr->hight);
	
	sprintf(jsonData, "%s\"N_sp\":\"%d\",", jsonData, CMD6ptr->N_sp);
	sprintf(jsonData, "%s\"autent\":\"%d\",", jsonData, CMD6ptr->autent);
	sprintf(jsonData, "%s\"dir_deg\":\"%d\",", jsonData, CMD6ptr->dir_deg);


#define mySprintfTx(a) sprintf(jsonData, "%s\"tx_"#a"""\":\"%d\",", jsonData, CMD18ptr->a)

	mySprintfTx(mainMotor);
	mySprintfTx(mainMotorValue);
	mySprintfTx(secondMotor);
	mySprintfTx(secondMotorValue);
	mySprintfTx(typeManage);
	mySprintfTx(servo1);
	mySprintfTx(servo1Value);
	mySprintfTx(servo2);
	mySprintfTx(servo2Value);
	mySprintfTx(indication);
	mySprintfTx(StabRoll);
	mySprintfTx(StabPitch);
	mySprintfTx(StabYaw);
	mySprintfTx(StabAngelPitch);
	mySprintfTx(StabAngelRoll);
	mySprintfTx(StabKoeffManage1);
	mySprintfTx(StabKoeffManage2);
	mySprintfTx(StabKoeffCh1);
	mySprintfTx(StabKoeffCh2);
	mySprintfTx(pos_rev);
	mySprintfTx(pos_ang1);
	mySprintfTx(pos_ang2);
	mySprintfTx(StabServoMax1);
	mySprintfTx(StabServoMax2);
	mySprintfTx(CmdTimeout);
	mySprintfTx(math_K_angle);
	mySprintfTx(math_K_bias);
	mySprintfTx(math_K_measure);
	mySprintfTx(math_gyroRate);

	sprintf(jsonData, "%s\"indx.x\":\"0\"}", jsonData);
	printf("%s", jsonData);

}


#define DEBUG 0

    char message[1000] , server_reply[2000];

int main(int argc , char *argv[])
{
    int sock;
    struct sockaddr_in server;

    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1)
    {
        returnData(NULL);//printf("Could not create socket");
        return 1;
    }
    if (DEBUG)puts("Socket created");

    server.sin_addr.s_addr = inet_addr("127.0.0.1");//("192.168.100.1");//("192.168.96.20");//("127.0.0.1");
    server.sin_family = AF_INET;
    server.sin_port = htons( 2000 );

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        returnData(NULL);//perror("connect failed. Error");
        return 1;
    }

    if (DEBUG)puts("Connected...");

        message[0] = '1';
        message[1] = 0;

        //Send some data
        //if( send(sock , message , strlen(message) , 0) < 0)
        if( send(sock , message , 1 , 0) < 0)
        {
            returnData(NULL);//puts("Send failed");
            return 1;
        }
        if (DEBUG)puts("Sended...");
        //Receive a reply from the server
        if( recv(sock , server_reply , 2000 , 0) < 0)
        {
            returnData(NULL);///puts("recv failed");
			
            return 1;
        }

        char tempStr[2000] = {0,};
        memcpy(tempStr, server_reply, strlen("data:")+1);



        if (strcmp(tempStr, "data:")==0){
        	if (DEBUG)printf("Rx string...\n\r");
			returnData(server_reply);
        }
		else {returnData(NULL);}

    close(sock);

    return 0;
}




