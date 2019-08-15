

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <sys/socket.h>

#include <pthread.h>

#include "networkEnv.h"

#include "main.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

// SETTINGS

#define ROLL_INVERSION 0
#define ROLL_RANGEVALUE 127	//MAX 127
#define PITCH_INVERSION 0
#define PITCH_RANGEVALUE 127	//MAX 127



#define TIMER_WITHOUTCMD 20 // TIMER_WITHOUTCMD*100 = time of ms!

//Stabilization

#define CONTROL_ROLL_INV 1
#define CONTROL_PITCH_INV 1

#define MAXVALUE_STAB_SERVO1 127
#define MAXVALUE_STAB_SERVO2 127

#define MAXVALUE_STAB_MANAGE1 127//127
#define MAXVALUE_STAB_MANAGE2 127//127

#define MAXVALUE_STAB_CH1 127
#define MAXVALUE_STAB_CH2 127

//Stabilization end


// END SETTINGS


#define JOYSTICK_DEVNAME "/dev/input/js0"

#define JS_EVENT_BUTTON 0x01 
#define JS_EVENT_AXIS 0x02 
#define JS_EVENT_INIT 0x80

int btn_state[24];
int axis_state[24];
int MainMotorValue = 100;


    int TX_mainMotor = 0;	// 0 - off else - on
    int TX_mainMotorValue = 0;	//0-255
    int TX_servo1 = 1;		// 0 - off else - on
    int TX_servo1Value = 0;	//0-255
    int TX_servo2 = 1;		// 0 - off else - on
    int TX_servo2Value = 0;	//0-255
    int TX_indication = 0;	//0-indication off else - on

    int TX_secondMotor = 0;	// 0 - off else - on
    int TX_secondMotorValue = 0;	//0-255


    unsigned char TX_typeManage = 1;		// 0 - manual 1- stabilization

    unsigned char TX_StabRoll = 0;
    unsigned char TX_StabPitch = 0;
    unsigned char TX_StabYaw = 0;
    unsigned char TX_StabAngelPitch = 0;
    unsigned char TX_StabAngelRoll = 0;
    unsigned char TX_StabKoeffManage1 = MAXVALUE_STAB_MANAGE1;//TX_StabMaxValue1 = ROLL_RANGEVALUE;
    unsigned char TX_StabKoeffManage2 = MAXVALUE_STAB_MANAGE2;//TX_StabMaxValue2 = PITCH_RANGEVALUE;
    unsigned char TX_StabServo1 = 0;		// 0bit - rev Pitch; 1bit - rev Roll
    unsigned char TX_StabServo2 = 0;		// 0bit - rev Pitch; 1bit - rev Roll
    unsigned char TX_StabKoeffCh1 = MAXVALUE_STAB_CH1;
    unsigned char TX_StabKoeffCh2 = MAXVALUE_STAB_CH2;

    unsigned char TX_pos_rev = 0;	//Pitch <==> Roll //1 or 0
    unsigned char TX_pos_ang1 = 0x3;	//(0x02;pitch reverse)//
    unsigned char TX_pos_ang2 = 0x13;

    unsigned char TX_StabServoMax1 = MAXVALUE_STAB_SERVO1;	// max value for servo1
    unsigned char TX_StabServoMax2 = MAXVALUE_STAB_SERVO2;	// max value for servo2

    unsigned char Tx_CmdTimeout = TIMER_WITHOUTCMD;

    unsigned char Tx_math_K_angle = 1;
    unsigned char Tx_math_K_bias = 3;
    unsigned char Tx_math_K_measure = 30;
    unsigned char Tx_math_gyroRate = 131;


#define BOARD_BananaPi 0
#define BOARD_RaspberryPi3 1


#define BOARD BOARD_RaspberryPi3

#if (BOARD == BOARD_RaspberryPi3)
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

//for read open write file
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif


int uart0_filestream = -1;




struct js_event {	//joystick event
    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;
};

static int joystick_fd = -1;

int open_joystick()
{
    joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK); 
    //  /dev/input/js0
    return joystick_fd;
}

int read_joystick_event(struct js_event *jse)
{
    int bytes;
    int size = sizeof(*jse);
    bytes = read(joystick_fd, jse, sizeof(*jse)); 
    if (bytes == -1)
        return 0;
    if (bytes == sizeof(*jse))
        return 1;
    printf("Unexpected bytes from joystick:%d\n", bytes);
    return -1;
}

void close_joystick()
{
    close(joystick_fd);
}


unsigned char PacketTrueRx[2000];		// Buffer for send data by TCP
unsigned char PacketTrueTx[2000];		// Buffer for send data by TCP



int fd_uart;

void* thread_readUart(void* thread_data)
{
	unsigned char PacketRx[255];
	int PacketRxB = 0;

	static int position = 0;
	static int data_len = 0;
	static unsigned char PacketCRC = 0;
	static int cmd = 0;
    unsigned char buffer[255];
	    while(1)
	    {
            int n=read(uart0_filestream,buffer,1);
            if (n!=1){usleep(100); continue;}

            //printf("rx\n\r");

	    	int iTemp = buffer[0];//serialGetchar(fd_uart);
	    	if (iTemp >= 0)
	    	{
	    			unsigned char cTemp = iTemp&0xFF;
	    			//HAL_UART_Transmit(&huart1, &UART_rx_bufer[UART_rx0_index], 1, 100);
	    			//HAL_UART_Receive


	    			PacketCRC ^= cTemp;
    				//printf("read is %d, pos = %d, nPack = %d,\n\r",cTemp, position, PacketRxB);
	    			//printf("rx %d\n\r", position);
	    			if ((position == 0) && (cTemp == '$')) {position++; PacketCRC = 0;}
	    			else if ((position == 1) && ((cTemp == 0x2) || (cTemp == 0x4) || (cTemp == 0x6))) {cmd = cTemp; position++;}
	    			else if (position == 2) {data_len = cTemp; position++; PacketRxB = 3; PacketCRC = 0;}
	    			else if ((position == 3) && (data_len > 0)) {PacketRx[PacketRxB++] = cTemp; data_len--;}
	    			else if ((position == 3) && (data_len <= 0)) {if (PacketCRC == 0)position++;else {position=0; printf("crc err %d, %d\n\r", cTemp, PacketCRC);}} //Check CRC
	    			else if ((position == 4) && (cTemp == '#')){

	    				PacketRx[1] = cmd;

	    				if (cmd == 2){
							CMD2PacketStruct * CMD2ptr = (CMD2PacketStruct *)&PacketRx[0];

							//printf("Rx Data: v:%d, press:%d\n\r", (CMD2ptr->voltage)*(10+2)/2, CMD2ptr->press);
							//printf("Rx Data: motor:%d, motorV:%d, press:%d\n\r", CMD2ptr->mainMotor, CMD2ptr->mainMotorValue, CMD2ptr->press);
							//printf("Rx Data: s1V:%d, s2V:%d, press:%d\n\r", CMD2ptr->servo1Value, CMD2ptr->servo2Value, CMD2ptr->press);
							printf("Rx Data: motor:%d, motorV:%d, ", CMD2ptr->mainMotor, CMD2ptr->mainMotorValue);
							printf("s1V:%d, s2V:%d, press:%d, ", CMD2ptr->servo1Value, CMD2ptr->servo2Value, CMD2ptr->press);
							printf("Motor:%d\n\r", MainMotorValue);

							memcpy(PacketTrueRx, PacketRx, 250);
	    				}
	    				else if (cmd == 4){
							CMD4PacketStruct * CMD4ptr = (CMD4PacketStruct *)&PacketRx[0];

							printf("Rx Data: motor:%d, motorV:%d, ", CMD4ptr->mainMotor, CMD4ptr->mainMotorValue);
							printf("p:%d, roll:%d, yaw:%d, ", CMD4ptr->pitch/100, CMD4ptr->roll/100, CMD4ptr->yaw/100);
							printf("Motor:%d\n\r", MainMotorValue);

							memcpy(PacketTrueRx, PacketRx, 250);
	    				}
	    				else if (cmd == 6){

							CMD6PacketStruct * CMD6ptr = (CMD6PacketStruct *)&PacketRx[0];

							printf("Rx Data: motor:%d, motorV:%d, ", CMD6ptr->mainMotor, CMD6ptr->mainMotorValue);
							//printf("t:%d, d:%d, speed:%d, NSP:%d, ", CMD6ptr->time, CMD6ptr->day, CMD6ptr->speed,  CMD6ptr->N_sp);
							//printf("h:%d, l:%d, l_p:%d, ", CMD6ptr->hight, CMD6ptr->longer, CMD6ptr->longer_p);
							//printf("lat:%d, lat_p:%d, N_sp:%d, ", CMD6ptr->latit, CMD6ptr->latit_p, CMD6ptr->N_sp);
							//printf("aut:%d, deg:%d, l_p:%d, ", CMD6ptr->autent, CMD6ptr->dir_deg, CMD6ptr->longer_p);
							printf("Motor:%d, Angel:%d\n\r", MainMotorValue, *(char *)&TX_StabAngelPitch);



							memcpy(PacketTrueRx, PacketRx, 250);
	    				}

	    				position = 0;
	    			}
	    			else position = 0;
	    			//else

	    	}
	    }
	    serialClose(fd_uart);
	    return 0;
}


int fd_joystick, rc_joystick;
struct js_event jse;

void* thread_readJoystick(void* thread_data)
{

    do {
        fd_joystick = open_joystick();
        if (fd_joystick < 0) {
            printf("open joystick failed.\n");
            sleep(1);
            //exit(1);
        }
    }while(fd_joystick < 0);

    while (1) {
        rc_joystick = read_joystick_event(&jse);
        usleep(1000);
        if (rc_joystick == 1) {
            jse.type &= ~JS_EVENT_INIT;
            if (jse.type == JS_EVENT_AXIS) {
                //printf("Event: time %8u, value %8hd, type: %3u, axis: %u\n",
                //    jse.time, jse.value, jse.type, jse.number);
                axis_state[jse.number] = jse.value;

                if (jse.number == 6){
                	if (jse.value>0){if (MainMotorValue>0)MainMotorValue-=10;}
                	if (jse.value<0){if (MainMotorValue<250)MainMotorValue+=10;}

                }
                if (jse.number == 5){	//angel pitch settings
                	char temp = *(char *)&TX_StabAngelPitch;
                	if (jse.value<0){if ((TX_StabAngelPitch<60) || (TX_StabAngelPitch>=(127+60-5))) temp+=5;}
                	if (jse.value>0){if ((TX_StabAngelPitch<=60) || (TX_StabAngelPitch>(127+60))) temp-=5;}
                	TX_StabAngelPitch = temp;

                }
            }
            if (jse.type == JS_EVENT_BUTTON) {
                //printf("Event: time %8u, value %8hd, type: %3u, button: %u\n",
                //    jse.time, jse.value, jse.type, jse.number);

                //if (jse.number == 6) TX_mainMotor = jse.value;


            	if ((jse.number == 0) && (jse.value != 0))TX_typeManage = 0;
            	if ((jse.number == 1) && (jse.value != 0))TX_typeManage = 1;
                btn_state[jse.number] = jse.value;

            }
        }
    }
}

void CMD18_Fill(unsigned char * data)
{
	CMD18PacketStruct* ptr = (CMD18PacketStruct *)data;
	ptr->startByte = '$';
	ptr->cmd = 0x18;
	ptr->len = sizeof(CMD18PacketStruct)-3-2;//7+2+9;
	ptr->mainMotor = TX_mainMotor;
	ptr->mainMotorValue = TX_mainMotorValue;
	ptr->secondMotor = TX_secondMotor;
	ptr->secondMotorValue = TX_secondMotorValue;
	ptr->servo1 = TX_servo1;
	ptr->servo1Value = *(unsigned char *)(&TX_servo1Value);//cTX_servo1Value;//TX_servo1Value;
	ptr->servo2 = TX_servo2;
	ptr->servo2Value = *(unsigned char *)(&TX_servo2Value);//cTX_servo2Value;//TX_servo2Value;
	ptr->indication = TX_indication;

	ptr->typeManage = TX_typeManage;
	ptr->StabRoll = TX_StabRoll;
	ptr->StabPitch = TX_StabPitch;
	ptr->StabYaw = TX_StabYaw;
	ptr->StabAngelPitch = TX_StabAngelPitch;
	ptr->StabAngelRoll = TX_StabAngelRoll;
	ptr->StabKoeffManage1 = TX_StabKoeffManage1;
	ptr->StabKoeffManage2 = TX_StabKoeffManage2;
	ptr->StabServo1 = TX_StabServo1;
	ptr->StabServo2 = TX_StabServo2;
	ptr->StabKoeffCh1 = TX_StabKoeffCh1;
	ptr->StabKoeffCh2 = TX_StabKoeffCh2;

	ptr->pos_rev = TX_pos_rev;
	ptr->pos_ang1 = TX_pos_ang1;
	ptr->pos_ang2 = TX_pos_ang2;
	ptr->StabServoMax1 = TX_StabServoMax1;
	ptr->StabServoMax2 = TX_StabServoMax2;

	ptr->CmdTimeout = Tx_CmdTimeout;

	ptr->math_K_angle = Tx_math_K_angle;
	ptr->math_K_bias = Tx_math_K_bias;
	ptr->math_K_measure = Tx_math_K_measure;
	ptr->math_gyroRate = Tx_math_gyroRate;


	ptr->crc = 0x00;
	ptr->endByte = '#';

	int i=0;
	unsigned char matchCRC = 0;
	for (i=0;i<(sizeof(CMD18PacketStruct)-3-2);i++) matchCRC ^= data[i+3];
	ptr->crc = matchCRC;
}


void* thread_timerSendUart(void* thread_data)		// Cyclic command sending by UART
{
    while (1) {

    	int TX_servo1ValueTemp;
    	int TX_delta;

    	if (btn_state[6]!=0){	// MOTOR ON
    		TX_mainMotor = 1;	//Enable main motor
    		TX_secondMotor=1;	//Enable second motor
    		if (btn_state[4]!=0) {TX_mainMotorValue=0xFF; TX_secondMotorValue = 0xFF;}	// max value rate
    		else {											// calibrate value by stick
    			//TX_mainMotorValue = MainMotorValue;
    			//char temp = (axis_state[1]*127)/0x7FFF;
    			int tempValue = MainMotorValue - ((axis_state[1]*127)/0x7FFF);

    			int delta = ((axis_state[0]*50)/0x7FFF);
    			TX_mainMotorValue = tempValue + delta;
    			TX_secondMotorValue = tempValue - delta;

    			if (TX_mainMotorValue >255) TX_mainMotorValue = 255;
    			else if  (TX_mainMotorValue <0) TX_mainMotorValue = 0;
    			if (TX_secondMotorValue >255) TX_secondMotorValue = 255;
    			else if  (TX_secondMotorValue <0) TX_secondMotorValue = 0;


    		}
    	}
    	else {TX_mainMotor=0; TX_mainMotorValue = 0; TX_secondMotorValue = 0; TX_secondMotor=1;}	//disable motor
    	TX_servo1=1;
    	//TX_servo1Value
		TX_servo1ValueTemp = (axis_state[4]*PITCH_RANGEVALUE)/0x7FFF;	//was 2 axis
    	TX_servo2=1;
    	//TX_servo2Value = -TX_servo1Value;//
    	TX_delta = (axis_state[3]*ROLL_RANGEVALUE)/0x7FFF;
    	//TX_delta/=2;
#if !ROLL_INVERSION
    	TX_delta =-TX_delta;
#endif
#if PITCH_INVERSION
    	TX_servo1ValueTemp =-TX_servo1ValueTemp;
#endif

    	TX_StabRoll = (axis_state[3]*100)/0x7FFF;
    	TX_StabPitch = (axis_state[4]*100)/0x7FFF;

    	if (CONTROL_ROLL_INV) TX_StabRoll = (-axis_state[3]*100)/0x7FFF;
    	else TX_StabRoll = (axis_state[3]*100)/0x7FFF;

    	if (CONTROL_PITCH_INV) TX_StabPitch = (-axis_state[4]*100)/0x7FFF;
    	else TX_StabPitch = (axis_state[4]*100)/0x7FFF;

    	TX_servo1Value = -TX_servo1ValueTemp + TX_delta;
    	TX_servo2Value = TX_servo1ValueTemp + TX_delta;

    	if (TX_servo1Value<(-127)) TX_servo1Value= -127;
    	if (TX_servo1Value>(127)) TX_servo1Value= 127;

    	if (TX_servo2Value<(-127)) TX_servo2Value= -127;
    	if (TX_servo2Value>(127)) TX_servo2Value= 127;


    	unsigned char DataToTx[255];
    	CMD18_Fill(&DataToTx[0]);		
#define CMD_TO_TX CMD18PacketStruct

int t=0;
for (t=0;t<sizeof(CMD_TO_TX);t++)PacketTrueTx[t] = DataToTx[t];

        if ((BOARD == BOARD_BananaPi)){
        	int i=0;
    	    for (i=0;i<(sizeof(CMD_TO_TX)+10);i++) serialPutchar(fd_uart, DataToTx[i]);
        }
        else if ((BOARD == BOARD_RaspberryPi3)){
            int count = write(uart0_filestream, &DataToTx[0], (sizeof(CMD_TO_TX)+10));		//Filestream, bytes to write, number of bytes to write
		    if (count < 0) printf("UART TX error\n");

		}

        usleep(200000);
    }
}



int COM_PortOpen_RPI(void)
{
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	//uart0_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	uart0_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
        return -1;
	}



	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

    //write(uart0_filestream,"123",3);
    return 0;
}




int main(int argc, char *argv[])
{

    if (BOARD == BOARD_BananaPi) printf("Board is BOARD_BananaPi\n\r");
    else if (BOARD == BOARD_RaspberryPi3) printf("Board is BOARD_RaspberryPi3\n\r");
    else printf("Board is undef\n\r");

    if(wiringPiSetup() < 0){printf("wiringPiSetup error\n\r");return 1;}

    if (BOARD == BOARD_BananaPi) {if((fd_uart = serialOpen("/dev/ttyS2",115200)) < 0){printf("Open tty error\n\r");return 1;}}
    else if (BOARD == BOARD_RaspberryPi3) {if(COM_PortOpen_RPI() < 0){printf("Open tty error\n\r");return 1;}}
    else {printf("Board is undef\n\r"); return 1;}


	void* thread_data = NULL;

	//UART reader
	pthread_t threadUart;
	pthread_create(&threadUart, NULL, thread_readUart, thread_data);

	//Joystick reader
	pthread_t threadJoystick;
	pthread_create(&threadJoystick, NULL, thread_readJoystick, thread_data);

	//timer for send cmd by UART
	pthread_t threadSendUart;
	pthread_create(&threadSendUart, NULL, thread_timerSendUart, thread_data);

	// Opening TCP for read variables
	OpenTCP_Server();

    while (1) {
        usleep(1000);
    }



}

