






#include <stdint.h>
typedef struct
{
    uint8_t startByte; // $
    uint8_t cmd;// = {0x02,0x2C};
    uint8_t len;

    uint16_t voltage;
    uint16_t current;

    uint8_t mainMotor;	// 0 - off else - on
    uint8_t mainMotorValue;	//0-255
    uint8_t servo1;		// 0 - off else - on
    uint8_t servo1Value;	//0-255
    uint8_t servo2;		// 0 - off else - on
    uint8_t servo2Value;	//0-255
    uint8_t indication;	//0-indication off else - on

    int16_t dofAx;		//data Acceleration
    int16_t dofAy;
    int16_t dofAz;
    int16_t dofGx;		//data Gyroscope
    int16_t dofGy;
    int16_t dofGz;
    int16_t dofMx;		//data Magnitude
    int16_t dofMy;
    int16_t dofMz;

    uint16_t temp1;		// from MPU
    int32_t temp2;		// from BMP
    int32_t press;		// from BMP, in Pa

    uint8_t crc;
    uint8_t endByte;

    //pos_struct qwe;
}CMD2PacketStruct;


typedef struct
{
    uint8_t startByte; // $
    uint8_t cmd;// = {0x02,0x2C};
    uint8_t len;

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
    uint8_t indication;	//0-indication off else - on

    int16_t pitch;		//data Acceleration
    int16_t roll;
    int16_t yaw;

    int16_t dofAx;		//data Acceleration
    int16_t dofAy;
    int16_t dofAz;
    int16_t dofGx;		//data Gyroscope
    int16_t dofGy;
    int16_t dofGz;
    int16_t dofMx;		//data Magnitude
    int16_t dofMy;
    int16_t dofMz;

    uint16_t temp1;		// from MPU
    int32_t temp2;		// from BMP
    int32_t press;		// from BMP, in Pa

    uint8_t crc;
    uint8_t endByte;

    //pos_struct qwe;
}CMD4PacketStruct;


////#pragma pack(push, 1)
typedef struct

//struct __attribute__ ((__packed__)) CMD6PacketStruct
//struct __attribute__((__aligned__(1))) CMD6PacketStruct


//struct CMD6PacketStruct


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
//#pragma pack(pop)

typedef struct
{
    unsigned char startByte; // $
    unsigned char cmd;// = {0x02,0x2C};

    unsigned char len;

    unsigned char mainMotor;	// 0 - off else - on
    unsigned char mainMotorValue;	//0-255
    unsigned char servo1;		// 0 - off else - on
    unsigned char servo1Value;	//0-255
    unsigned char servo2;		// 0 - off else - on
    unsigned char servo2Value;	//0-255
    unsigned char indication;	//0-indication off else - on

    unsigned char crc;
    unsigned char endByte;
}CMD12PacketStruct;



typedef struct
{
    unsigned char startByte; // $
    unsigned char cmd;// = {0x02,0x2C};

    unsigned char len;

    unsigned char mainMotor;	// 0 - off else - on
    unsigned char mainMotorValue;	//0-255
    unsigned char secondMotor;	// 0 - off else - on
    unsigned char secondMotorValue;	//0-255
    unsigned char servo1;		// 0 - off else - on
    unsigned char servo1Value;	//0-255
    unsigned char servo2;		// 0 - off else - on
    unsigned char servo2Value;	//0-255
    unsigned char indication;	//0-indication off else - on

    unsigned char crc;
    unsigned char endByte;
}CMD14PacketStruct;


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

    unsigned char crc;
    unsigned char endByte;
}CMD16PacketStruct;



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



typedef struct
{
    signed short x;
    signed short y;
    signed short z;
} pos_struct;


