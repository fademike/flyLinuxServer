


#define nRF24//nRF24	//SI4463

typedef enum  {
	STATE_IDLE 				= 0,
	STATE_RX				= 1,
	STATE_TX				= 2
} STATE_TRANSIEVER;


// Need to add to msTimer: if(delay_to_wait_answer>0) delay_to_wait_answer--; and other! //FIXME
// and:
//	extern volatile int delay_to_wait_answer;
//	extern volatile int delay_measure_tx;
//	extern volatile int delay_synchro;
//	if(delay_to_wait_answer>0) delay_to_wait_answer--;
//	if(delay_synchro>0) delay_synchro--;	// timer for synchronization modem
//	delay_measure_tx++;



#define READCMDSTREAM_ARRAY 10



#define PACKET_LEN 64	// length receive buffer of modem

#define SYNCHRO_MODE 1	// 1- synchronization mode; 0- simple mode. More in stm8+si4463 project
////////////////////////////For SYNCHRO_MODE :

#if SYNCHRO_MODE


#define SYNCHRO_TIME 500                                                        // sync time
#define SYNCHRO_TWAIT 6*4//6*4

#endif

//DELETE									//circular buffer for send out by RF
#define UART_BUF_SIZE 255			// buffer lenght

#define BUFFER_TX_RF 64

#define MODEM_PINCONTROL 0	// For debug pin output


//void ModemControl_init(void);

void ModemControl_ReadSymbolRfomRF_Callback(char data);
void ModemControl_ReadMSGRfomRF_Callback(char * data);

void ModemControl_SendPacket(char * buff);
void ModemControl_SendSymbol(char buff);

int ModemControl_ReadOnly(void);
// void ModemControl_Work(void);

//
void ModemControl_Loop(void);
// void ModemControl_SendPacket(char * buff);
// void ModemControl_SendSymbol(char data);
// int ModemControl_GetPacket(char * buf);
// int ModemControl_GetByte(char * buf);
// int ModemControl_getStatus(void);
 int ModemControl_init(void);
//

void CRC_PacketCalculate(unsigned char * buff);
int CRC_PacketCheck(unsigned char * buff);

#define SI_SIMPLE_PRINTF

#ifdef SI_SIMPLE_PRINTF
//void Printf(const char *fmt, ...)
#else

#endif
#ifdef SI_POOR_PRINTF
//void Printf_str(char * str);
//void Printf_hex(int);
#endif

