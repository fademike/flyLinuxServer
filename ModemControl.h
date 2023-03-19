


void ModemControl_Work(void);


void ModemControl_SendPacket(char * buff);
void ModemControl_SendSymbol(char data);

int ModemControl_GetPacket(char * buf);
int ModemControl_GetByte(char * buf);

int ModemControl_getStatus(void);
int ModemControl_init(void);