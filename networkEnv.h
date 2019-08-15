




int SelectStatus(int Socket, long Timeout);

int RecvWithTimeout(int Socket, char *Buffer, int Len, long Timeout, int *bTimedOut );

void* threadTCPserver(void* thread_data);

int OpenTCP_Server(void);


