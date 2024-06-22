


#pragma pack(push, 1)

typedef struct {
	uint8_t st;
	uint8_t len;
} rf_pack_header;
typedef struct {
	uint8_t status;
	uint16_t current_block;
	uint16_t total_block;
} rf_pack_program_header;
typedef struct {
	uint16_t crc;
} rf_pack_crc;
#pragma pack(pop)

typedef enum {
	PROGRAM_ERROR = -1,
	PROGRAM_NONE = 0,
	PROGRAM_RUN = 1,
	PROGRAM_FINISH = 2,
};

int pack_getLen(uint8_t * pack);
uint16_t pack_returnCRC(uint8_t * pack, int len);
void pack_setCRC(uint8_t * pack);
bool pack_checkCRC(uint8_t * pack);
void pack_read_single(uint8_t * pack);
int16_t pack_getRxBlock(void);
int16_t pack_getRxTotal(void);
void pack_read_raw_byte(uint8_t data);
void pack_read_raw_buffer(uint8_t * data, int len);

int program_block (int file, uint16_t send_block, uint16_t total_block, uint8_t * package);
int program_getCntBlock(int file);

void* thread_program(void *arg);
