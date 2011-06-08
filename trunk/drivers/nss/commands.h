/* RF Commands */
//#define UNUSED 0x00
#define NEW_NODE     0x01
#define ACK_NODE     0x02
#define ALARMED_NODE 0x03
#define ACK_ALARM    0x04
#define RESET_NODE   0x05
#define ACK_RESET    0x06
//#define UNUSED 0x07
//#define UNUSED 0x08
//#define UNUSED 0x09
//#define UNUSED 0x0A
//#define UNUSED 0x0B
//#define UNUSED 0x0C
//#define UNUSED 0x0D
//#define UNUSED 0x0E
#define LOW_BATT     0x0F

/* RF Packet Information */
#define SRC_ADDR 4
#define DST_ADDR 8
#define CMD      9
#define DATA     10

/* Board devices */
#define LED_RED     0x01
#define LED_GREEN   0x02
#define PUSH_BUTTON 0x04