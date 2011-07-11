/* RF commands: Node */
#define NEW_NODE     0x01
#define ALARMED_NODE 0x02
#define RESET_NODE   0x03
#define NODE_ALIVE   0x04
#define LOW_BATT     0x0F

/* RF commands: Hub */
#define ACK_NODE     0x11
#define ACK_ALARM    0x12
#define ACK_RESET    0x13
#define ACK_ALIVE    0x14
#define ACK_LOW_BATT 0x1F

/* RF packet fields */
#define SRC_ADDR 4
#define DST_ADDR 8
#define CMD      9
#define DATA     10

/* Board devices */
#define LED_RED     0x01
#define LED_GREEN   0x02
#define PUSH_BUTTON 0x04