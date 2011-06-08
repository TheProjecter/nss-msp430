#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "commands.h"

#define MY_ADDR 0
#define MAX_NODES 10

#define ONLINE 0
#define ALARM 1

unsigned short int node_data[MAX_NODES][3];

void MRFI_GpioIsr( void );

void main( void ) {
  unsigned int alarm_mode;
  
  // Initialize Board Devices
  BSP_Init();
  MRFI_Init();
  mrfiSpiWriteReg(0x3E, 0xFF); // Increase Tx power
  MRFI_WakeUp();
  MRFI_RxOn();
  
  // Setup I/O
  P1DIR |= (LED_RED+LED_GREEN);
  P1DIR &= ~PUSH_BUTTON;
  P1REN |= PUSH_BUTTON;
  P1IE |= PUSH_BUTTON;
  
  // Setup Timer A
  BCSCTL3 |= LFXT1S_2;
  TACCTL0 = CCIE;
  TACCR0 = 12000;
  TACTL = MC_1+TASSEL_1; 
  
  // Initialize hub data
  unsigned int i;
  
  for (i = 0; i < MAX_NODES; i++) {
    node_data[i][ONLINE] = 0;
    node_data[i][ALARM]  = 0;
  }
  
  // Turn on power LED
  P1OUT |= LED_GREEN;
  
  // Enter main loop
  while(1) {
    __bis_SR_register(GIE+LPM3_bits);
    
    // Check if any nodes are alarmed
    alarm_mode = 0;
    for (i = 0; i < MAX_NODES; i++) {
      if (node_data[i][ALARM]) {
        alarm_mode = 1;
      }
    }
    
    // Set alarm if in alarm mode
    if (alarm_mode) {
      P1OUT |= LED_RED;  
    } else {
      P1OUT &= ~LED_RED;  
    }
  }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A ( void ) {
  __bic_SR_register_on_exit(LPM3_bits);
}

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR ( void ) {
  
}

#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR ( void ) {

  MRFI_GpioIsr();  
}

void MRFI_RxCompleteISR() {
  mrfiPacket_t rx_packet;
  unsigned short int rx_cmd;
  unsigned short int rx_dst;
  unsigned short int rx_src;  
  unsigned short int tx_cmd;  
  unsigned int broadcast;
  
  // Grab packet from buffer
  MRFI_Receive(&rx_packet);
  
  // Gather packet data
  rx_src = rx_packet.frame[SRC_ADDR];
  rx_dst = rx_packet.frame[DST_ADDR];
  rx_cmd = rx_packet.frame[CMD];  

  // Perform address filtering  
  if (!(rx_src == MY_ADDR) && rx_dst == MY_ADDR) {
    switch (rx_cmd) {
      case NEW_NODE:
        node_data[rx_src][ONLINE] = 1;
        node_data[rx_src][ALARM]  = 0;
        tx_cmd = ACK_NODE;
        broadcast = 1;
        break;
      case ALARMED_NODE:
        node_data[rx_src][ALARM] = 1;
        tx_cmd = ACK_ALARM;
        broadcast = 1;
        break;
      case RESET_NODE:
        node_data[rx_src][ALARM] = 0;
        tx_cmd = ACK_RESET;
        broadcast = 1;
        break;
    }
  } else {
    broadcast = 0;
  }
  
  // Send any pending messages
  if (broadcast) {
    mrfiPacket_t tx_packet;
    
    tx_packet.frame[0] = 8+20;
    tx_packet.frame[SRC_ADDR] = MY_ADDR;
    tx_packet.frame[DST_ADDR] = rx_src;
    tx_packet.frame[CMD] = tx_cmd;
    MRFI_Transmit(&tx_packet, MRFI_TX_TYPE_FORCED);
  }
}