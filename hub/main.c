#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "commands.h"

#define MY_ADDR   0
#define MAX_NODES 10

uint8_t free_addr;
uint8_t hub;
uint8_t node_data[MAX_NODES];

void MRFI_GpioIsr( void );

void main( void ) {

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
  
  // Initialize hub settings
  hub &= ~(BROADCAST+ALARMED);
  free_addr = MAX_NODES;
  
  // Initialize node data
  int8_t i;  
  for (i = (MAX_NODES-1); i >= 0; i--) {
    node_data[i] = 0;
  }
  
  // Turn on power LED
  P1OUT |= LED_GREEN;
  
  // Enter main loop
  while(1) {
    __bis_SR_register(GIE+LPM3_bits);
    
    // Check if any nodes are alarmed
    hub &= ~ALARMED;
    for (i = MAX_NODES-1; i >= 0; i--) {
      if (node_data[i]&ALARMED) {
        hub |= ALARMED;
      }
    }
    
    // Set alarm if in alarm mode
    if (hub&ALARMED) {
      P1OUT |= LED_RED;  
    } else {
      P1OUT &= ~LED_RED;  
    }
  }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A ( void ) {

  // Wake up CPU after ISR exits
  __bic_SR_register_on_exit(LPM3_bits);
}

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR ( void ) {
  
}

#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR ( void ) {

  // Required for RF interrupt
  MRFI_GpioIsr();  
}

void MRFI_RxCompleteISR() {
  mrfiPacket_t rx_packet;
  uint8_t rx_cmd;
  uint8_t rx_dst;
  uint8_t rx_src;  
  uint8_t tx_cmd;
  uint8_t tx_data;  
  
  // Grab packet from buffer
  MRFI_Receive(&rx_packet);
  
  // Gather packet data
  rx_src = rx_packet.frame[SRC_ADDR];
  rx_dst = rx_packet.frame[DST_ADDR];
  rx_cmd = rx_packet.frame[CMD];  

  // Perform address filtering  
  if (rx_dst == MY_ADDR) {
    switch (rx_cmd) {
      case NEW_NODE:
        if (free_addr > 0) {
          int8_t curr_addr;
          
          for (curr_addr = MAX_NODES-1; curr_addr >= 0; curr_addr--) {
            if (!(node_data[curr_addr]&PAIRED)) { // Address is free to use
              free_addr--;
              node_data[curr_addr] |= (PAIRED+ALIVE);
              tx_cmd = ACK_NODE;
              tx_data = curr_addr+1;
              hub |= BROADCAST;
              break;
            }
          }
        }
        break;
      case ALARMED_NODE:
        node_data[rx_src-1] |= ALARMED;
        tx_cmd = ACK_ALARM;
        hub |= BROADCAST;
        break;
      case RESET_NODE:
        node_data[rx_src-1] &= ~ALARMED;
        tx_cmd = ACK_RESET;
        hub |= BROADCAST;
        break;
    }
  } else {
    hub &= ~BROADCAST;
  }
  
  // Send any pending messages
  if (hub&BROADCAST) {
    mrfiPacket_t tx_packet;
    
    tx_packet.frame[0] = 8+20;
    tx_packet.frame[SRC_ADDR] = MY_ADDR;
    tx_packet.frame[DST_ADDR] = rx_src;
    tx_packet.frame[CMD] = tx_cmd;
    tx_packet.frame[DATA] = tx_data;
    MRFI_Transmit(&tx_packet, MRFI_TX_TYPE_FORCED);
  }
}