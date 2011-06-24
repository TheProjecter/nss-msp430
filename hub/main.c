#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "commands.h"

#define MY_ADDR 0
#define MAX_NODES 10
#define WINDOW_MAX 5
#define WINDOW_CYCLE 10

uint8_t free_addr;
uint8_t hub;
uint8_t node_data[MAX_NODES];
uint8_t node_voltage[MAX_NODES];

void MRFI_GpioIsr( void );

void main( void ) {
  int time = 0;
  char update_txt[] = {"\r\n#ID,A,S,V.Vv"};
                      
  // Initialize Board Devices
  BSP_Init();
  
  // Initialize Comm
  P3SEL |= 0x30;
  UCA0CTL1 = UCSSEL_2;
  UCA0BR0 = 0x41;
  UCA0BR1 = 0x3;
  UCA0MCTL = UCBRS_2;
  UCA0CTL1 &= ~UCSWRST;
  IE2 |= UCA0RXIE;

  // Initialize Radio
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
  hub |= IDLE;
  hub &= ~(BROADCAST+ALARMED);
  free_addr = MAX_NODES;
  
  // Initialize node data
  int8_t i;  
  for (i = (MAX_NODES-1); i > 0; i--) {
    node_data[i] = 0;
    node_voltage[i] = 0;
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
    
    if (hub&IDLE) {
      if (time == WINDOW_CYCLE) {
        time = 0;  // Reset the clock
        hub &= ~IDLE; // Take the hub out of IDLE mode
        P1OUT &= ~LED_GREEN;
        
        for (i = MAX_NODES-1; i >= 0; i--) { // Reset node data
          if (node_data[i]&PAIRED) {
            node_data[i] &= ~ALIVE;
          }
        }
      }
    } else {
      if (time == WINDOW_MAX) {
        time = 0; // Reset the clock
        hub |= IDLE; // Put the hub in IDLE mode
        P1OUT |= LED_GREEN;
      
        for (i = MAX_NODES-1; i >= 0; i--) {
          if (node_data[i]&PAIRED) {
            update_txt[3] = '0'+(((i+1)/10)%10);
            update_txt[4] = '0'+((i+1)%10);
            
            if (node_data[i]&ALIVE) {
              update_txt[6] = '1';
            } else {
              update_txt[6] = '0';              
            }
            
            if (node_data[i]&ALARMED) {
              update_txt[8] = '1';
            } else {
              update_txt[8] = '0';              
            }
            
            update_txt[10] = '0'+((node_voltage[i]/10)%10);
            update_txt[12] = '0'+(node_voltage[i]%10); 
            
            TXString(update_txt, sizeof update_txt);
          }
         }
        
      } else {
        P1OUT ^= LED_GREEN;
      }
    }
    
    // Set alarm if in alarm mode
    if (hub&ALARMED) {
      P1OUT |= LED_RED;  
    } else {
      P1OUT &= ~LED_RED;  
    }   
    time++;
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
  uint8_t rx_src;  
  uint8_t rx_dst;
  uint8_t rx_cmd;
  uint8_t rx_data;
  uint8_t tx_cmd;
  uint8_t tx_data;  
  
  // Grab packet from buffer
  MRFI_Receive(&rx_packet);
  
  // Gather packet data
  rx_src = rx_packet.frame[SRC_ADDR];
  rx_dst = rx_packet.frame[DST_ADDR];
  rx_cmd = rx_packet.frame[CMD];
  rx_data = rx_packet.frame[DATA];  

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
      case NODE_ALIVE:
        node_data[rx_src-1] |= ALIVE;
        node_voltage[rx_src-1] = rx_data;
        tx_cmd = ACK_ALIVE;
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