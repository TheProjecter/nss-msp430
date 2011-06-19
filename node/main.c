/*******************************************************************************

  MODE_SELECT Properties
    Bit = 0, Circuit reset with H2L, Alarm set with L2H, Normally Open
    Bit = 1, Circuit reset with L2H, Alarm set with H2L, Normally Closed

*******************************************************************************/
  
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "commands.h"

#define MODE_SELECT 0x08 // P4.3 - Pin 8
#define TRIGGER_L2H 0x01 // P2.0 - Pin 3
#define TRIGGER_H2L 0x02 // P2.1 - Pin 4

uint8_t my_addr;
uint8_t node;

void MRFI_GpioIsr( void );

void main ( void ) { 
  uint8_t tx_cmd;
  mrfiPacket_t tx_packet;

  // Initialize board devices 
  BSP_Init();
  MRFI_Init();
  mrfiSpiWriteReg(0x3E, 0xFF); // Increase Tx power

  // Setup I/O
  P1DIR |= (LED_RED+LED_GREEN);        // Enable LEDs  
  P1DIR &= ~PUSH_BUTTON;               // Enable push button  
  P1REN |= PUSH_BUTTON;                // Enable pull-up/down resistor
  P1IE |= PUSH_BUTTON;                 // Enable interrupt  
  P2DIR &= ~(TRIGGER_L2H+TRIGGER_H2L); // Enable inputs
  P4DIR &= ~MODE_SELECT;
  P2IE |= (TRIGGER_L2H+TRIGGER_H2L);   // Enable interrupts
  P2IES &= ~TRIGGER_L2H;               // Set rising edge select
  P2IES |= TRIGGER_H2L;                // Set falling edge select

  // Setup Timer A
  BCSCTL3 |= LFXT1S_2;
  TACCTL0 = CCIE;
  TACCR0 = 12000;
  TACTL = MC_1+TASSEL_1;  
  
  // Initialize device settings
  my_addr = 0;
  node |= (LINK_MODE+WAKE_RADIO);
  node &= ~(ALARMED+BROADCAST);
  
  // Turn on both LEDs to signal initialization complete
  P1OUT |= (LED_RED+LED_GREEN);
  
  // Enter main loop
  while(1) {    
    __bis_SR_register(GIE+LPM3_bits);

    if (node&LINK_MODE) {
      P1OUT ^= (LED_RED+LED_GREEN);
      tx_cmd = NEW_NODE;
      node |= BROADCAST;
    } else {
      // Check state of node
      if (node&ALARMED) {
        P1OUT ^= LED_RED;
        tx_cmd = ALARMED_NODE;    
      } else {
        P1OUT &= ~LED_RED;
        tx_cmd = RESET_NODE;
      }    
    }
    
    // Wake radio if needed
    if (node&WAKE_RADIO) {
      MRFI_WakeUp();
      MRFI_RxOn();
    }

    // Send any pending messages
    if (node&BROADCAST) {
  
      // Setup packet info
      tx_packet.frame[0] = 8+20;
      tx_packet.frame[SRC_ADDR] = my_addr;
      tx_packet.frame[DST_ADDR] = 0;  
      tx_packet.frame[CMD] = tx_cmd;
      MRFI_Transmit(&tx_packet, MRFI_TX_TYPE_FORCED);
    }
    
    // Put radio to sleep
    if (!(node&WAKE_RADIO)) {
      MRFI_Sleep();
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
  
  // Falling edge trigger
  if (P2IFG&TRIGGER_H2L) {
    P2IFG &= ~TRIGGER_H2L;
    
    if (P4IN&MODE_SELECT) {
      node |= ALARMED;
    } else {
      node &= ~ALARMED;
    }    
 
    if (!(node&LINK_MODE)) {
      node |= (WAKE_RADIO+BROADCAST);
    }
  }  
  
  // Rising edge trigger
  if (P2IFG&TRIGGER_L2H) {
    P2IFG &= ~TRIGGER_L2H;
    
    if (P4IN&MODE_SELECT) {
      node &= ~ALARMED;
    } else {
      node |= ALARMED;
    }    

    if (!(node&LINK_MODE)) {
      node |= (WAKE_RADIO+BROADCAST);
    }
  }    
}

void MRFI_RxCompleteISR( void ) {
  mrfiPacket_t rx_packet;
  uint8_t rx_dst;
  uint8_t rx_cmd;
  uint8_t rx_data;
  
  // Grab packet from buffer
  MRFI_Receive(&rx_packet);

  // Gather packet data
  rx_dst = rx_packet.frame[DST_ADDR];
  rx_cmd = rx_packet.frame[CMD];
  rx_data = rx_packet.frame[DATA];
  
  // Perform address filtering  
  if ((rx_dst == my_addr) || (!(node&PAIRED))) {
    switch (rx_cmd) {
      case ACK_NODE:
        node |= PAIRED;
        node &= ~(LINK_MODE+BROADCAST+WAKE_RADIO);
        my_addr = rx_data;
        break;
      case ACK_ALARM:
        node &= ~(BROADCAST+WAKE_RADIO);
        break;
      case ACK_RESET: 
        node &= ~(BROADCAST+WAKE_RADIO);
        break;
    }
  }
}