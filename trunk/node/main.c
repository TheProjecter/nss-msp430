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

#define MY_ADDR 4

unsigned int alarmed;
unsigned int broadcast;
unsigned int link_mode;

void MRFI_GpioIsr( void );

void main ( void ) { 
  unsigned short int tx_cmd;
  mrfiPacket_t tx_packet;

  // Initialize board devices 
  BSP_Init();
  MRFI_Init();
  mrfiSpiWriteReg(0x3E, 0xFF); // Increase Tx power
  MRFI_WakeUp();
  MRFI_RxOn();

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
  
  // Initialize Device States
  alarmed = 0;
  broadcast = 0;
  link_mode = 1;
  
  // Turn on both LEDs to signal initialization complete
  P1OUT |= (LED_RED+LED_GREEN);
  
  // Enter main loop
  while(1) {
    __bis_SR_register(GIE+LPM3_bits);

    if (link_mode) {
      P1OUT ^= (LED_RED+LED_GREEN);
      tx_cmd = NEW_NODE;
      broadcast = 1;
    } else {
      // Check state of node
      if (alarmed) {
        P1OUT ^= LED_RED;
        tx_cmd = ALARMED_NODE;    
      } else {
        P1OUT &= ~LED_RED;
        tx_cmd = RESET_NODE;
      }    
    }
  
    // Send any pending messages
    if (broadcast) {
      tx_packet.frame[0] = 8+20;
      tx_packet.frame[SRC_ADDR] = MY_ADDR;
      tx_packet.frame[DST_ADDR] = 0;
      tx_packet.frame[CMD] = tx_cmd;
      MRFI_Transmit(&tx_packet, MRFI_TX_TYPE_FORCED);
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
  
  // Required for RF interrupt
  MRFI_GpioIsr();    
  
  // Falling edge trigger
  if (P2IFG&TRIGGER_H2L) {
    P2IFG &= ~TRIGGER_H2L;
    
    if (P4IN&MODE_SELECT) {
      alarmed = 1;
    } else {
      alarmed = 0;
    }    
 
    if (!link_mode) {
      broadcast = 1;
    }
  }  
  
  // Rising edge trigger
  if (P2IFG&TRIGGER_L2H) {
    P2IFG &= ~TRIGGER_L2H;
    
    if (P4IN&MODE_SELECT) {
      alarmed = 0;
    } else {
      alarmed = 1;
    }    

    if (!link_mode) {
      broadcast = 1;
    }
  }    
}

void MRFI_RxCompleteISR( void ) {
  mrfiPacket_t rx_packet;
  unsigned short int rx_cmd;
  unsigned short int rx_dst;
  unsigned short int rx_src;
  
  // Grab packet from buffer
  MRFI_Receive(&rx_packet);

  // Gather packet data
  rx_dst = rx_packet.frame[DST_ADDR];
  rx_src = rx_packet.frame[SRC_ADDR];
  rx_cmd = rx_packet.frame[CMD];

  // Perform address filtering
  if (!(rx_src == MY_ADDR) && (rx_dst == MY_ADDR)) {
    switch (rx_cmd) {
      case ACK_NODE:
        P1OUT |= LED_GREEN;
        link_mode = 0;
        broadcast = 0;
        break;
      case ACK_ALARM:
        broadcast = 0;
        break;
      case ACK_RESET: 
        broadcast = 0;
        break;
    }
  }
}