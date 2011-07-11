/*------------------------------------------------------------------------------
 * MODE_SELECT Properties
 *  Bit = 0, Circuit reset with H2L, Alarm set with L2H, Normally Open
 *  Bit = 1, Circuit reset with L2H, Alarm set with H2L, Normally Closed
 *----------------------------------------------------------------------------*/
  
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "commands.h"

/*------------------------------------------------------------------------------
 * Defines
 *----------------------------------------------------------------------------*/
#define MODE_SELECT 0x08 // P4.3 - Pin 8
#define TRIGGER_L2H 0x01 // P2.0 - Pin 3
#define TRIGGER_H2L 0x02 // P2.1 - Pin 4

#define LINK_MODE     0x01
#define WAKE_RADIO    0x02
#define BROADCAST     0x04
#define PAIRED        0x08
#define ALARMED       0x10
#define STATE_CHANGED 0x20
#define PACKET_RX     0x40

/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
void MRFI_GpioIsr( void );
__interrupt void Timer_A ( void );
__interrupt void Port1_ISR ( void );
__interrupt void Port2_ISR ( void );
__interrupt void ADC10_ISR ( void );

/*------------------------------------------------------------------------------
 * Globals
 *----------------------------------------------------------------------------*/
static volatile uint8_t NODE1 = 0; // Status register for device

/*------------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
void main ( void ) {
  static const uint8_t my_addr = 0x01;
  uint8_t tx_cmd;
  uint8_t tx_data;

  /* Initialize board devices */
  BSP_Init();
  MRFI_Init();

  /* Setup I/O */
  P1DIR |= (LED_RED+LED_GREEN);        // Enable LEDs  
  P1DIR &= ~PUSH_BUTTON;               // Enable push button  
  P1REN |= PUSH_BUTTON;                // Enable pull-up/down resistor
  P1IE |= PUSH_BUTTON;                 // Enable interrupt  
  P2DIR &= ~(TRIGGER_L2H+TRIGGER_H2L); // Enable inputs
  P4DIR &= ~MODE_SELECT;
  P2IE |= (TRIGGER_L2H+TRIGGER_H2L);   // Enable interrupts
  P2IES &= ~TRIGGER_L2H;               // Set rising edge select
  P2IES |= TRIGGER_H2L;                // Set falling edge select

  /* Setup Timer A */
  BCSCTL3 |= LFXT1S_2;   // Source VLO @ 12kHz
  TACCTL0 = CCIE;        // Enable TimerA interrupt
  TACCR0 = 12000;        // ~1Hz
  TACTL = MC_1+TASSEL_1; // Count up + ACLK
  
  /* Initialize device settings */
  NODE1 |= LINK_MODE;

  /* Signal boot complete */  
  P1OUT |= (LED_RED+LED_GREEN);
  
  /* Enter main loop */
  while(1) {    
    __bis_SR_register(GIE+LPM3_bits);
    
    if (NODE1&PACKET_RX) {
      mrfiPacket_t rx_packet;
      uint8_t rx_dst;
      uint8_t rx_cmd;
      uint8_t rx_data;
  
      /* Grab packet from buffer */
      MRFI_Receive(&rx_packet);
      NODE1 &= ~PACKET_RX;
    
      /* Gather packet data */
      rx_dst = rx_packet.frame[DST_ADDR];
      rx_cmd = rx_packet.frame[CMD];
      rx_data = rx_packet.frame[DATA];
    
      /* Perform address filtering */
      if ((rx_dst == my_addr) || (!(NODE1&PAIRED))) {
        switch (rx_cmd) {
          case ACK_NODE:
            NODE1 |= PAIRED;
            NODE1 &= ~(LINK_MODE+WAKE_RADIO);
            break;
          case ACK_ALARM:
            NODE1 &= ~(WAKE_RADIO+STATE_CHANGED);
            break;
          case ACK_RESET:
            NODE1 &= ~(WAKE_RADIO+STATE_CHANGED);
            break;
          case ACK_ALIVE:
            break;
        }
      }      
    }
    
    if (NODE1&LINK_MODE) {      
      P1OUT ^= (LED_RED+LED_GREEN);
      tx_cmd = NEW_NODE;
      NODE1 |= (WAKE_RADIO+BROADCAST);      
    } else {
      if (NODE1&STATE_CHANGED) { // Alarm was set/reset
        if (NODE1&ALARMED) {
          P1OUT |= LED_RED;
          tx_cmd = ALARMED_NODE;
        } else {
          P1OUT &= ~LED_RED;
          tx_cmd = RESET_NODE;
        }
        NODE1 |= (WAKE_RADIO+BROADCAST);
      }
    }
    
    if (NODE1&WAKE_RADIO) {
      MRFI_WakeUp();
      MRFI_RxOn();
    }

    if (NODE1&BROADCAST) {  
      mrfiPacket_t tx_packet;
      
      tx_packet.frame[0] = 8+20;
      tx_packet.frame[SRC_ADDR] = my_addr;
      tx_packet.frame[DST_ADDR] = 0x00;  
      tx_packet.frame[CMD] = tx_cmd;
      tx_packet.frame[DATA] = tx_data;
      MRFI_Transmit(&tx_packet, MRFI_TX_TYPE_FORCED);
      
      NODE1 &= ~BROADCAST;
    }
    
    if (!(NODE1&WAKE_RADIO)) {
      MRFI_Sleep();
    }  
  }
}

/*------------------------------------------------------------------------------
 * MRFI Rx interrupt service routine
 *----------------------------------------------------------------------------*/
void MRFI_RxCompleteISR( void ) {
  NODE1 |= PACKET_RX;
}

/*------------------------------------------------------------------------------
 * Timer A0 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A ( void ) {
  
  /* Wake up CPU after ISR exits */
  __bic_SR_register_on_exit(LPM3_bits);
}

/*------------------------------------------------------------------------------
 * PORT1 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR ( void ) {  

}

/*------------------------------------------------------------------------------
 * PORT2 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR ( void ) {
  
  /* Required for RF interrupt */
  MRFI_GpioIsr();    
  
  if (!(NODE1&LINK_MODE)) {
    
    /* Falling edge trigger */
    if (P2IFG&TRIGGER_H2L) {
      P2IFG &= ~TRIGGER_H2L;
      
      if (P4IN&MODE_SELECT) {
        NODE1 |= ALARMED;
      } else {
        NODE1 &= ~ALARMED;
      }     
    }  
  
    /* Rising edge trigger */
    if (P2IFG&TRIGGER_L2H) {
      P2IFG &= ~TRIGGER_L2H;
      
      if (P4IN&MODE_SELECT) {
        NODE1 &= ~ALARMED;
      } else {
        NODE1 |= ALARMED;
      }    
    }
    
    NODE1 |= STATE_CHANGED;
  }
}

/*------------------------------------------------------------------------------
 * ADC10 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
  __bic_SR_register_on_exit(CPUOFF);
}