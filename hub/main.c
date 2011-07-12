#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "commands.h"

/*------------------------------------------------------------------------------
 * Defines
 *----------------------------------------------------------------------------*/
#define MAX_NODES 10
#define WINDOW_LENGTH 12

#define WAKE_RADIO  0x01
#define BROADCAST   0x02
#define ALARMED     0x04
#define SEND_STATUS 0x08
#define ALIVE       0x10

/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
void MRFI_GpioIsr( void );
void ProcessPacket(mrfiPacket_t inc_packet);
__interrupt void Timer_A ( void );
__interrupt void Port1_ISR ( void );
__interrupt void Port2_ISR ( void );

/*------------------------------------------------------------------------------
 * Globals
 *----------------------------------------------------------------------------*/
static const uint8_t my_addr = 0x00;
volatile uint8_t NODE_DATA1[MAX_NODES]; // Register for node MAC addresses
volatile uint8_t NODE_DATA2[MAX_NODES]; // Register for node status
volatile uint8_t NODE_DATA3[MAX_NODES]; // Register for node voltages
volatile uint8_t HUB1;                  // Status register for device
volatile uint8_t WindowCounter = WINDOW_LENGTH;

/*------------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
void main( void ) {  
  char update_txt[] = {"\r\n#ID,A,S,V.Vv"};
                      
  /* Initialize Board Devices */
  BSP_Init();
  
  /* Initialize Radio */
  MRFI_Init();
  MRFI_WakeUp();
  MRFI_RxOn();

  /* Initialize UART */
  P3SEL |= 0x30;
  UCA0CTL1 = UCSSEL_2;
  UCA0BR0 = 0x41;
  UCA0BR1 = 0x3;
  UCA0MCTL = UCBRS_2;
  UCA0CTL1 &= ~UCSWRST;
  IE2 |= UCA0RXIE;
  
  /* Setup I/O */
  P1DIR |= (LED_RED+LED_GREEN);
  P1DIR &= ~PUSH_BUTTON;
  P1REN |= PUSH_BUTTON;
  P1IE |= PUSH_BUTTON;
  
  /* Setup Timer A */
  BCSCTL3 |= LFXT1S_2;   // Source VLO @ 12kHz
  TACCTL0 = CCIE;        // Enable TimerA interrupt
  TACCR0 = 12000;        // ~1Hz
  TACTL = MC_1+TASSEL_1; // Count up + ACLK
  
  /* Initialize hub settings */
  HUB1 = 0;
  _EINT();  
  
  /* Initialize node data */
  for (int i = (MAX_NODES-1); i >= 0; i--) {
    NODE_DATA1[i] = 0;
    NODE_DATA2[i] = 0;
    NODE_DATA3[i] = 0;
  }
  
  /* Turn on power LED */
  P1OUT |= LED_GREEN;
  
  /* Enter main loop */
  while(1) {
    
    /* Check if any nodes are alarmed */
    HUB1 &= ~ALARMED;
    for (int i = (MAX_NODES-1); i >= 0; i--) {
      if (NODE_DATA2[i]&ALARMED) {
        HUB1 |= ALARMED;
      }
    }

    /* Set alarm if in alarm mode */
    if (HUB1&ALARMED) {
      P1OUT |= LED_RED;  
    } else {
      P1OUT &= ~LED_RED;  
    }   
    
    /* Send data to PC */
    if (HUB1&SEND_STATUS) {
      for (int i = (MAX_NODES-1); i >= 0; i--) {
        if (NODE_DATA1[i] != 0) {
          update_txt[3] = '0'+(((i+1)/10)%10);
          update_txt[4] = '0'+((i+1)%10);

          if (NODE_DATA2[i]&ALIVE) {
            update_txt[6] = '1';
          } else {
            update_txt[6] = '0';              
          }
          
          if (NODE_DATA2[i]&ALARMED) {
            update_txt[8] = '1';
          } else {
            update_txt[8] = '0';              
          }
          
          update_txt[10] = '0'+((NODE_DATA3[i]/10)%10);
          update_txt[12] = '0'+(NODE_DATA3[i]%10); 

          TXString(update_txt, sizeof update_txt);
          
          NODE_DATA2[i] &= ~ALIVE;
          NODE_DATA3[i] = 0;
        }
      }
      
      HUB1 &= ~SEND_STATUS;
      TACCTL0 |= CCIE;      
    }    
  }
}

/*------------------------------------------------------------------------------
 * MRFI Rx interrupt service routine
 *----------------------------------------------------------------------------*/
void MRFI_RxCompleteISR() {
  mrfiPacket_t rx_packet;

  /* Grab packet from buffer */
  MRFI_Receive(&rx_packet);
  
  /* Send packet off to be processed in another thread */
  ProcessPacket(rx_packet);
}

/*------------------------------------------------------------------------------
 * ProcessPacket method
 *----------------------------------------------------------------------------*/
void ProcessPacket(mrfiPacket_t inc_packet) {
  uint8_t tx_cmd  = 0;
  uint8_t tx_data = 0;
  uint8_t rx_src  = 0;
  uint8_t rx_dst  = 0;
  uint8_t rx_cmd  = 0;
  uint8_t rx_data = 0;

  /* Gather packet data */
  rx_src = inc_packet.frame[SRC_ADDR];
  rx_dst = inc_packet.frame[DST_ADDR];
  rx_cmd = inc_packet.frame[CMD];
  rx_data = inc_packet.frame[DATA];
      
  /* Perform address filtering */
  if (rx_dst == my_addr) {
    int node_index = 0;
        
    /* Find index for node in memory */
    for (int i = (MAX_NODES-1); i >=0; i--) {
      if (NODE_DATA1[i] == rx_src) {
        node_index = i;
        break;
      }
    }
        
    switch (rx_cmd) {
      case NEW_NODE:
        for (int i = (MAX_NODES-1); i >= 0; i--) {
          if (NODE_DATA1[i] == 0) {
            NODE_DATA1[i] = rx_src;
            NODE_DATA2[i] |= ALIVE;
            NODE_DATA3[i] = 36; // Start with highest possible voltage
            tx_cmd = ACK_NODE;
            HUB1 |= BROADCAST;
            break;
          }
        }
        break;
      case ALARMED_NODE:
        break;
      case RESET_NODE:
        break;
      case NODE_ALIVE:
        NODE_DATA2[node_index] |= ALIVE;
        NODE_DATA3[node_index] = rx_data;
        break;
    }
  }

  /* Send data over RF */
  if (HUB1&BROADCAST) {
    mrfiPacket_t tx_packet;
     
    tx_packet.frame[0] = 8+20;
    tx_packet.frame[SRC_ADDR] = my_addr;
    tx_packet.frame[DST_ADDR] = rx_src;  
    tx_packet.frame[CMD] = tx_cmd;
    tx_packet.frame[DATA] = tx_data;
    MRFI_Transmit(&tx_packet, MRFI_TX_TYPE_FORCED);
      
    HUB1 &= ~BROADCAST;      
  }
}

/*------------------------------------------------------------------------------
 * Timer A0 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A ( void ) {
  WindowCounter--;
  
  if (WindowCounter == 0) {
    TACCTL0 &= ~CCIE;              // Disable TimerA interrupt
    WindowCounter = WINDOW_LENGTH; // Reset counter
    HUB1 |= SEND_STATUS;           // Set flag to send data to PC
  }
}

/*------------------------------------------------------------------------------
 * PORT1 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR ( void ) {
  // Do nothing...
}

/*------------------------------------------------------------------------------
 * PORT2 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR ( void ) {

  /* Required for RF interrupt */
  MRFI_GpioIsr();  
}