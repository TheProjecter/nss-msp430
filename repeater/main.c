#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"

int main( void )
{
  BSP_Init();
  MRFI_Init();
  mrfiSpiWriteReg(0x3E, 0xFF); // Increase Tx power
  MRFI_WakeUp();
  MRFI_RxOn();
  __bis_SR_register(GIE+LPM4_bits);
}

void MRFI_RxCompleteISR() {
  mrfiPacket_t in_packet;

  MRFI_Receive(&in_packet); // Received packet
  P1OUT ^= 0x02; // Toggle green LED
  MRFI_Transmit(&in_packet, MRFI_TX_TYPE_FORCED);   // Send packet
  P1OUT ^= 0x02; // Toggle green LED
}