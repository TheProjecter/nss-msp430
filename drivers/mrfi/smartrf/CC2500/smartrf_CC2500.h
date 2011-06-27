/* Deviation = 38.085938 */
/* Base frequency = 2424.999878 */
/* Carrier frequency = 2425.749695 */
/* Channel number = 3 */
/* Carrier frequency = 2425.749695 */
/* Modulated = true */
/* Modulation format = 2-FSK */
/* Manchester enable = false */
/* Sync word qualifier mode = 30/32 sync word bits detected */
/* Preamble count = 4 */
/* Channel spacing = 249.938965 */
/* Carrier frequency = 2425.749695 */
/* Data rate = 2.39897 */
/* RX filter BW = 203.125000 */
/* Data format = Normal mode */
/* Length config = Variable packet length mode. Packet length configured by the first byte after sync word */
/* CRC enable = true */
/* Packet length = 255 */
/* Device address = 0 */
/* Address config = No address check */
/* CRC autoflush = false */
/*  = false */
/* TX power = 1 */
/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 ***************************************************************/

#ifndef SMARTRF_CC2500_H
#define SMARTRF_CC2500_H

#define SMARTRF_RADIO_CC2500

#define SMARTRF_SETTING_FSCTRL1    0x08
#define SMARTRF_SETTING_IOCFG0     0x06
#define SMARTRF_SETTING_FSCTRL0    0x00
#define SMARTRF_SETTING_FREQ2      0x5D
#define SMARTRF_SETTING_FREQ1      0x44
#define SMARTRF_SETTING_FREQ0      0xEC
#define SMARTRF_SETTING_MDMCFG4    0x86
#define SMARTRF_SETTING_MDMCFG3    0x83
#define SMARTRF_SETTING_MDMCFG2    0x03
#define SMARTRF_SETTING_MDMCFG1    0x23
#define SMARTRF_SETTING_MDMCFG0    0x3B
#define SMARTRF_SETTING_CHANNR     0x03
#define SMARTRF_SETTING_DEVIATN    0x44
#define SMARTRF_SETTING_FREND1     0x56
#define SMARTRF_SETTING_FREND0     0x10
#define SMARTRF_SETTING_MCSM0      0x18
#define SMARTRF_SETTING_FOCCFG     0x16
#define SMARTRF_SETTING_BSCFG      0x6C
#define SMARTRF_SETTING_AGCCTRL2   0x03
#define SMARTRF_SETTING_AGCCTRL1   0x40
#define SMARTRF_SETTING_AGCCTRL0   0x91
#define SMARTRF_SETTING_FSCAL3     0xA9
#define SMARTRF_SETTING_FSCAL2     0x0A
#define SMARTRF_SETTING_FSCAL1     0x00
#define SMARTRF_SETTING_FSCAL0     0x11
#define SMARTRF_SETTING_FSTEST     0x59
#define SMARTRF_SETTING_TEST2      0x88
#define SMARTRF_SETTING_TEST1      0x31
#define SMARTRF_SETTING_TEST0      0x0B
#define SMARTRF_SETTING_FIFOTHR    0x07
#define SMARTRF_SETTING_IOCFG2     0x29
#define SMARTRF_SETTING_PKTCTRL1   0x04
#define SMARTRF_SETTING_PKTCTRL0   0x05
#define SMARTRF_SETTING_ADDR       0x00
#define SMARTRF_SETTING_PKTLEN     0xFF

#endif
