#include "../NRF24L01.hpp"
#include "NRF24L01_COMM.hpp"

// Logging
#include "msp430lib/usci/v1/MSP430_USCI_UART.hpp"
#include "msp430lib/usci/v1/MSP430_USCI_A0.hpp"
#include "UART.hpp"
#include "Logger.hpp"

// Basic System Methods
#include "msp430lib/sys/v1/MSP430_System.hpp"

// MCU Support
#include <msp430.h>

int main(int, char**)
{
  // Disable Watchdog, Setup Clock Module
  WDTCTL  = WDTPW + WDTHOLD;  // Stop WDT
  BCSCTL1 = CALBC1_16MHZ;      // Set DCO
  DCOCTL  = CALDCO_16MHZ;

  // Setup Logging - we use the UART on USCI_A0
  typedef UART<msp430lib::MSP430_USCI_UART<msp430lib::MSP430_USCI_A0>> LoggingDeviceType;
  LoggingDeviceType logging_device;
  logging_device.init();

  typedef Logger<LoggingDeviceType> SerialLogger;
  SerialLogger logger(static_cast<LoggingDeviceType&&>(logging_device));

  // Initialize LED
  P1DIR |= (1 << 0);
  P1OUT &= ~(1 << 0);

  logger << "Initializing SYS\n";
  msp430lib::MSP430_System system;
  system.init();

  __enable_interrupt();

  logger << "Initializing NRF24L01...\n";
  NRF24L01_COMM comm(system);
  NRF24L01::NRF24L01<NRF24L01_COMM> nrf(comm);
  logger << "Devices created ...\n";

  nrf.init();

  char addr[5];
  // Set our RX address
  addr[0] = 0xBE;
  addr[1] = 0xEF;
  addr[2] = 0xBE;
  addr[3] = 0xEF;
  addr[4] = 0x00;

  nrf.setChannel(119);
  nrf.setAddressWidth(4);

  logger << "Setup receiver address\n";
  nrf.openReadingPipe(0, 0xABCDEFAB);
  nrf.setPayloadSize(0, 15);

  logger << "Start Listening\n";
  nrf.startListening();

  logger << "Receiving\n";
  nrf.dumpStatus(logger);

  while(1)
  {
    logger << ".";
    uint8_t buf[32];

    nrf.read(&buf[0], 32);
    logger << "-";
    //P1OUT ^= BIT0;
  }

  logger << "UNREACHABLE\n";
  while (1);
}

