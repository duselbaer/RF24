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

  // Initialize Button on P1.3
  P2DIR &= ~BIT3; // P1.3 --> input
  P2OUT |= BIT3;  // Configure resistors to be Pull-Up
  P2REN |= BIT3;  // Enable resistors

  // LED_1
  P1DIR |= BIT0 + BIT3 + BIT4;
  P1OUT &= ~(BIT0 + BIT3 + BIT4);

  logger << "Initializing SYS\n";
  msp430lib::MSP430_System system;
  system.init();

  __enable_interrupt();

  logger << "Initializing NRF24L01\n";

  NRF24L01_COMM nrf_comm(system);
  NRF24L01::NRF24L01<NRF24L01_COMM> nrf(nrf_comm);
  nrf.init();
  nrf.setChannel(119);
  nrf.setAddressWidth(4);
  nrf.setPayloadSize(0, 1);

  nrf.openWritingPipe(0xABCDEFAB);

  logger << "Press Button to send something\n";

  while(1)
  {
    // Wait for button to be pressed
    if ((P2IN & BIT3) == 0)
    {
      char buf = 'X';
      if (nrf.write(&buf, 15))
      {
        P1OUT ^= BIT0;
      }
      else
      {
        P1OUT ^= BIT3;
        logger << "-";
      }
    }
  }

  logger << "UNREACHABLE\n";
  while (1);
}

