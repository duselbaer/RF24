#pragma once

#include "msp430lib/usci/v1/MSP430_USCI_B0.hpp"
#include "msp430lib/usci/v1/MSP430_USCI_SPI.hpp"

#include "msp430lib/sys/v1/MSP430_System.hpp"

#include <stddef.h>
#include <stdint.h>

extern void (*port2_interrupt_function)(void*);
extern void* port2_interrupt_payload;

namespace NRF24L01 {
inline namespace v1 {
namespace driver {

// NRF24L01 - COMM
// USCI B0 - BIT5 = SCLK, BIT6 = MISO, BIT7 = MOSI
// CS = PORT2.0
// CE = PORT2.1
class MSP430
{
  typedef msp430lib::MSP430_USCI_SPI<msp430lib::MSP430_USCI_B0> SPI;
  SPI                       m_spi;

  msp430lib::MSP430_System& m_sys;
  volatile bool             m_interrupt_occured;

  constexpr static volatile uint8_t& OUTPUT = P2OUT;
  constexpr static volatile uint8_t& DIRECTION = P2DIR;
  constexpr static volatile uint8_t& REN = P2REN;
  constexpr static uint8_t CS = BIT0;
  constexpr static uint8_t CE = BIT1;
  constexpr static uint8_t INT = BIT2;

public:
  MSP430(msp430lib::MSP430_System& sys)
    : m_sys(sys)
    , m_interrupt_occured(false)
  {
  }

  template<typename DRIVER>
  auto init(DRIVER* /* nrf */) -> void
  {
    // Initialize SPI
    m_spi.init();

    // Set CS & CE as output
    DIRECTION |= CS + CE;

    // INT as input
    DIRECTION &= ~INT; // Set as input
    OUTPUT |= INT;  // Enable PUll-Up
    REN |= INT;  // Enable Pull-Up

    // Enable PIN Change interrupt on falling edge
    P2IE |= INT;
    P2IES |= INT;
    P2IFG &= ~INT;

    port2_interrupt_function = &MSP430::interrupt_handler;
    port2_interrupt_payload = this;
  }

  auto CE_LOW() -> void
  {
    OUTPUT &= ~CE;
  }

  auto CE_HIGH() -> void
  {
    OUTPUT |= CE;
  }

  auto CS_LOW() -> void
  {
    OUTPUT &= ~CS;
    __delay_cycles(16);
  }

  auto CS_HIGH() -> void
  {
    __delay_cycles(16);
    OUTPUT |= CS;
    __delay_cycles(16);
  }

  auto SPI_SHIFT(const uint8_t data) -> uint8_t
  {
    uint8_t x = m_spi.shift_data(data);
    return x;
  }

  auto SPI_TRANSMIT_SYNC(const uint8_t data[], const size_t len) -> void
  {
    for (size_t x = 0; x < len; ++x)
    {
      SPI_SHIFT(data[x]);
    }
  }

  auto SPI_TRANSFER_SYNC(const uint8_t data[], uint8_t out[], const size_t len) -> void
  {
    for (size_t x = 0; x < len; ++x)
    {
      out [x] = SPI_SHIFT(data[x]);
    }
  }

  auto delay_ms(uint8_t const ms) -> void
  {
    m_sys.delay_ms(ms);
  }

  auto delay_us(uint8_t const us) -> void
  {
    __delay_cycles(us * 16);
  }

  auto clearPendingInterrupt() -> void
  {
    m_interrupt_occured = false;
  }

  auto waitForInterrupt() -> void
  {
    while (! m_interrupt_occured)
    {
      LPM4;
    }
  }

private:
  static void interrupt_handler(void* payload)
  {
    if (P2IFG & INT)
    {
      MSP430* comm = (MSP430*) payload;
      comm->m_interrupt_occured = true;

      P1OUT ^= BIT0;
      P2IFG &= ~INT;
    }
  }
};

}
}
}

