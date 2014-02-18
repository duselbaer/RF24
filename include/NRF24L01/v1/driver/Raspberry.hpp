#pragma once

// #include "NRF24L01/v1/NRF24L01.hpp"

extern "C"
{
# include <bcm2835.h>
}

#include <cstdint>
#include <unistd.h>

namespace NRF24L01 {
inline namespace v1
namespace driver {

  //! \brief Provides all the hardware specific functions for the NRF24L01 module
  //!
  //! SPI - MISO, MOSI, SCK
  //! CS  - GPIO17
  //! CE  - GPIO27
  //! INT - GPIO22
  //!
  //! \todo - Move initialization of bcm2835 out of here
  //! \todo - Create scoped SPI object
  template<uint8_t CS = RPI_V2_GPIO_P1_11, uint8_t CE = RPI_V2_GPIO_P1_13, uint8_t INT = RPI_V2_GPIO_P1_15>
  class Raspberry
  {
  public:
    void init(NRF24L01<NRF24L01_COMM>* nrf)
    {
      if (!bcm2835_init())
      {
        std::cerr << "Initialization Failed" << std::endl;
        return;
      }

      // INT - input, pull up enabled
      bcm2835_gpio_fsel(INT, BCM2835_GPIO_FSEL_INPT);
      bcm2835_gpio_set_pud(INT, BCM2835_GPIO_PUD_UP);

      // CS - output
      bcm2835_gpio_fsel(CS, BCM2835_GPIO_FSEL_OUTP);

      // CE - output
      bcm2835_gpio_fsel(CE, BCM2835_GPIO_FSEL_OUTP);

      // Initialize SPI - max. SPI clock 8 MHz
      bcm2835_spi_begin();
      bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);   // The default
      bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                // The default
      bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32); // 7.8125 MHz

      // We do not use the CS pins assigned with the SPI interface here because
      // we have to provide methods triggering the CS line and we don't know
      // the scope of these operations.
      CE_LOW();
      CS_HIGH();
    }

    void delay_ms(int const ms)
    {
      usleep(ms * 1000);
    }

    inline void CS_LOW()
    {
      bcm2835_gpio_write(CS, LOW);
    }

    inline void CS_HIGH()
    {
      bcm2835_gpio_write(CS, HIGH);
    }

    inline void CE_HIGH()
    {
      std::cout << "CE HIGH" << std::endl;
      bcm2835_gpio_write(CE, HIGH);
    }

    inline void CE_LOW()
    {
      std::cout << "CE LOW" << std::endl;
      bcm2835_gpio_write(CE, LOW);
    }

    uint8_t SPI_SHIFT(uint8_t const data)
    {
      uint8_t tmp = bcm2835_spi_transfer(data);
      return tmp;
    }

    void SPI_TRANSMIT_SYNC(uint8_t const * data, uint8_t const size)
    {
      bcm2835_spi_writenb(reinterpret_cast<char*>(const_cast<uint8_t*>(data)), size);
    }

    void SPI_TRANSFER_SYNC(uint8_t const * output, uint8_t * input, uint8_t const size)
    {
      bcm2835_spi_transfernb(reinterpret_cast<char*>(const_cast<uint8_t*>(output)), reinterpret_cast<char*>(input), size);
    }

    void waitForInterrupt()
    {
      // delay_ms(30);
    }

    void clearPendingInterrupt()
    {
    }
  };
}
}
}

