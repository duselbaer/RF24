#pragma once

#include "config.h"

#include <stdint.h>

#ifdef HAVE_ENDIAN_H
# include <endian.h>
#elif HAVE_MACHINE_ENDIAN_H
# include <machine/endian.h>
#else
# error Endianess Header Required
#endif

namespace NRF24L01 {
inline namespace v1 {

  template <typename T>
  const T& min(const T& a, const T& b)
  {
    return a < b ? a : b;
  }

  template <typename T>
  const T& max(const T& a, const T& b)
  {
    return a > b ? a : b;
  }

  template<typename COMM>
  class NRF24L01
  {
    friend COMM;

    COMM&             m_comm;

    uint8_t           m_payload_size;
    uint8_t           m_feature;
    uint8_t           m_config;

  public:
    NRF24L01(COMM& comm)
      : m_comm(comm)
      , m_payload_size(32)
      , m_feature(0x00)
      , m_config(0)
    {
    }



    auto init() -> void
    {
      //! \todo Give the Interrupt Handler Callback instead
      m_comm.init(this);

      // Wait 100ms for device to settle
      m_comm.delay_ms(100);

      clearPendingInterrupts();
      close();
      setRetransmitDelay(2000);
      setRetransmitCount(15);
      setSpeed(RF24_SPEED_250KBPS);
      setPower(RF24_POWER_MINUS18DB);

      setAddressWidth(5);
      enableDynamicPayloads();
      enableAckPayload();
      enableCRC(RF24_CRC_2BYTE);

      writeRegister(RF24_REGISTER_SETUP_RETR, 0x7F);

      powerDown();
      flushTx();
      flushRx();
    }



    //! \brief Check if this instance of NRF24L01 has been initialized correctly
    //!
    //! \todo Implement if necessary
    constexpr auto isValid() -> bool
    {
      return true;
    }



    auto powerDown() -> void
    {
      m_comm.CE_LOW();
      setConfig(0);
    }



    auto standBy() -> void
    {
      m_comm.CE_LOW();
      setConfig(RF24_CONFIG_PWR_UP);
      m_comm.delay_ms(5);
    }



    auto startListening() -> void
    {
      standBy();
      flushRx();

      // Clear any pending interrupts
      clearPendingInterrupts(RF24_STATUS_RX_DR);

      // Set PWR_UP & PRIM_RX
      setConfig(RF24_CONFIG_PRIM_RX | RF24_CONFIG_PWR_UP);
      m_comm.CE_HIGH();

      // Now it takes 130us for PLL to settle
    }



    //! \brief Stop listening by setting CE to low and flushing RX and TX buffers
    auto stopListening() -> void
    {
      m_comm.CE_LOW();
      flushTx();
      flushRx();
    }



    //! Sends the R_RX_PAYLOAD command to the device which returns
    //! the data at the front of the FIFO.
    //!
    //! \return the status retrieved by the device
    auto read(uint8_t* buffer, uint8_t const buffer_size) -> uint8_t
    {
      while (!available())
      {
        m_comm.waitForInterrupt();
        m_comm.clearPendingInterrupt();
      }

      m_comm.CS_LOW();

      auto status = m_comm.SPI_SHIFT(RF24_COMMAND_R_RX_PAYLOAD);
      m_comm.SPI_TRANSFER_SYNC(buffer, buffer, buffer_size);

      m_comm.CS_HIGH();

      // Clear the interrupt flag
      clearPendingInterrupts(RF24_STATUS_RX_DR);

      return status;
    }



    auto openWritingPipe(uint64_t const & address, bool const auto_ack = true) -> void
    {
#if __BYTE_ORDER == __LITTLE_ENDIAN
      // Note that AVR & MSP430 8-bit uC's store this LSB first, and the NRF24L01(+)
      // expects it LSB first too, so we're good.

      writeRegister(RF24_REGISTER_RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&address), 5);
      writeRegister(RF24_REGISTER_TX_ADDR, reinterpret_cast<const uint8_t*>(&address), 5);
#else
#error Not yet implemented for BIG_ENDIAN
#endif

      uint8_t en_aa = readRegister(RF24_REGISTER_EN_AA);
      if (auto_ack)
      {
        en_aa |= 1;
      }
      else
      {
        en_aa &= ~(0x01);
      }
      writeRegister(RF24_REGISTER_EN_AA, en_aa);

      uint8_t en_rxaddr = readRegister(RF24_REGISTER_EN_RXADDR);
      en_rxaddr |= 0x01;
      writeRegister(RF24_REGISTER_EN_RXADDR, en_rxaddr);
    }



    auto getReadingPipeAddress(uint8_t const number) -> uint64_t
    {
      uint64_t address;

      if (number < 2)
      {
        readRegister(rxAddresses[number], 5, reinterpret_cast<uint8_t*>(&address));
      }
      else
      {
        readRegister(RF24_REGISTER_RX_ADDR_P1, 5, reinterpret_cast<uint8_t*>(&address));
        *reinterpret_cast<uint8_t*>(&address) = readRegister(rxAddresses[number]);
      }

      return address;
    }



    auto openReadingPipe(uint8_t const number, uint64_t const address) -> void
    {
#if __BYTE_ORDER == __LITTLE_ENDIAN
      // Note that AVR & MSP430 8-bit uC's store this LSB first, and the NRF24L01(+)
      // expects it LSB first too, so we're good.

      if (number < 2)
      {
        writeRegister(rxAddresses[number], reinterpret_cast<const uint8_t*>(&address), 5);
      }
      else
      {
        writeRegister(rxAddresses[number], reinterpret_cast<const uint8_t*>(&address), 1);
      }
#else
#error Not yet implemented for BIG_ENDIAN
#endif

      uint8_t en_aa = readRegister(RF24_REGISTER_EN_AA);
      en_aa |= (1 << number);
      writeRegister(RF24_REGISTER_EN_AA, en_aa);

      uint8_t en_rxaddr = readRegister(RF24_REGISTER_EN_RXADDR);
      en_rxaddr |= (1 << number);
      writeRegister(RF24_REGISTER_EN_RXADDR, en_rxaddr);
    }



    //! Blocking write
    auto write(void const * buf, uint8_t const len) -> bool
    {
      m_comm.clearPendingInterrupt();
      startWrite(buf, len);

      // TODO add timeout
      m_comm.waitForInterrupt();
      uint8_t status = getStatus();

      bool ret;

      if (status & RF24_STATUS_TX_DS)
      {
        ret = true;
      }
      else if (status & RF24_STATUS_MAX_RT)
      {
        ret = false;
      }

      clearPendingInterrupts();
      return ret;
    }



    //! Non-blocking write
    auto startWrite(void const * buf, uint8_t const len) -> void
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_W_TX_PAYLOAD);
      m_comm.SPI_TRANSMIT_SYNC(reinterpret_cast<uint8_t const*>(buf), len);
      m_comm.CS_HIGH();

      standBy();
      clearPendingInterrupts(RF24_STATUS_TX_DS | RF24_STATUS_MAX_RT);
      m_comm.CE_HIGH();
      m_comm.delay_us(15);
      m_comm.CE_LOW();
    }



    auto available(uint8_t* pipe = nullptr) -> bool
    {
      // 0001 is TX-FIFO-STATUS
      // xxxT is RX-FIFO-STATUS where 111T means RX-FIFO-EMPTY
      return (getStatus() & 0x0E) < 0x0E;
    }



    auto close() -> void
    {
      writeRegister(RF24_REGISTER_EN_RXADDR, 0x00);
      writeRegister(RF24_REGISTER_EN_AA, 0x00);
      writeRegister(RF24_REGISTER_DYNPD, 0x00);
    }



    auto setRetransmitDelay(uint16_t delay) -> void
    {
      uint8_t x = readRegister(RF24_REGISTER_SETUP_RETR);
      if (delay > 4000)
      {
        delay = 4000;
      }
      if (delay < 500)
      {
        delay = 500;
      }
      delay = (delay - 250) / 250;
      delay = delay << 4;
      writeRegister(RF24_REGISTER_SETUP_RETR, x | (delay & 0xF0));
    }



    auto setRetransmitCount(uint8_t count) -> void
    {
      // TODO: Implement
      //static_assert(false, "Unimplemented");
    }



    auto setAddressWidth(uint8_t const width) -> void
    {
      writeRegister(RF24_REGISTER_SETUP_AW, (width - 2) & 0x03);
    }



    auto setChannel(uint8_t const channel) -> void
    {
      writeRegister(RF24_REGISTER_RF_CH, min(channel, static_cast<uint8_t>(125)));
    }



    typedef enum {
      RF24_CRC_1BYTE = 0x00
      , RF24_CRC_2BYTE = 0x04
      , RF24_CRC_MASK = 0x04
    } Rf24_CRC;



    auto enableCRC(Rf24_CRC const type) -> void
    {
      m_config |= RF24_CONFIG_EN_CRC;
      m_config &= ~RF24_CRC_MASK;
      m_config |= type;

      setConfig(0);
    }



    //! \brief Convenience Function to be compliant to the RF24 interface
    auto setCRCLength(Rf24_CRC const type) -> void
    {
      enableCRC(type);
    }



    auto disableCRC() -> void
    {
      setConfig(getConfig() & ~RF24_CONFIG_EN_CRC);
    }



    typedef enum {
      RF24_SPEED_250KBPS = 0x20
      , RF24_SPEED_1MBPS = 0x00
      , RF24_SPEED_2MBPS = 0x08
      , RF24_SPEED_MASK = 0x28
    } Rf24_Speed;



    typedef enum {
      RF24_POWER_0DB = 0x06
      , RF24_POWER_MINUS6DB = 0x04
      , RF24_POWER_MINUS12DB = 0x02
      , RF24_POWER_MINUS18DB = 0x00
      , RF24_POWER_MASK = 0x07
    } Rf24_Power;


    auto setSpeed(Rf24_Speed const speed) -> void
    {
      uint8_t rf_setup = readRegister(RF24_REGISTER_RF_SETUP);
      rf_setup &= ~(RF24_SPEED_MASK);
      rf_setup |= speed;
      writeRegister(RF24_REGISTER_RF_SETUP, rf_setup);
    }



    //! \brief Convenience Function to be compliant to the RF24 interface
    auto setDataRate(Rf24_Speed const speed) -> void
    {
      setSpeed(speed);
    }



    auto setPower(Rf24_Power const power) -> void
    {
      uint8_t rf_setup = readRegister(RF24_REGISTER_RF_SETUP);
      rf_setup &= ~(RF24_POWER_MASK);
      rf_setup |= power;
      writeRegister(RF24_REGISTER_RF_SETUP, rf_setup);
    }



    //! Enables dynamic payloads on _all_ pipes
    auto enableDynamicPayloads() -> void
    {
      setFeature(getFeature() | RF24_FEATURE_EN_DPL);
    }



    auto disableDynamicPayloads() -> void
    {
      setFeature(getFeature() & ~RF24_FEATURE_EN_DPL);
    }



    //! Set the payload size for a pipe
    //!
    //! If dynamic payload is requested the enableDynamicPayloads feature is set automatically
    //!
    //! \param payload_size 0 for dynamic payloads, everything else up to 32 allowed
    auto setPayloadSize(uint8_t const pipe, uint8_t payload_size) -> void
    {
      payload_size = min(payload_size, static_cast<uint8_t>(32));
      if (payload_size == 0)
      {
        if (getFeature() & RF24_FEATURE_EN_DPL == 0)
        {
          enableDynamicPayloads();
        }

        uint8_t dynpd = readRegister(RF24_REGISTER_DYNPD);
        uint8_t const dynpd_bit = (1 << pipe);
        if ((dynpd & dynpd_bit) == 0)
        {
          writeRegister(RF24_REGISTER_DYNPD, dynpd | dynpd_bit);
        }
      }
      else
      {
        writeRegister(static_cast<Rf24Register>(
          static_cast<uint8_t>(RF24_REGISTER_RX_PW_P0) + pipe), payload_size);
      }
    }


    auto enableAckPayload() -> void
    {
      // TODO - Correct Name?
      setFeature(getFeature() | RF24_FEATURE_EN_DYN_ACK);
    }



    auto disableAckPayload() -> void
    {
      setFeature(getFeature() & ~RF24_FEATURE_EN_DYN_ACK);
    }



    //! Flush TX buffer
    auto flushTx() -> void
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_FLUSH_TX);
      m_comm.CS_HIGH();
    }



    //! Flush RX buffer
    auto flushRx() -> void
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_FLUSH_RX);
      m_comm.CS_HIGH();
    }



    //! Read the status register
    auto getStatus() -> uint8_t
    {
      return readRegister(RF24_REGISTER_STATUS);
    }



    template<typename OUTPUT>
    auto dumpStatus(OUTPUT & output) -> void
    {
      output << "NRF24 Status\n"
             << "============\n\n";
      dumpRegister(output, RF24_REGISTER_CONFIG, "CONFIG");
      dumpRegister(output, RF24_REGISTER_EN_AA, "EN_AA");
      dumpRegister(output, RF24_REGISTER_EN_RXADDR, "EN_RXADDR");
      dumpRegister(output, RF24_REGISTER_SETUP_AW, "SETUP_AW");
      dumpRegister(output, RF24_REGISTER_SETUP_RETR, "SETUP_RETR");
      dumpRegister(output, RF24_REGISTER_RF_CH, "RF_CH");
      dumpRegister(output, RF24_REGISTER_RF_SETUP, "RF_SETUP");
      dumpRegister(output, RF24_REGISTER_STATUS, "STATUS");
      dumpRegister(output, RF24_REGISTER_OBSERVE_TX, "OBSERVE_TX");
      dumpRegister(output, RF24_REGISTER_CD, "CD");

      dumpAddress(output, "RX_ADDR_P0", getReadingPipeAddress(0));
      dumpAddress(output, "RX_ADDR_P1", getReadingPipeAddress(1));
      dumpAddress(output, "RX_ADDR_P2", getReadingPipeAddress(2));
      dumpAddress(output, "RX_ADDR_P3", getReadingPipeAddress(3));
      dumpAddress(output, "RX_ADDR_P4", getReadingPipeAddress(4));
      dumpAddress(output, "RX_ADDR_P5", getReadingPipeAddress(5));

      dumpRegister(output, RF24_REGISTER_TX_ADDR, "TX_ADDR", 5);
      dumpRegister(output, RF24_REGISTER_RX_PW_P0, "RX_PW_P0");
      dumpRegister(output, RF24_REGISTER_RX_PW_P1, "RX_PW_P1");
      dumpRegister(output, RF24_REGISTER_RX_PW_P2, "RX_PW_P2");
      dumpRegister(output, RF24_REGISTER_RX_PW_P3, "RX_PW_P3");
      dumpRegister(output, RF24_REGISTER_RX_PW_P4, "RX_PW_P4");
      dumpRegister(output, RF24_REGISTER_RX_PW_P5, "RX_PW_P5");
      dumpRegister(output, RF24_REGISTER_FIFO_STATUS, "FIFO_STATUS");
      dumpRegister(output, RF24_REGISTER_DYNPD, "DYNPD");
      dumpRegister(output, RF24_REGISTER_FEATURE, "FEATURE");
    }
  private:
    enum {
      RF24_NOP = 0xFF
      , RF24_REGISTER_MASK = 0x12
    };

    typedef enum {
      RF24_REGISTER_CONFIG = 0x00
      , RF24_REGISTER_EN_AA = 0x01
      , RF24_REGISTER_EN_RXADDR = 0x02
      , RF24_REGISTER_SETUP_AW = 0x03
      , RF24_REGISTER_SETUP_RETR = 0x04
      , RF24_REGISTER_RF_CH = 0x05
      , RF24_REGISTER_RF_SETUP = 0x06
      , RF24_REGISTER_STATUS = 0x07
      , RF24_REGISTER_OBSERVE_TX = 0x08
      , RF24_REGISTER_CD = 0x09
      , RF24_REGISTER_RX_ADDR_P0 = 0x0A
      , RF24_REGISTER_RX_ADDR_P1 = 0x0B
      , RF24_REGISTER_RX_ADDR_P2 = 0x0C
      , RF24_REGISTER_RX_ADDR_P3 = 0x0D
      , RF24_REGISTER_RX_ADDR_P4 = 0x0E
      , RF24_REGISTER_RX_ADDR_P5 = 0x0F
      , RF24_REGISTER_TX_ADDR = 0x10
      , RF24_REGISTER_RX_PW_P0 = 0x11
      , RF24_REGISTER_RX_PW_P1 = 0x12
      , RF24_REGISTER_RX_PW_P2 = 0x13
      , RF24_REGISTER_RX_PW_P3 = 0x14
      , RF24_REGISTER_RX_PW_P4 = 0x15
      , RF24_REGISTER_RX_PW_P5 = 0x16
      , RF24_REGISTER_FIFO_STATUS = 0x17
      , RF24_REGISTER_DYNPD = 0x1C
      , RF24_REGISTER_FEATURE = 0x1D
    } Rf24Register;

    static constexpr Rf24Register rxAddresses[] =
    {
      RF24_REGISTER_RX_ADDR_P0
        , RF24_REGISTER_RX_ADDR_P1
        , RF24_REGISTER_RX_ADDR_P2
        , RF24_REGISTER_RX_ADDR_P3
        , RF24_REGISTER_RX_ADDR_P4
        , RF24_REGISTER_RX_ADDR_P5
    };

    typedef enum {
      RF24_COMMAND_R_REGISTER = 0x00
      , RF24_COMMAND_W_REGISTER = 0x20
      , RF24_COMMAND_REGISTER_MASK = 0x1F
      , RF24_COMMAND_R_RX_PAYLOAD = 0x61
      , RF24_COMMAND_W_TX_PAYLOAD = 0xA0
      , RF24_COMMAND_FLUSH_TX = 0xE1
      , RF24_COMMAND_FLUSH_RX = 0xE2
      , RF24_COMMAND_REUSE_TX_PL = 0xE3
      , RF24_COMMAND_ACTIVATE = 0x50
      , RF24_COMMAND_R_RX_PL_WID = 0x60
      , RF24_COMMAND_W_ACK_PAYLOAD = 0xA8
      , RF24_COMMAND_W_TX_PAYLOAD_NOACK = 0xB0
    } Rf24Commands;



    typedef enum {
      RF24_STATUS_TX_FULL = 0x01
      , RF24_STATUS_RX_P_NO_MASK = 0x0E
      , RF24_STATUS_MAX_RT = 0x10
      , RF24_STATUS_TX_DS = 0x20
      , RF24_STATUS_RX_DR = 0x40
      , RF24_STATUS_IRQMASK = RF24_STATUS_MAX_RT + RF24_STATUS_TX_DS + RF24_STATUS_RX_DR
    } Rf24Status;



    typedef enum {
      RF24_CONFIG_PRIM_RX = 0x01
      , RF24_CONFIG_PWR_UP = 0x02
      , RF24_CONFIG_CRC0 = 0x04
      , RF24_CONFIG_EN_CRC = 0x08
      , RF24_CONFIG_MASK_MAX_RT = 0x10
      , RF24_CONFIG_MASK_TX_DS = 0x20
      , RF24_CONFIG_MASK_RX_DR = 0x40
    } Rf24Config;



    typedef enum {
      RF24_FEATURE_EN_DPL = 0x04
      , RF24_FEATURE_EN_ACK_PAY = 0x02
      , RF24_FEATURE_EN_DYN_ACK = 0x01
    } Rf24Features;



    auto writeRegister(Rf24Register const register_name
      , uint8_t const * data, uint8_t const size) -> void
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_W_REGISTER + (register_name & RF24_COMMAND_REGISTER_MASK));
      m_comm.SPI_TRANSMIT_SYNC(data, size);
      m_comm.CS_HIGH();
    }



    auto writeRegister(Rf24Register const register_name , uint8_t const value) -> void
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_W_REGISTER + (register_name & RF24_COMMAND_REGISTER_MASK));
      m_comm.SPI_SHIFT(value);
      m_comm.CS_HIGH();
    }



    auto readRegister(Rf24Register const register_name) -> uint8_t
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_R_REGISTER + (register_name & RF24_COMMAND_REGISTER_MASK));
      uint8_t ret = m_comm.SPI_SHIFT(RF24_NOP);
      m_comm.CS_HIGH();

      return ret;
    }



    auto readRegister(Rf24Register const register_name, uint8_t const size,
      uint8_t* buffer) -> void
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(RF24_COMMAND_R_REGISTER + (register_name & RF24_COMMAND_REGISTER_MASK));
      m_comm.SPI_TRANSFER_SYNC(buffer, buffer, size);
      m_comm.CS_HIGH();
    }


    auto setConfig(uint8_t const config) -> void
    {
      writeRegister(RF24_REGISTER_CONFIG, m_config | config);
    }



    auto getConfig() -> uint8_t
    {
      return readRegister(RF24_REGISTER_CONFIG);
    }



    auto setFeature(uint8_t const new_feature) -> void
    {
      m_feature = new_feature;
      writeRegister(RF24_REGISTER_FEATURE, m_feature);
    }



    auto getFeature() const -> uint8_t
    {
      return m_feature;
    }


    auto clearPendingInterrupts(uint8_t const mask = RF24_STATUS_IRQMASK) -> void
    {
      writeRegister(RF24_REGISTER_STATUS, mask);
    }



    template<typename OUTPUT>
    auto dumpRegister(OUTPUT& output, Rf24Register const register_name, char const * name)
      -> void
    {
      output << name << " = " << readRegister(register_name) << "\n";
    }



    template<typename OUTPUT>
    auto dumpRegister(OUTPUT& output, Rf24Register const register_name, char const * name,
      uint8_t const size) -> void
    {
      uint8_t buf[size];

      readRegister(register_name, size, buf);
      output << name << " = ";
      for (auto i = 0; i < size; ++i)
      {
        output << buf[i] << " ";
      }
      output << "\n";
    }



    template<typename OUTPUT>
    auto dumpAddress(OUTPUT& output, char const * name, uint64_t const address) -> void
    {
      output << name << " = ";
      for (auto i = 0; i < 5; ++i)
      {
        output << reinterpret_cast<uint8_t const*>(&address)[i] << " ";
      }
      output << "\n";
    }

};

template<typename COMM>
constexpr typename NRF24L01<COMM>::Rf24Register NRF24L01<COMM>::rxAddresses[];

}
}

