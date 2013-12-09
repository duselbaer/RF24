#pragma once

#include <stdint.h>
#include "msp430lib/sys/v1/ISys.hpp"

#include "RF24.h"

#ifdef LOGGING
#include <Logger.hpp>
#endif

template<typename COMM>
class NRF24L01
{
  friend COMM;

  COMM              m_comm;
  msp430lib::ISys&  m_sys;
#ifdef LOGGING
  ILogger*          m_logger;
#endif

  uint8_t           m_ptx;    // 1 while data is sent, 0 if not
  uint8_t           m_channel;
  uint8_t           m_rf_settings;
  uint8_t           m_config;

public:
  NRF24L01(msp430lib::ISys& sys)
    : m_sys(sys)
#ifdef LOGGING
    , m_logger(nullptr)
#endif
    , m_ptx(0)
    , m_channel(0)
    , m_rf_settings(0)
    , m_config(0)
  {
  }

#ifdef LOGGING
  void setLogger(ILogger* logger)
  {
    m_logger = logger;
  }
#endif

  // Initialize the NRF24L01 module
  void init()
  {
    m_sys.init();
    m_comm.init(this);

    m_channel = 2;
    m_config = ( (1<<MASK_RX_DR) | (1<<EN_CRC) | (0<<CRCO) );

    m_comm.CS_HIGH();
    m_comm.CE_LOW();
  }

  //! Set the address where the next packet should be sent to
  void setupTXAddress(const char address[5])
  {
    write_register(TX_ADDR, (const uint8_t*) address, 5);
  }

  //! Set the address where this instance receives any data
  //! Channel 0 has a unique address
  //! Channel 1-5 share the most significant part of the address
  //! and the last byte might differ
  void setupRXAddress(const uint8_t channel, const char address[5])
  {
    write_register(static_cast<Register>(static_cast<uint8_t>(RX_ADDR_P0) + channel)
      , reinterpret_cast<const uint8_t*>(address), 5);
  }

  // Set the RF channel to use
  void setChannel(const uint8_t channel)
  {
    m_channel = channel;
  }

  void initAsReceiver(const size_t payload_size)
  {
    write_config_register(RF_CH, m_channel);
    write_config_register(RF_SETUP, m_rf_settings);
    write_config_register(RX_PW_P0, payload_size);
    write_config_register(RX_PW_P1, payload_size);

    m_ptx = 0;
    rx_powerup();

    m_comm.CE_HIGH();
  }

  // Prepare the module for sending data
  void initAsSender()
  {
    uint8_t tx_addr[5];

#ifdef LOGGING
    *m_logger << "[N] initAsSender(" << m_channel << ") called\n";
#endif

    write_config_register(RF_CH, m_channel);
    write_config_register(RF_SETUP, m_rf_settings);
    // write_config_register(CONFIG, m_config);

    // write_config_register(SETUP_RETR, (ARD_750 | ARC_15));
/*
    switch (m_channel)
    {
    case 0:
      tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P0_B0_DEFAULT_VAL;
      break;
    case 1:
      tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
      break;
    case 2:
      tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
      tx_addr[0] = RX_ADDR_P2_DEFAULT_VAL;
      break;
    case 3:
      tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
      tx_addr[0] = RX_ADDR_P3_DEFAULT_VAL;
      break;
    case 4:
      tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
      tx_addr[0] = RX_ADDR_P4_DEFAULT_VAL;
      break;
    case 5:
      tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
      tx_addr[0] = RX_ADDR_P5_DEFAULT_VAL;
      break;
    }

    set_tx_addr(tx_addr);
    set_rx_addr(tx_addr);
*/
    m_ptx = 0;
  }

  // Sends a data package to the default address. Be sure to send the correct
  // amount of bytes as configured as payload on the receiver.
  void send(const uint8_t* value, const uint8_t len)
  {
    // Wait until last packet has been sent
    while (m_ptx);

    m_comm.CE_LOW();
    m_ptx = 1;
    tx_powerup();

    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(FLUSH_TX);
    m_comm.CS_HIGH();

    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(W_TX_PAYLOAD);
    m_comm.SPI_TRANSMIT_SYNC(value, len);
    m_comm.CS_HIGH();

    m_comm.CE_HIGH(); // Start transmission
    m_sys.delay_us(10);
    m_comm.CE_LOW();
  }

  void sendNoAck(const uint8_t* value, const uint8_t len)
  {
    // Wait until last packet has been sent
    while (m_ptx);

    m_comm.CE_LOW();
    m_ptx = 1;

    flushTX();

    tx_powerup();

    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(W_TX_PAYLOAD_NOACK);
    m_comm.SPI_TRANSMIT_SYNC(value, len);
    m_comm.CS_HIGH();

    m_comm.CE_HIGH(); // Start transmission
    m_sys.delay_us(10);
    m_comm.CE_LOW();
  }

  uint8_t read(uint8_t* buffer, const uint8_t len)
  {
    uint8_t status;

    m_comm.CE_LOW();

    m_comm.CS_LOW();
    status = m_comm.SPI_SHIFT(R_RX_PAYLOAD);
    m_comm.SPI_TRANSFER_SYNC(buffer, buffer, len);
    m_comm.CS_HIGH();

    write_config_register(STATUS, (1 << RX_DR));

    m_comm.CE_HIGH();

    return status;
  }

  // Check if there's some data to read
  bool dataReady()
  {
    if (m_ptx)
    {
      return false;
    }

    uint8_t status;
    m_comm.CS_LOW();
    status = m_comm.SPI_SHIFT(NOP);
    m_comm.CS_HIGH();

    return status & (1 << RX_DR);
  }

private:
  typedef enum {
    // Read command and status registers.
    R_REGISTER = 0

    // Write command and status registers. Executable in power down or standby modes only.
    , W_REGISTER = 0x20

    // Register Mask for R_REGISTER & W_REGISTER
    , REGISTER_MASK = 0x1F

    // Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is
    // deleted from FIFO after it is read. Used in RX mode.
    , R_RX_PAYLOAD = 0x61

    // Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
    , W_TX_PAYLOAD = 0xA0

    // Flush TX FIFO, used in TX mode
    , FLUSH_TX = 0xE1

    // Flush RX FIFO, used in RX mode
    // Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
    , FLUSH_RX = 0xE2

    // Used for a PTX device
    // Reuse last transmitted payload. Packets are repeatedly retransmitted as long as CE is high.
    // TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed.
    // TX payload reuse must not be activated or deacti- vated during package transmission
    , REUSE_TX_PL = 0xE3

    , ACTIVATE = 0x50

    // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
    , R_RX_PL_WID = 0x60

    // Used in RX mode.
    // Write Payload to be transmitted together with ACK packet on PIPE PPP.
    // (PPP valid in the range from 000 to 101). Maximum three ACK packet payloads can be pending.
    // Payloads with same PPP are handled using first in - first out principle.
    // Write payload: 1– 32 bytes. A write operation always starts at byte 0.
    , W_ACK_PAYLOAD = 0xA8

    // Used in TX mode. Disables AUTOACK on this specific packet.
    , W_TX_PAYLOAD_NOACK = 0xB0

    // No Operation. Might be used to read the STATUS register
    , NOP = 0xFF
  } Command;

  typedef enum {
    // Configuration Register
    CONFIG = 0x00

    // Enable ‘Auto Acknowledgment’ Function Dis- able this functionality to be compatible with nRF2401
    , EN_AA = 0x01

    // Enabled RX Addresses
    , EN_RXADDR = 0x02

    // Setup of Address Widths
    , SETUP_AW = 0x03

    // Setup of Automatic Retransmission
    , SETUP_RETR = 0x04

    // RF Channel
    , RF_CH = 0x05

    // RF Setup Register
    , RF_SETUP = 0x06

    // Status Register (In parallel to the SPI command word applied on the MOSI pin,
    // the STATUS reg- ister is shifted serially out on the MISO pin)
    , STATUS = 0x07

    // Transmit observe register
    , OBSERVE_TX = 0x08

    // Carrier Detect
    , CD = 0x09

    // Receive address data pipe 0. 5 Bytes maximum length. (LSByte is written first.
    // Write the number of bytes defined by SETUP_AW)
    , RX_ADDR_P0 = 0x0A
    , RX_ADDR_P1 = 0x0B
    , RX_ADDR_P2 = 0x0C
    , RX_ADDR_P3 = 0x0D
    , RX_ADDR_P4 = 0x0E
    , RX_ADDR_P5 = 0x0F

    // Transmit address. Used for a PTX device only. (LSByte is written first)
    // Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if
    // this is a PTX device with Enhanced ShockBurstTM enabled.
    , TX_ADDR = 0x10
    , RX_PW_P0 = 0x11
    , RX_PW_P1 = 0x12
    , RX_PW_P2 = 0x13
    , RX_PW_P3 = 0x14
    , RX_PW_P4 = 0x15
    , RX_PW_P5 = 0x16
    , FIFO_STATUS = 0x17
    , DYNPD = 0x1C
    , FEATURE = 0x1D

  } Register;

  typedef enum {
    // Mask interrupt caused by RX_DR
    // 1: Interrupt not reflected on the IRQ pin
    // 0: Reflect RX_DR as active low interrupt on the IRQ pin
    MASK_RX_DR = 6

    // Mask interrupt caused by TX_DS
    // 1: Interrupt not reflected on the IRQ pin
    // 0: Reflect TX_DS as active low interrupt on the IRQ pin
    , MASK_TX_DS = 5

    // Mask interrupt caused by MAX_RT
    // 1: Interrupt not reflected on the IRQ pin
    // 0: Reflect MAX_RT as active low interrupt on the IRQ pin
    , MASK_MAX_RT = 4

    // Enable CRC. Forced high if one of the bits in the EN_AA is high
    , EN_CRC = 3

    // CRC encoding scheme '0' - 1 byte
    // '1' – 2 bytes
    , CRCO = 2

    // 1: POWER UP, 0:POWER DOWN
    , PWR_UP = 1

    // RX/TX control 1: PRX, 0: PTX
    , PRIM_RX = 0
  } Config;

  typedef enum {
    // Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFOb. Write 1 to clear bit.
    RX_DR = 6

    // Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. If AUTO_ACK is acti-
    // vated, this bit is set high only when ACK is received.
    // Write 1 to clear bit.
    , TX_DS = 5

    // Maximum number of TX retransmits interrupt Write 1 to clear bit.
    // If MAX_RT is asserted it must be cleared to enable further communication.
    , MAX_RT = 4

    , TX_FULL = 0
  } Status;

  typedef enum {
    // 250us
    ARD_250 = 0
    // 500 us
    , ARD_500 = 0x10
    // 750 us
    , ARD_750 = 0x20
    // ... 0xF0 = 4000us

    , ARC_15 = 0x0F
  } Setup_Retry;

  typedef enum {
    RX_ADDR_P0_B0_DEFAULT_VAL = 0x01
    , RX_ADDR_P1_B0_DEFAULT_VAL = 0x02
    , RX_ADDR_P2_DEFAULT_VAL = 0x03
    , RX_ADDR_P3_DEFAULT_VAL = 0x04
    , RX_ADDR_P4_DEFAULT_VAL = 0x05
    , RX_ADDR_P5_DEFAULT_VAL = 0x06
  } RxAddressDefaultValues;

  void write_register(const Register& reg, const uint8_t* value, const uint8_t len)
  {
    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(W_REGISTER | (REGISTER_MASK & reg));
    m_comm.SPI_TRANSMIT_SYNC(value, len);
    m_comm.CS_HIGH();
  }

  void write_config_register(const Register& reg, const uint8_t value)
  {
#ifdef LOGGING
    *m_logger << "[N] write_config_register(" << reg << ", " << value << ")\n";
#endif
    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(W_REGISTER | (REGISTER_MASK & reg));
    m_comm.SPI_SHIFT(value);
    m_comm.CS_HIGH();
  }

  void irqHandler()
  {
    m_comm.CS_LOW();        // CHIP-SELECT
    uint8_t status = m_comm.SPI_SHIFT(NOP);  // Write NOP but read status
    m_comm.CS_HIGH();       // UNSELECT

#ifdef LOGGING
    *m_logger << "[I] Status = " << status << "\n";
#endif

    if (status & (1 << TX_DS))
    {
#ifdef LOGGING
      *m_logger << "[I] Clear TX_DS\n";
#endif
      write_config_register(STATUS, (1 << TX_DS)); // Clear TX_DS
      m_ptx = 0;
    }

    if (status & (1 << MAX_RT))
    {
#ifdef LOGGING
      *m_logger << "[I] Clear MAX_RT and start retransmission\n";
#endif
      write_config_register(STATUS, (1 << MAX_RT)); // Clear MAX_RT

      // Start retransmission
      /* m_comm.CE_HIGH();
      m_sys.delay_us(10);
      m_comm.CE_LOW(); */
    }

    if (status & (1 << TX_FULL))
    {
#ifdef LOGGING
      *m_logger << "[I] Flush TX buffer\n";
#endif
      flushTX();
    }
  }

  inline void tx_powerup()
  {
    write_config_register(CONFIG, m_config | (1 << PWR_UP) | (0 << PRIM_RX));
  }

  void rx_powerup()
  {
    write_config_register(CONFIG, m_config | (1 << PWR_UP) | (1 << PRIM_RX));
  }

  void flushTX()
  {
    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(FLUSH_TX);
    m_comm.CS_HIGH();
  }

  void flushRX()
  {
    m_comm.CS_LOW();
    m_comm.SPI_SHIFT(FLUSH_TX);
    m_comm.CS_HIGH();
  }
};

