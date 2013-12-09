#pragma once

#include "msp430lib/sys/v1/ISys.hpp"

#include "RF24.h"

#ifdef LOGGING
#include <Logger.hpp>
#endif

#include <endian.h>

#ifndef _BV
#define _BV(x) (1 << x)
#endif

namespace NRF24L01 {
inline namespace v1 {

  template <typename T>
  const T& min(const T& a, const T& b)
  {
    return a < b ? a : b;
  }

  /*! \brief Interface of the RF24 driver provided by maniacbug on GITHUB.
   *
   * Interface of RF24 driver by maniacbug. This is just his interface
   * and there is another adapter class which adapts any NRF24 driver
   * based implementation to it.
   *
   * Based on: https://github.com/maniacbug/RF24/blob/master/RF24.h
   * which is Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
   */
  template<typename COMM>
  class NRF24L01
  {
    friend COMM;

    COMM              m_comm;
    msp430lib::ISys&  m_sys;
#ifdef LOGGING
    ILogger*          m_logger;
#endif

    uint8_t           m_payload_size;
    bool              m_wide_band;
    bool              m_p_variant;
    bool              m_ack_payload_available;
    uint8_t           m_ack_payload_length;
    bool              m_dynamic_payloads_enabled;
    uint64_t          m_pipe0_reading_address;

  public:
    NRF24L01(msp430lib::ISys& sys)
      : m_sys(sys)
      , m_payload_size(32)
      , m_wide_band(true)
      , m_p_variant(false)
      , m_ack_payload_available(false)
      , m_ack_payload_length(0)
      , m_dynamic_payloads_enabled(false)
      , m_pipe0_reading_address(0)
    {
    }

    void begin(void)
    {
      m_sys.init();
      m_comm.init(this);

      m_comm.CE_LOW();
      m_comm.CS_HIGH();

      // Must allow the radio time to settle else configuration bits will not necessarily stick.
      // This is actually only required following power up but some settling time also appears to
      // be required after resets too. For full coverage, we'll always assume the worst.
      // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
      // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
      // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
      m_sys.delay_us(5000);

      // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
      // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
      // sizes must never be used. See documentation for a more complete explanation.
      write_register(SETUP_RETR, (0x04 << 4) | (0x0F << 0));

      // Restore our default PA level
      setPALevel(RF24_PA_MAX);

      // Determine if this is a p or non-p RF24 module and then
      // reset our data rate back to default value. This works
      // because a non-P variant won't allow the data rate to
      // be set to 250Kbps.
      if(setDataRate(RF24_250KBPS))
      {
        m_p_variant = true ;
      }

      // Then set the data rate to the slowest (and most reliable) speed supported by all
      // hardware.
      setDataRate(RF24_1MBPS) ;

      // Initialize CRC and request 2-byte (16bit) CRC
      setCRCLength(RF24_CRC_16) ;

      // Disable dynamic payloads, to match dynamic_payloads_enabled setting
      write_register(DYNPD,0);

      // Reset current status
      // Notice reset and flush is the last thing we do
      write_register(STATUS, (1 << RX_DR) | (1 << TX_DS) || (1 << MAX_RT));

      // Set up default configuration.  Callers can always change it later.
      // This channel should be universally safe and not bleed over into adjacent
      // spectrum.
      setChannel(76);

      // Flush buffers
      flush_rx();
      flush_tx();
    }

    //! Start Listening on the pipes opened for reading
    void startListening(void)
    {
      write_register(CONFIG, read_register(CONFIG) | (1 << PWR_UP) | (1 << PRIM_RX));
      write_register(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

      // Restore the pipe0 adddress, if exists
      if (m_pipe0_reading_address)
        write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&m_pipe0_reading_address), 5);

      // Flush buffers
      flush_rx();
      flush_tx();

      // Go!
      m_comm.CE_HIGH();

      // wait for the radio to come up (130us actually only needed)
      m_sys.delay_us(130);
    }

    //! Stop Listening on the pipes
    void stopListening(void)
    {
      m_comm.CE_LOW();
      flush_tx();
      flush_rx();
    }

    bool write(const void* buf, uint8_t len)
    {
      bool result = false;

      // Begin the write
      startWrite(buf, len);

      // ------------
      // At this point we could return from a non-blocking write, and then call
      // the rest after an interrupt

      // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
      // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
      // is flaky and we get neither.

      // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
      // if I tighted up the retry logic.  (Default settings will be 1500us.
      // Monitor the send
      uint8_t observe_tx;
      uint8_t status;
      uint32_t sent_at = 0; // millis
      const uint32_t timeout = 500; //ms to wait for timeout
      do
      {
        status = read_register(OBSERVE_TX, &observe_tx, 1);
      }
      while(!(status & ((1 << TX_DS) | (1 << MAX_RT)))); // && (  < timeout ) );

      // The part above is what you could recreate with your own interrupt handler,
      // and then call this when you got an interrupt
      // ------------

      // Call this when you get an interrupt
      // The status tells us three things
      // * The send was successful (TX_DS)
      // * The send failed, too many retries (MAX_RT)
      // * There is an ack packet waiting (RX_DR)
      bool tx_ok, tx_fail;
      whatHappened(tx_ok,tx_fail,m_ack_payload_available);

      result = tx_ok;

      // Handle the ack packet
      if (m_ack_payload_available)
      {
        m_ack_payload_length = getDynamicPayloadSize();
      }

      // Yay, we are done.

      // Power down
      powerDown();

      // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
      flush_tx();

      return result;
    }

    //! Read the status register to check if data is available for reading
    bool available(void)
    {
      uint8_t status = get_status();
      return status & (1 << RX_DR);
    }

    //! Read available data into buffer at buf (of size len)
    bool read(void* buf, uint8_t len)
    {
      // Fetch the payload
      read_payload( buf, len );

      // was this the last of the data available?
      return read_register(FIFO_STATUS) & _BV(RX_EMPTY);
    }

    void openWritingPipe(uint64_t address)
    {
#if __BYTE_ORDER == __LITTLE_ENDIAN
      // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
      // expects it LSB first too, so we're good.

      write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&address), 5);
      write_register(TX_ADDR, reinterpret_cast<const uint8_t*>(&address), 5);

      const uint8_t max_payload_size = 32;
      write_register(RX_PW_P0, min(m_payload_size, max_payload_size));
#else
#error Not yet implemented for BIG_ENDIAN
#endif
    }

    void openReadingPipe(uint8_t child, uint64_t address)
    {
#if __BYTE_ORDER == __LITTLE_ENDIAN
      // If this is pipe 0, cache the address.  This is needed because
      // openWritingPipe() will overwrite the pipe 0 address, so
      // startListening() will have to restore it.
      if (child == 0)
      {
        m_pipe0_reading_address = address;
      }

      if (child <= 6)
      {
        // For pipes 2-5, only write the LSB
        if ( child < 2 )
        {
          write_register(static_cast<const Register>(static_cast<const uint8_t>(RX_ADDR_P0) + child),
            reinterpret_cast<const uint8_t*>(&address), 5);
        }
        else
        {
          write_register(static_cast<const Register>(static_cast<const uint8_t>(RX_ADDR_P0) + child),
            reinterpret_cast<const uint8_t*>(&address), 1);
        }

        write_register(static_cast<const Register>(static_cast<const uint8_t>(RX_PW_P0) + child),
          m_payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(EN_RXADDR, read_register(EN_RXADDR) | (1 << child));
      }
#else
#error Not yet implemented for BIG_ENDIAN
#endif
    }

    void setRetries(uint8_t delay, uint8_t count)
    {
      write_register(SETUP_RETR,(delay&0xf)<<4 | (count&0xf)<<0);
    }

    void setChannel(uint8_t channel)
    {
      const uint8_t max_channel = 127;
      write_register(RF_CH,min(channel,max_channel));
    }

    void setPayloadSize(uint8_t size)
    {
      const uint8_t max_payload_size = 32;
      m_payload_size = min(size,max_payload_size);
    }

    uint8_t getPayloadSize(void)
    {
      return 32;
    }

    uint8_t getDynamicPayloadSize(void)
    {
      uint8_t result = 0;

      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(R_RX_PL_WID);
      result = m_comm.SPI_SHIFT(NOP);
      m_comm.CS_HIGH();

      return result;
    }

    void enableAckPayload(void)
    {
      //
      // enable ack payload and dynamic payload features
      //
      write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

      // If it didn't work, the features are not enabled
      if ( ! read_register(FEATURE) )
      {
        // So enable them and try again
        toggle_features();
        write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
      }

      //
      // Enable dynamic payload on pipes 0 & 1
      //
      write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
    }

    void enableDynamicPayloads(void)
    {
      // Enable dynamic payload throughout the system
      write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );

      // If it didn't work, the features are not enabled
      if ( ! read_register(FEATURE) )
      {
        // So enable them and try again
        toggle_features();
        write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );
      }

      // Enable dynamic payload on all pipes
      //
      // Not sure the use case of only having dynamic payload on certain
      // pipes, so the library does not support it.
      write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

      m_dynamic_payloads_enabled = true;
    }

    bool isPVariant(void)
    {
      return m_p_variant;
    }

    void setAutoAck(bool enable)
    {
      write_register(EN_AA, enable ? 0x3F : 0x00);
    }

    void setAutoAck(uint8_t pipe, bool enable)
    {
      uint8_t aa = read_register(EN_AA);
      if (enable)
      {
        aa |= _BV(pipe);
      }
      else
      {
        aa &= ~(_BV(pipe));
      }

      write_register(EN_AA, aa);
    }

    void setPALevel(rf24_pa_dbm_e level)
    {
      uint8_t setup = read_register(RF_SETUP) ;
      setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

      // switch uses RAM (evil!)
      if ( level == RF24_PA_MAX )
      {
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
      }
      else if ( level == RF24_PA_HIGH )
      {
        setup |= _BV(RF_PWR_HIGH) ;
      }
      else if ( level == RF24_PA_LOW )
      {
        setup |= _BV(RF_PWR_LOW);
      }
      else if ( level == RF24_PA_MIN )
      {
        // nothing
      }
      else if ( level == RF24_PA_ERROR )
      {
        // On error, go to maximum PA
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
      }

      write_register( RF_SETUP, setup ) ;
    }

    rf24_pa_dbm_e getPALevel(void)
    {
      rf24_pa_dbm_e result = RF24_PA_ERROR ;
      uint8_t power = read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

      // switch uses RAM (evil!)
      if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
      {
        result = RF24_PA_MAX ;
      }
      else if ( power == _BV(RF_PWR_HIGH) )
      {
        result = RF24_PA_HIGH ;
      }
      else if ( power == _BV(RF_PWR_LOW) )
      {
        result = RF24_PA_LOW ;
      }
      else
      {
        result = RF24_PA_MIN ;
      }

      return result ;
    }

    bool setDataRate(rf24_datarate_e speed)
    {
      bool result = false;
      uint8_t setup = read_register(RF_SETUP) ;

      // HIGH and LOW '00' is 1Mbs - our default
      m_wide_band = false ;
      setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
      if( speed == RF24_250KBPS )
      {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        m_wide_band = false ;
        setup |= _BV( RF_DR_LOW ) ;
      }
      else
      {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if ( speed == RF24_2MBPS )
        {
          m_wide_band = true ;
          setup |= _BV(RF_DR_HIGH);
        }
        else
        {
          // 1Mbs
          m_wide_band = false ;
        }
      }
      write_register(RF_SETUP,setup);

      // Verify our result
      if ( read_register(RF_SETUP) == setup )
      {
        result = true;
      }
      else
      {
        m_wide_band = false;
      }

      return result;
    }

    rf24_datarate_e getDataRate(void)
    {
      rf24_datarate_e result ;
      uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

      // switch uses RAM (evil!)
      // Order matters in our case below
      if ( dr == _BV(RF_DR_LOW) )
      {
        // '10' = 250KBPS
        result = RF24_250KBPS ;
      }
      else if ( dr == _BV(RF_DR_HIGH) )
      {
        // '01' = 2MBPS
        result = RF24_2MBPS ;
      }
      else
      {
        // '00' = 1MBPS
        result = RF24_1MBPS ;
      }
      return result ;
    }

    void setCRCLength(rf24_crclength_e length)
    {
      uint8_t config = read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

      // switch uses RAM (evil!)
      if ( length == RF24_CRC_DISABLED )
      {
        // Do nothing, we turned it off above. 
      }
      else if ( length == RF24_CRC_8 )
      {
        config |= _BV(EN_CRC);
      }
      else
      {
        config |= _BV(EN_CRC);
        config |= _BV( CRCO );
      }
      write_register( CONFIG, config ) ;
    }

    rf24_crclength_e getCRCLength(void)
    {
      rf24_crclength_e result = RF24_CRC_DISABLED;
      uint8_t config = read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

      if ( config & _BV(EN_CRC ) )
      {
        if ( config & _BV(CRCO) )
          result = RF24_CRC_16;
        else
          result = RF24_CRC_8;
      }

      return result;
    }

    void disableCRC(void)
    {
      write_register( CONFIG, read_register(CONFIG) & ~_BV(EN_CRC) ) ;
    }

    void powerUp(void)
    {
      write_register(CONFIG, read_register(CONFIG) | (1 << PWR_UP));
    }

    void powerDown(void)
    {
      write_register(CONFIG, read_register(CONFIG) & ~(1 << PWR_UP));
    }

    bool available(uint8_t& pipe_num)
    {
      uint8_t status = get_status();
      bool result = (status & (1 << RX_DR));

      if (result)
      {
        // If the caller wants the pipe number, include that
        pipe_num = (status >> RX_P_NO) & 0x07;

        // Clear the status bit

        // ??? Should this REALLY be cleared now?  Or wait until we
        // actually READ the payload?

        write_register(STATUS, (1 << RX_DR));

        // Handle ack payload receipt
        if (status & (1 << TX_DS))
        {
          write_register(STATUS, (1 << TX_DS));
        }
      }

      return result;
    }

    void startWrite(const void* buf, uint8_t len)
    {
      // Transmitter power-up
      write_register(CONFIG, ( read_register(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
      m_sys.delay_us(150);

      // Send the payload
      write_payload(buf, len);

      // Allons!
      m_comm.CE_HIGH();
      m_sys.delay_us(15);
      m_comm.CE_LOW();
    }

    void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
    {
      const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

      m_comm.CS_LOW();
      m_comm.SPI_SHIFT( W_ACK_PAYLOAD | ( pipe & 0x07 ) );
      const uint8_t max_payload_size = 32;
      uint8_t data_len = min(len,max_payload_size);
      while ( data_len-- )
        m_comm.SPI_SHIFT(*current++);

      m_comm.CS_HIGH();
    }

    bool isAckPayloadAvailable(void)
    {
      bool result = m_ack_payload_available;
      m_ack_payload_available = false;
      return result;
    }

    void whatHappened(bool& tx_ok, bool& tx_fail, bool& rx_ready)
    {
      // Read the status & reset the status in one easy call
      // Or is that such a good idea?
      uint8_t status = write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

      // Report to the user what happened
      tx_ok = status & _BV(TX_DS);
      tx_fail = status & _BV(MAX_RT);
      rx_ready = status & _BV(RX_DR);
    }

    bool testCarrier(void)
    {
      return ( read_register(CD) & 1 );
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
      RX_EMPTY = 0
      , RX_FULL = 1
      , TX_EMPTY = 4
      , TX_FULL = 5
      , TX_REUSE = 6
    } Fifo_Status;

    typedef enum {
      DPL_P0 = 0
      , DPL_P1 = 1
      , DPL_P2 = 2
      , DPL_P3 = 3
      , DPL_P4 = 4
      , DPL_P5 = 5
    } DynPd;

    typedef enum {
      EN_DYN_ACK = 0
      , EN_ACK_PAY = 1
      , EN_DPL = 2
    } Feature;

    typedef enum {
      LNA_HCURR = 0
      , RF_PWR_LOW = 1
      , RF_PWR_HIGH = 2
      , RF_DR_HIGH = 3
      , PLL_LOCK = 4
      , RF_DR_LOW = 5
    } Rf_Setup;

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

      , RX_P_NO = 1
      , STATUS_TX_FULL = 0
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

    void toggle_features(void)
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT( ACTIVATE );
      m_comm.SPI_SHIFT( 0x73 );
      m_comm.CS_HIGH();
    }

    uint8_t read_register(const Register& reg, uint8_t* buf, uint8_t len)
    {
      uint8_t status;

      m_comm.CS_LOW();
      status = m_comm.SPI_SHIFT(R_REGISTER | (REGISTER_MASK & reg));
      while (len--)
        *buf++ = m_comm.SPI_SHIFT(NOP);
      m_comm.CS_HIGH();

      return status;
    }

    uint8_t read_register(const Register& reg)
    {
      uint8_t ret;

      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(R_REGISTER | (REGISTER_MASK & reg));
      ret = m_comm.SPI_SHIFT(NOP);
      m_comm.CS_HIGH();

      return ret;
    }

    uint8_t write_register(const Register& reg, const uint8_t* value, const uint8_t len)
    {
      m_comm.CS_LOW();
      uint8_t status = m_comm.SPI_SHIFT(W_REGISTER | (REGISTER_MASK & reg));
      m_comm.SPI_TRANSMIT_SYNC(value, len);
      m_comm.CS_HIGH();

      return status;
    }

    uint8_t write_register(const Register& reg, const uint8_t value)
    {
      m_comm.CS_LOW();
      uint8_t status = m_comm.SPI_SHIFT(W_REGISTER | (REGISTER_MASK & reg));
      m_comm.SPI_SHIFT(value);
      m_comm.CS_HIGH();

      return status;
    }

    uint8_t get_status()
    {
      uint8_t status;

      m_comm.CS_LOW();
      status = m_comm.SPI_SHIFT(NOP);
      m_comm.CS_HIGH();

      return status;
    }

    uint8_t read_payload(void* buf, uint8_t len)
    {
      uint8_t status;
      uint8_t* current = reinterpret_cast<uint8_t*>(buf);

      uint8_t data_len = min(len,m_payload_size);
      uint8_t blank_len = m_dynamic_payloads_enabled ? 0 : m_payload_size - data_len;

      m_comm.CS_LOW();
      status = m_comm.SPI_SHIFT(R_RX_PAYLOAD);
      // XXX: Optimize?
      while ( data_len-- )
        *current++ = m_comm.SPI_SHIFT(NOP);
      while ( blank_len-- )
        m_comm.SPI_SHIFT(NOP);
      m_comm.CS_HIGH();

      return status;
    }

    uint8_t write_payload(const void* buf, uint8_t len)
    {
      uint8_t status;
      const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

      uint8_t data_len = min(len, m_payload_size);
      uint8_t blank_len = m_dynamic_payloads_enabled ? 0 : m_payload_size - data_len;

      m_comm.CS_LOW();
      status = m_comm.SPI_SHIFT(W_TX_PAYLOAD);
      m_comm.SPI_TRANSMIT_SYNC(reinterpret_cast<const uint8_t*>(buf), data_len);

      while ( blank_len-- )
        m_comm.SPI_SHIFT(0x00);

      m_comm.CS_HIGH();

      return status;
    }

    void irqHandler()
    {
      // TODO: Implement me
    }

    void flush_tx()
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(FLUSH_TX);
      m_comm.CS_HIGH();
    }

    void flush_rx()
    {
      m_comm.CS_LOW();
      m_comm.SPI_SHIFT(FLUSH_RX);
      m_comm.CS_HIGH();
    }
};
}
}

