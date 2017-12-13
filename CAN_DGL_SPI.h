/* 
 * File:   CAN_DGL_SPI.h
 * Author: dad
 *
 * Created on April 17, 2016, 8:07 AM
 */

#ifndef CAN_DGL_SPI_H
#define	CAN_DGL_SPI_H

#include <SPI.h>


#define CAN_16MZ_500KBPS  16

#define CAN_16MZ_1000KBPS 18

#define CAN_8MZ_500KBPS 19

#include "MCP_2515_CAN_RX_Msg.h"


class CAN_DGL_mcp_2515_SPI {
public:
    // Construct with pin for selecting this remote SPI device. 
    // The default pin is SS 10. If you use a different pin for chip 
    // select the SS pin (10) is still must be used for the MASTER SPI 
    CAN_DGL_mcp_2515_SPI(uint8_t chipSelectPin = SS); 
    
    
    
    bool begin(uint8_t speed); // crystal oscillator speed on CAN board
    
    // Configure to use interrupt pin
    void useInterrupt(uint8_t interrupt_number = 0);
    
    // Set Masks and filters 
    
    enum TX_BUFFER{ TX_BUF_0 =0, TX_BUF_1, TX_BUF_2};
    
    // Set standard CAN PID requests in one of the three TX buffers
    bool set_CAN_std_PID_request_TX_buffer(TX_BUFFER tx_buf_number, uint8_t mode, byte PID){return false;};
     
    // Transmit the message in one of the three TX buffers
    bool transmit_TX_buffer(TX_BUFFER tx_buf_number){return false;};
    
    //Check if there is data in the buffers 
    // add read buffers if data is available 
    bool tryRead(MCP_2515_CAN_RX_Msg& msg); 

    byte get_status_byte(void){return m_status_bits;};
    byte get_rx_status_byte(void){return m_rx_status_bits;};
    
    const char *last_error_msg(void);
    


private:
    static void isr(void);
    
    inline void select(void){digitalWrite(m_chipSelectPin, LOW);} 
    inline void unSelect(void){digitalWrite(m_chipSelectPin, HIGH);} 
    
    void reset_mcp(void);
    void setRegister(const byte address, const byte value);
    uint8_t readRegister(const byte address);
    void modifyRegister(const byte address, const byte mask, const byte changed_bits);
    
    // clear mask 0 or 1
    void clearMask(uint8_t mask_number);
    
    void readStatus(void); // called from ISR
    byte m_status_bits;
    void readRxStatus(void);
    
    #define READ_RX_BUFFER_AT_RXB0SIDH  B10010000 | B0000 
    #define READ_RX_BUFFER_AT_RXB0D0    B10010000 | B0010 
    #define READ_RX_BUFFER_AT_RXB1SIDH  B10010000 | B0100     
    #define READ_RX_BUFFER_AT_RXB1D0    B10010000 | B0110   
    
    uint8_t readRXBuffer(byte read_rx_at_cmd, MCP_2515_CAN_RX_Msg& msg);
    
    void configure_rate(uint8_t canSpeed);
    bool configure_receive_buffers(void);
    
        
    #define MCP_MODE_NORMAL     B00000000 // = Normal Operation mode
    #define MCP_MODE_SLEEP      B00100000 // = Sleep mode
    #define MCP_MODE_LOOPBACK   B01000000 // = Loopback mode
    #define MCP_MODE_LISTENONLY B01100000 // = Listen-Only mode
    #define MCP_MODE_CONFIG     B10000000 // = Configuration mode
    #define MCP_MODE_MASK       B11100000 /// 0xE0 Mode Mask

    bool set_mcp_mode(byte mode);

    // Status Bits
    #define  CANINTF_RX0IF   0x01
    #define  CANINTF_RX1IF   0x02
    #define  TXB0CNTRL_TXREQ 0x03
    #define  CANINTF_TX0IF   0x04
    #define  TXB1CNTRL_TXREQ 0x05
    #define  CANINTF_TX1IF   0x06
    #define  TXB2CNTRL_TXREQ 0x07
    #define  CANINTF_TX2IF   0x08
    
    
    SPISettings m_can_spi_settings;
    uint8_t m_chipSelectPin;
    
    
    byte m_rx_status_bits;
    
   
    bool m_check_buffer_0_first;
    bool m_use_interrupt;
    
    const char* m_last_error;

    volatile static bool m_interrupt_called;
};

#endif	/* CAN_DGL_SPI_H */

