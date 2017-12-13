/* 
 * File:   CAN_DGL_SPI.cpp
 * Author: dad
 * 
 * Created on April 17, 2016, 8:07 AM
 */

#include "CAN_DGL_SPI.h"

//#include <Speed-Studio_CAN_BUS_Shield/mcp_can_dfs.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

volatile bool CAN_DGL_mcp_2515_SPI::m_interrupt_called = false;

// interrupt Service rutine 
void CAN_DGL_mcp_2515_SPI::isr(void){
   CAN_DGL_mcp_2515_SPI::m_interrupt_called = !CAN_DGL_mcp_2515_SPI::m_interrupt_called;
}

void CAN_DGL_mcp_2515_SPI::useInterrupt(uint8_t interrupt_number){
    m_use_interrupt = true;
    attachInterrupt(interrupt_number, CAN_DGL_mcp_2515_SPI::isr, CHANGE); 
}


CAN_DGL_mcp_2515_SPI::CAN_DGL_mcp_2515_SPI(uint8_t chipSelectPin) :
    m_can_spi_settings(4000000, MSBFIRST, SPI_MODE0),
    m_chipSelectPin(chipSelectPin),
    m_rx_status_bits(0x0),
    m_check_buffer_0_first(true),
    m_use_interrupt(false),
    m_last_error(NULL)
{
    pinMode(m_chipSelectPin, OUTPUT);
    unSelect();  
}

//CAN_DGL_mcp_2515_SPI::~CAN_DGL_mcp_2515_SPI() {
//}


const char * CAN_DGL_mcp_2515_SPI::last_error_msg(void){
    if( m_last_error == NULL){
        m_last_error = "no_error";
    }
    return  m_last_error;
       
}
void CAN_DGL_mcp_2515_SPI::reset_mcp(void){
    // mcp2515_reset
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
    SPI.transfer(MCP_RESET);
    unSelect();  
    SPIClass::endTransaction();
    delay(10);     
}


void CAN_DGL_mcp_2515_SPI::setRegister(const byte address, const byte value){
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
    SPI.transfer(MCP_WRITE);
    SPI.transfer(address);
    SPI.transfer(value);
    unSelect();  
    SPIClass::endTransaction();
}

uint8_t CAN_DGL_mcp_2515_SPI::readRegister(const byte address){
    uint8_t ret = 0;

    SPIClass::beginTransaction(m_can_spi_settings);
    select();
    
    SPI.transfer(MCP_READ);
    SPI.transfer(address);
    ret = SPI.transfer(0x0);
    
    unSelect();  
    SPIClass::endTransaction();

    return ret;
}

// clear Mask. Accept all messages
void CAN_DGL_mcp_2515_SPI::clearMask(uint8_t mask_number){
    
    byte mask_register = 0xFF;
    if(mask_number == 0){
        mask_register = MCP_RXM0SIDH;
    }else if (mask_number == 0){
        mask_register = MCP_RXM1SIDH;
    }else{
        return; // not a valid mask number 
    }
    
    // If we are not in configuration set to configuration mode
    byte mode = (readRegister(MCP_CANCTRL) & MCP_MODE_MASK);
    if( mode != MCP_MODE_CONFIG ){
        set_mcp_mode(MCP_MODE_CONFIG);
    }
    
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
    SPI.transfer(MCP_WRITE);
    if( mask_number == 0 ){
        SPI.transfer(mask_register);
    }
    for(uint8_t i = 0; i< 4 ; ++i){
        SPI.transfer(0x0);
    }
    unSelect();  
    SPIClass::endTransaction();
    
    // If we were not in configuration mode put back to pervious mode
    if( mode != MCP_MODE_CONFIG ){
        set_mcp_mode(mode);
    }
    
}

void CAN_DGL_mcp_2515_SPI::readStatus(void){

    m_status_bits = 0;
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
	SPI.transfer(MCP_READ_STATUS);
	m_status_bits = SPI.transfer(0x0);
    unSelect();  
    SPIClass::endTransaction();
}

void CAN_DGL_mcp_2515_SPI::readRxStatus(void){

    m_rx_status_bits = 0;
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
	SPI.transfer(MCP_RX_STATUS);
	m_rx_status_bits = SPI.transfer(0x0);
    unSelect();  
    SPIClass::endTransaction();
}

void CAN_DGL_mcp_2515_SPI::modifyRegister(const byte address, const byte mask, const byte changed_bits){
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
    SPI.transfer(MCP_BITMOD);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(changed_bits);
    unSelect();  
    SPIClass::endTransaction();  
}


void CAN_DGL_mcp_2515_SPI::configure_rate(uint8_t canSpeed){
  // configure mcp 2515 rate 
    uint8_t  cfg1, cfg2, cfg3;

    switch (canSpeed)
    {
     case CAN_500KBPS:
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
                
     case CAN_8MZ_500KBPS:
        cfg1 = MCP_8MHz_500kBPS_CFG1;
        cfg2 = MCP_8MHz_500kBPS_CFG2;
        cfg3 = MCP_8MHz_500kBPS_CFG3;
        break;                  
     default:
        // INVALID CONFIGURURATION 
        m_last_error = "INVALID CONFIG";
        return;   
    }    
    
    setRegister(MCP_CNF1, cfg1);
    setRegister(MCP_CNF2, cfg2);
    setRegister(MCP_CNF3, cfg3);
    delay(10);
}


bool CAN_DGL_mcp_2515_SPI::configure_receive_buffers(void){
    
    // Init CANBuffers TX buffers to Zero
    byte a1, a2, a3;

    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (uint8_t i = 0; i < 14; i++) { 
        setRegister(a1, 0);
        setRegister(a2, 0);
        setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    
    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);    
    
    clearMask(0);
    clearMask(1);
    
   
    /*
        bit 7 Unimplemented: Read as ‘0’
        bit 6-5 RXM<1:0>: Receive Buffer Operating mode bits
            11 = Turn mask/filters off; receive any message
            10 = Receive only valid messages with extended identifiers that meet filter criteria
            01 = Receive only valid messages with standard identifiers that meet filter criteria. Extended ID filter
                 registers RXFnEID8:RXFnEID0 are ignored for the messages with standard IDs.
            00 = Receive all valid messages using either standard or extended identifiers that meet filter criteria.
                 Extended ID filter registers RXFnEID8:RXFnEID0 are applied to first two bytes of data in the
                 messages with standard IDs.
        bit 4 Unimplemented: Read as ‘0’
        bit 3 RXRTR: Received Remote Transfer Request bit
            1 = Remote Transfer Request Received
            0 = No Remote Transfer Request Received
        bit 2 BUKT: Rollover Enable bit
            1 = RXB0 message will rollover and be written to RXB1 if RXB0 is full
            0 = Rollover disabled
        bit 1 BUKT1: Read-only Copy of BUKT bit (used internally by the MCP2515)
        bit 0 FILHIT0: Filter Hit bit – indicates which acceptance filter enabled reception of message
            1 = Acceptance Filter 1 (RXF1)
            0 = Acceptance Filter 0 (RXF0)
        Note:
            If a rollover from RXB0 to RXB1 occurs, the FILHIT bit will reflect the filter that accepted
            the message that rolled over.     
     */
#define MCP_RXB_MODE_MASK    B01100000    // 6-5 RXM<1:0>: Receive Buffer Operating mode bits
#define MCP_RXB_RXRTR_MASK   B00001000    // RXRTR Received Remote Transfer Request bit
#ifndef MCP_RXB_BUKT_MASK
    #define MCP_RXB_BUKT_MASK    B00000100    // BUKT: Rollover Enable bit
#endif
#define MCP_RXB_FILHIT0_MASK B00000001    // FILHIT0: Filter Hit bit – indicates which acceptance filter enabled reception of message    
    
    /* enable both receive-buffers  
       to receive messages          
       with std. and ext. identifiers                           
       and enable rollover          */    
     modifyRegister(MCP_RXB0CTRL,
              MCP_RXB_MODE_MASK|MCP_RXB_BUKT_MASK,
             MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
     
     // Verify register was set
     if( (readRegister(MCP_RXB0CTRL) & (MCP_RXB_MODE_MASK|MCP_RXB_BUKT_MASK)) != (MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK) ){
         m_last_error = "set MCP_RXB0CTRL";
         return false;
     }
    

  //RXB1CTRL – RECEIVE BUFFER 1 CONTROL
     /*
        bit 7 Unimplemented: Read as ‘0’
        bit 6-5 RXM<1:0>: Receive Buffer Operating mode bits
            11 = Turn mask/filters off; receive any message
            10 = Receive only valid messages with extended identifiers that meet filter criteria
            01 = Receive only valid messages with standard identifiers that meet filter criteria
            00 = Receive all valid messages using either standard or extended identifiers that meet filter criteria
        bit 4 Unimplemented: Read as ‘0’
        bit 3 RXRTR: Received Remote Transfer Request bit
            1 = Remote Transfer Request Received
            0 = No Remote Transfer Request Received
        bit 2-0 FILHIT<2:0>: Filter Hit bits - indicates which acceptance filter enabled reception of message
            101 = Acceptance Filter 5 (RXF5)
            100 = Acceptance Filter 4 (RXF4)
            011 = Acceptance Filter 3 (RXF3)
            010 = Acceptance Filter 2 (RXF2)
            001 = Acceptance Filter 1 (RXF1) (Only if BUKT bit set in RXB0CTRL)
            000 = Acceptance Filter 0 (RXF0) (Only if BUKT bit set in RXB0CTRL)     
     */
     
     
     modifyRegister(MCP_RXB1CTRL,
                    MCP_RXB_MODE_MASK,
                    MCP_RXB_RX_STDEXT);        
     
     if( (readRegister(MCP_RXB1CTRL) & MCP_RXB_MODE_MASK)  != MCP_RXB_RX_STDEXT ){
         m_last_error = "set MCP_RXB1CTRL";
         return false;
     }
    
    
     return true;
}


bool CAN_DGL_mcp_2515_SPI::set_mcp_mode(byte mode){
    modifyRegister(MCP_CANCTRL, MCP_MODE_MASK, mode);
    if( (readRegister(MCP_CANCTRL) & MCP_MODE_MASK) == mode){
        return true;
    }else{
        m_last_error = "Setting MCP_CANCTRL";
        return false;
    }
}



bool CAN_DGL_mcp_2515_SPI::begin(byte canSpeed){
    // initialize SPI LIB and this board as the master
    pinMode(2, INPUT); // Setting pin 2 for /INT input
    SPI.begin();
    
    reset_mcp();
    
    // Enter Configure mode
    if(! set_mcp_mode(MCP_MODE_CONFIG) ){
        return false;
    }

    configure_rate(canSpeed);

    if(!configure_receive_buffers()){
        return false;
    }

    //interrupt mode          
    setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);      
    
    
    // enter normal mode        
    if(!set_mcp_mode(MCP_MODE_NORMAL)){
        return false;
    }
    return true;
}




uint8_t CAN_DGL_mcp_2515_SPI::readRXBuffer(byte read_rx_at_cmd, MCP_2515_CAN_RX_Msg& msg){
    
    SPIClass::beginTransaction(m_can_spi_settings);
    select();
	SPI.transfer(read_rx_at_cmd);    
    
    msg.m_RXBnSIDH = SPI.transfer(0x0);
    msg.m_RXBnSIDL = SPI.transfer(0x0);
    msg.m_RXBnEID8 = SPI.transfer(0x0);
    msg.m_RXBnEID0 = SPI.transfer(0x0);
    msg.m_RXBnDLC  = SPI.transfer(0x0);
    
    uint8_t msg_size =0;
    msg_size  = (msg.m_RXBnDLC & B00001111);
    for (uint8_t i=0; i<msg_size && i<CAN_MAX_CHAR_IN_MESSAGE; i++) {
		msg.m_buf[i] = SPI.transfer(0x0);
	}
    unSelect();  
    SPIClass::endTransaction();   

    return msg_size;
}



bool CAN_DGL_mcp_2515_SPI::tryRead(MCP_2515_CAN_RX_Msg& msg){
    bool ret_value = false;
    
    if(m_use_interrupt && ! CAN_DGL_mcp_2515_SPI::m_interrupt_called){
        return ret_value;
    }

    m_last_error = "-";

    readRxStatus();

    // Read buffer0 first unless buffer0 was last read and buffer1 one has data
    if ( m_rx_status_bits & B01000000 && (m_check_buffer_0_first || (!m_check_buffer_0_first && !(m_rx_status_bits & B10000000)) )   ){ //Message in RXB0
        m_check_buffer_0_first = false;
        if(readRXBuffer(READ_RX_BUFFER_AT_RXB0SIDH,msg) > 0 ){
            ret_value = true;
        }else{
            m_last_error = "no msg0";
        }
    }else if( m_rx_status_bits & B10000000 ){// Message in RXB1
        m_check_buffer_0_first = true;
        if(readRXBuffer(READ_RX_BUFFER_AT_RXB1SIDH,msg) > 0 ){
            ret_value = true;
        }else{
            m_last_error = "no msg1";
        }
    }
    else {
        m_check_buffer_0_first = true;
    }  
    
    
    return ret_value;
}



