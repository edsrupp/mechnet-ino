/* 
 * File:   MCP_2515_CAN_RX_Msg.h
 * Author: dad
 *
 * Created on April 27, 2016, 9:13 PM
 */

#ifndef MCP_2515_CAN_RX_MSG_H
#define	MCP_2515_CAN_RX_MSG_H

#include <Arduino.h>

#define CAN_MAX_RX_MSG_SIZE (8)
class MCP_2515_CAN_RX_Msg {
public:
    MCP_2515_CAN_RX_Msg();

    // Get size of message
    uint8_t getSize(){return (m_RXBnDLC & B00001111);}; //B00001111);};
    
    // Get message id
    uint32_t getId();
    
    //Standard Frame Remote Transmit Request
    bool isStandard_rtr(){ return !isExtended_frame() && (m_RXBnSIDL & B00010000); } // SRR: Standard Frame Remote Transmit Request bit (valid only if IDE bit = ‘0’)
    
    //Standard Data Frame Received
    bool isStandard_frame(){return !isExtended_frame() && !isStandard_rtr();};
    
    //Received message was an Extended Frame
    bool isExtended_frame(){return (m_RXBnSIDL & B00001000);}; //IDE: Extended Identifier Flag bit
    
    const byte* getBuffer(){return &(m_buf[0]) ;};  
    
protected:
friend class CAN_DGL_mcp_2515_SPI;
    byte m_RXBnSIDH;
    byte m_RXBnSIDL;
    byte m_RXBnEID8;
    byte m_RXBnEID0;
    byte m_RXBnDLC;
    byte m_buf[CAN_MAX_RX_MSG_SIZE];
};



class Rx_Message_Pool8_t {
public:
    Rx_Message_Pool8_t(void);

    // Get an available message buffer
    // return NULL if no free messages 
    MCP_2515_CAN_RX_Msg* get_message(void);
    
    // Return message to pool
    void return_message(MCP_2515_CAN_RX_Msg* msg_ptr );
    
    void print_map(void){
        Serial.println(m_message_available_bits,HEX);
    }
       
protected:
    byte m_message_available_bits;
    MCP_2515_CAN_RX_Msg m_message_pool[8];
};


#endif	/* MCP_2515_CAN_RX_MSG_H */

