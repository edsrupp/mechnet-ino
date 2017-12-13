/* 
 * File:   MCP_2515_CAN_RX_Msg.cpp
 * Author: dad
 * 
 * Created on April 27, 2016, 9:13 PM
 */

#include "MCP_2515_CAN_RX_Msg.h"

MCP_2515_CAN_RX_Msg::MCP_2515_CAN_RX_Msg():
   m_RXBnSIDH(0),
   m_RXBnSIDL(0),
   m_RXBnEID8(0),
   m_RXBnEID0(0),
   m_RXBnDLC(0)
{
    for(uint8_t i = 0; i <CAN_MAX_RX_MSG_SIZE; ++i  ){
       m_buf[i] = 0;
    }
}

uint32_t MCP_2515_CAN_RX_Msg::getId(){
    
    if(isStandard_frame() || isStandard_rtr()){
        return (m_RXBnSIDH<<3) + (m_RXBnSIDL>>5);
    }else{
        return ((uint32_t)(m_RXBnSIDL & B00000011) << 16) + (m_RXBnEID8 << 8) + m_RXBnEID0;
    }
}



Rx_Message_Pool8_t::Rx_Message_Pool8_t():
       m_message_available_bits(0)
{};
       
// Get an available message buffer
// return NULL if no free messages 
MCP_2515_CAN_RX_Msg* Rx_Message_Pool8_t::get_message(void){
    if(m_message_available_bits == 0xFF){
        return NULL;
    }
    
    uint8_t index = 0;
    for( ; index < 8 && bitRead(m_message_available_bits,index) == 1 ; ++index){
    }
    
    bitSet(m_message_available_bits,index);

    return &m_message_pool[index];
}

// Return message to pool
void Rx_Message_Pool8_t::return_message(MCP_2515_CAN_RX_Msg* msg_ptr){
    if( msg_ptr == NULL){
        return;
    }
    uint8_t index = 0;
    for(; index < 8 and msg_ptr != &m_message_pool[index]; ++index){
    }
    bitClear(m_message_available_bits,index);
}