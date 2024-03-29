#include <Adafruit_BLEGatt.h>
#include <Adafruit_BluefruitLE_SPI.h>

#include <mcp_can.h>
#include <mcp_can_dfs.h>

//#include "CAN_DGL_SPI.h"

#define BUFSIZE                        128

#define BLUEFRUIT_SPI_CS               9
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

// SOFTWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for SW SPI communication.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI
// (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11

#define VERBOSE_MODE                   true  // If set to 'true' enables debug output

#define CAN_ID_PID          0x7DF

// demo: CAN-BUS Shield, receive data
//#include <Arduino.h>

//#include <EEPROM.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

//DGL_CanbusClass Canbus;

char buffer[456];  //Data will be temporarily stored to this buffer before being written to the file

const char *blank_label = "                    "; //20 spaces
const char *vehicle_Speed_label = "Vehicle Speed";
const char *engine_RPM_label = "Engine RPM";
const char *throttle_label = "Throttle";
const char *engine_Coolant_Temp_label = "Engine Coolant Temp";
const char *voltage_O2_label = "O2 Voltage";

//This needs to be set dynamically from the call that registers the characteristic
int32_t charIdTx;
int32_t charIdRx;

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//IMPORTANT: I spent 3 WEEKS trying to get the CAN and SPI libs to work together i.e. including
//the CAN initialization in the setup function kept screwing up the 
//ble spi communication with the phone - switching to the "hardware" ctor fixed that...
//Must be some configuration conflict with my use of the "software SPI" ctor seen above... 
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//Provides syntactic sugar to make programing to the api easier
Adafruit_BLEGatt gatt(ble);

//CAN_DGL_mcp_2515_SPI CAN(9);
MCP_CAN CAN(BLUEFRUIT_SPI_CS);

//Some globals for keeping track of characteristics
int characteristicId_[5];
int serviceId_[5];

void updateBleVal(uint8_t CharacteristicId, uint32_t MeasuredValue) {
  ble.print(F("AT+GATTCHAR="));
  ble.print(CharacteristicId);
  ble.print(F(","));
  ble.println(MeasuredValue);

  if(!ble.waitForOK()) {
    Serial.println(F("Problem updating characteristic value"));
  }
}

void sendPid(unsigned char __pid)
{
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    Serial.print("SEND PID: 0x");
    Serial.println(__pid, HEX);
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

//Here we should figure out a way to wait for the requested data to come back from the CAN.
//I think it would simplify the flow control a little...
void taskCanRecv()
{
    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == CAN.checkReceive())                   // check if get data
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        Serial.println("\r\n------------------------------------------------------------------");
        Serial.print("Get Data From id: ");
        Serial.println(CAN.getCanId(), HEX);
        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print("0x");
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();
        uint8_t one = 1;
        
        //Instead of calling setChar here would could return the data from taskCanRec for setChar from within bleRx
        gatt.setChar(one, buf, len);
    }
}

//Callback to handle data/requests coming from the phone i.e. central device.
void bleRx(int32_t chars_id, uint8_t data[], uint16_t len)
{
  Serial.print( F("[BLE GATT RX] (" ) );
  Serial.print(chars_id);
  Serial.print(") ");
  Serial.print("The PID: ");
  uint8_t pid = data[1] | data[0];
  Serial.println(pid);
  if (chars_id == charIdRx)
  {  
    Serial.write(data, len);
    Serial.println();
  }else{
    Serial.println("We got something else back");
  }

  //Update our characteristic for the phone to pick up
  //updateBleVal(1,777);

  //Here we would take the message coming from the phone and create a message(request) for the CAN...
  //send it out and return the requested information back to the phone which in turn sends that
  //information back to the DMV for emission analysis
  //uint8_t const retDat[] = {'E','d',' ','T','h','i','s',' ','Y','o','u','r',' ','D','a','t','a','!'};
  //uint8_t const retDat[] = {'F','U','C','K'};
  unsigned char const retDat[] = {0x46,0x55,0x43,0x4b};
  //char retDat[] = "FUCK";
  uint8_t one = 1;
  uint8_t sd = 4;

  //Here we will forward the incomming PID requested by the phone
  sendPid(data[0]);

  //Then we wait for the data to be returned
  taskCanRecv();
  
  //Then we turn around and update the characteristic we are using to relay the data back to the phone
  //gatt.setChar(one, retDat, sd);
}

void setupBleServices() {
  //used when adding characteristics to define their data
  //types and formats for presentation to the Central device i.e. phone or pad
  //Look up presentation structure in BLE 
  //GattPresentationFormat pres = GattPresentationFormat(); 
   //Set up our service and characteristics
  //use gatt.clear() instead
  //use ble.* api like the old man uses for the rest of this stuff
  //Generated by uuidgenerator.net: 6e-24-60-4a-b4-30-48-5a-81-e8-df-e0-9e-d5-6a-a7
  ble.factoryReset(); 
  ble.sendCommandCheckOK( F("AT+GATTCLEAR") );
  ble.sendCommandCheckOK( F("AT+GATTADDSERVICE=UUID128=00-11-FE-ED-99-55-66-77-88-AA-BB-AA-FF-EE-EE-DD") );
  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-11-24-24-99-55-66-77-88-AA-BB-AA-FF-EE-EE-DD,PROPERTIES=0x1A,MIN_LEN=1,MAX_LEN=15,DATATYPE=string,DESCRIPTION=string,VALUE=abc"), &charIdTx);
  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-11-42-42-99-55-66-77-88-AA-BB-AA-FF-EE-EE-DD,PROPERTIES=0x1A,MIN_LEN=1,MAX_LEN=15,DATATYPE=string,DESCRIPTION=string,VALUE=def"), &charIdRx);
  ble.setBleGattRxCallback(charIdRx, bleRx);
  ble.setBleGattRxCallback(charIdTx, bleRx);
  Serial.print(F("CharIdRx: "));
  Serial.println(charIdRx);
  
  //Aparently we need to do this for our changes to take effect!!!
  //If you don't reset here none of the services added above will be seen by your app!!!
  ble.reset();
  ble.sendCommandCheckOK( F("AT+GATTLIST") );
  ble.end();
}

bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}

void setup()
{

 /* Initialise the module */

    while (!Serial);  // required for Flora & Micro
     delay(500);
   Serial.begin(115200);
  //
  //CAN related setup
  //
  
    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 1000k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    
    Serial.println("CAN BUS Shield init ok!");
    
    CAN.init_Mask(0, 0, 0x7FC);                         // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, 0, 0x7FC);

    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    CAN.init_Filt(0, 0, 0x7E8);                 
    CAN.init_Filt(1, 0, 0x7E8);

    CAN.init_Filt(2, 0, 0x7E8);
    CAN.init_Filt(3, 0, 0x7E8);
    CAN.init_Filt(4, 0, 0x7E8); 
    CAN.init_Filt(5, 0, 0x7E8);


   //Serial.begin(9600);

  //
  //BLE related setup
  //
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  setupBleServices();

  /* Disable command echo from Bluefruit */
  //ble.echo(false);
    
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Adafruit Bluefruit AT Command Example"));
  Serial.println(F("-------------------------------------"));

  
  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  
}


const char * last_lable = NULL;
void display(const char* label, const char *buffer, uint8_t line = 0 ){
    Serial.print(label);
   // Serial.println(buffer);
        for(int i = 0; i<8; i++)    // print the data
            {
                Serial.print(buffer[i], HEX);
            }
        
        Serial.print(F("\t"));
   
    delay(10);
}


//Rx_Message_Pool8_t message_pool;
uint8_t packetbuffer[10];
uint16_t replyidx = 0;
char data = 0;
void loop()
{
   ble.update();

   if (Serial.available() > 0) {
    data = Serial.read();
    Serial.print(data);
   }

     // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");
  }

   /*
   Serial.println("Update has been called");
   uint8_t* bufStuff;
   uint8_t bufsize;
   gatt.getChar( charIdTx, bufStuff, bufsize);
   Serial.print(F("Size of data: "));
   Serial.println(bufsize);
   Serial.print(F("Data value: "));
   ble.printByteArray(bufStuff, bufsize);
   Serial.println();
   */

/*
    char* cmdRes = gatt.getCharStr((uint8_t)charIdRx);
    if (cmdRes != 0) {
      Serial.print(cmdRes);    
    }
*/
   // if (!cmdRes){
    // Serial.println(F("Problem requesting characteristic data"));
   // }
    
    //uint8_t buffStuff[10];
    //ble.readline(buffStuff, 10);
   // Serial.println("What is buffStuff");
    
   // ble.printByteArray(buffStuff, 10);
   /*
   char inputs[BUFSIZE+1];
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }
  */
  /*
    if (ble.available()) {
      Serial.println("Data is available...");
      int c =  ble.read();
      Serial.print(c, HEX);
      Serial.println();
      if (c == '!') {
        replyidx = 0;
      }
      packetbuffer[replyidx] = c;
      replyidx++;
    }
    */
    
    //unsigned char len = 0;
    //unsigned char buf[8];
    //Set up filters for CAN message types
    //CAN.init_Filt(byte num, byte ext, unsigned long ulData
    /*
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned int canId = CAN.getCanId();
        
        Serial.println("-----------------------------");
        Serial.print("Get data from ID: ");
        Serial.println(canId, HEX);

        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();
    }

*/
    
        // MCP_2515_CAN_RX_Msg* msg = message_pool.get_message();
        /*if(msg ){
            if(CAN.tryRead(*msg))            // check if data coming
            {
                Serial.println("-------------");
                Serial.println(msg->getSize() );
                Serial.println(msg->getId() ,HEX);

                display("buf:",(const char*)msg->getBuffer());


                 Serial.println(CAN.last_error_msg() );
                 if(CAN.get_rx_status_byte() & B01000000){
                     Serial.println("Buff 0");
                 }else if(CAN.get_rx_status_byte() & B10000000){
                     Serial.println("Buff 1");
                 }

            }else{
              // Serial.println(CAN.get_status_byte(),BIN);

            }
            message_pool.return_message(msg);
        }*/

                
        
    /*
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        uint32_t canId = CAN.getCanId();
        
        Serial.println("-----------------------------");
        Serial.print("Buffer: ");
        Serial.print(CAN.m_buffer_read, DEC);
        Serial.print(" ID: ");
        Serial.print(canId, HEX);
        Serial.print(" len: ");
        Serial.print(len, DEC);
        
        if( CAN.isExtendedFrame()){
            Serial.print(" EF");
        }
        if( CAN.isRemoteRequest()){
            Serial.print(" RR");
        }
        Serial.println();
        if(INT8U ret =  CAN.checkError() != CAN_OK){
            Serial.print("ERROR:");
            Serial.print(ret);
        }else{
        for(int i = 0; i<len; i++)    // print the data
            {
                Serial.print(buf[i], HEX);
                Serial.print("\t");
            }
        }
        Serial.println();
    }
    */
}

//Look into serialEventRun(){} it is used for interrupt event driven processing


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
