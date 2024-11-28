/***********************************************************************
 * Project      :     tiny32_SX1278_LoRa_Slave
 * Description  :     Example code tiny32 board interface with SX1278 to be LoRa Client
 * Hardware     :     tiny32_v3
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     28/11/2024
 * Revision     :     1.1
 * Rev1.0       :     Origital
 * Rev1.1       :    - Repeat send data to LoRa Gateway N times
 *                   - Ignore DIO0 pin
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     uten.boonliam@tenergyinnovation.co.th
 * TEL          :     +66 89-140-7205
 ***********************************************************************/
#include <Arduino.h>
#include <tiny32_v3.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include <esp_task_wdt.h>
/**************************************/
/*          Firmware Version          */
/**************************************/
#define FIRMWARE_VERSION "1.1"
#define TOPIC "lora_topc"
#define lora_repeatsend_time 3 //จำนวนครั้งที่ส่งซ้ำไปยัง LoRa gateway เพื่อไม่ให้พลาดการรับข้อมูล
/**************************************/
/*          Header project            */
/**************************************/
void header_print(void)
{
    Serial.printf("\r\n***********************************************************************\r\n");
    Serial.printf("* Project      :     tiny32_SX1278_LoRa_Slave\r\n");
    Serial.printf("* Description  :     Example code tiny32 board interface with SX1278 to be LoRa Client\r\n");
    Serial.printf("* Hardware     :     tiny32_v3\r\n");
    Serial.printf("* Author       :     Tenergy Innovation Co., Ltd.\r\n");
    Serial.printf("* Date         :     04/07/2022\r\n");
    Serial.printf("* Revision     :     %s\r\n", FIRMWARE_VERSION);
    Serial.printf("* Rev1.0       :     Origital\r\n");
    Serial.printf("* website      :     http://www.tenergyinnovation.co.th\r\n");
    Serial.printf("* Email        :     uten.boonliam@tenergyinnovation.co.th\r\n");
    Serial.printf("* TEL          :     +66 89-140-7205\r\n");
    Serial.printf("***********************************************************************/\r\n");
}

/**************************************/
/*        define object variable      */
/**************************************/
tiny32_v3 mcu;

/**************************************/
/*            GPIO define             */
/**************************************/
#define SCK 18
#define MISO 23
#define MOSI 19
#define SS 5
#define RST 26
#define DIO0 -1
// #define DIO0 27  //ignor

/**************************************/
/*           Bandwidth  define        */
/**************************************/
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define BAND 433E6

/**************************************/
/*       Constand define value        */
/**************************************/
// 10 seconds WDT
#define WDT_TIMEOUT 10

/**************************************/
/*       eeprom address define        */
/**************************************/

/**************************************/
/*        define global variable      */
/**************************************/
String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte LoRa_ID_Gateway = 1;     // LoRa Gate Way ID {Every Client will indicate to this ID} ส่งข้อมูลไปหา GateWay นี้
byte LoRa_ID_local = 2;       // address of this device **  เป็น ID ของ LoRa ตัวนี้
byte LoRa_ID_destination = 1; // destination to send to **
byte LoRa_ID_wildcard = 99;   // All Client node will accept wildcard id , Client ทุกตัวจะยอมรับ ID นี้
long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
char mac_address[17];
char unit[50];
// char macid[50];

char data_read[100];
uint8_t byte_cnt = 0;
String msg = "";

struct param_LoRa
{
    String id;
    String fw;
    String rssi;
    String topic;
    float param_1;
    float param_2;
    float param_3;
    float param_4;
    float param_5;
    float param_6;
    float param_7;
    float param_8;
    float param_9;
    float param_10;
};

param_LoRa LoRa_client_1;
byte incomingMsgId_LoRa = 0;

/**************************************/
/*           define function          */
/**************************************/
void LoRa_onReceive(int packetSize); // LoRa get message funct
void sendMessage(String outgoing);   // LoRa send message funct

/***********************************************************************
 * FUNCTION:    setup
 * DESCRIPTION: setup process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void setup()
{
    Serial.begin(115200);
    header_print();

    //*** Define Unit name ***
    String _mac1 = WiFi.macAddress().c_str();
    Serial.printf("Setup: macAddress = %s\r\n", WiFi.macAddress().c_str());
    String _mac2 = _mac1.substring(9, 17);
    _mac2.replace(":", "");
    String _mac3 = "tiny32-" + _mac2;
    _mac3.toCharArray(unit, _mac3.length() + 1);
    Serial.printf("Setup: Unit ID => %s\r\n", unit);

    /* LoRa configuration */
    Serial.println("Info: LoRa Duplex config....");
    // SPI LoRa pins
    SPI.begin(SCK, MISO, MOSI, SS);
    // setup LoRa transceiver module
    LoRa.setPins(SS, RST, DIO0);
    if (!LoRa.begin(BAND))
    { // initialize ratio at 866 MHz
        Serial.println("\r\nError: LoRa init failed. Check your connections.");
        mcu.buzzer_beep(3);
        while (true)
            ; // if failed, do nothing
    }
    Serial.printf("\tInfo: Frequency Band  = %dHz\r\n", int(BAND));
    Serial.printf("\tInfo: Gateway ID = %02d\r\n", LoRa_ID_Gateway);
    Serial.printf("\tInfo: My ID = %02d\r\n", LoRa_ID_local);
    Serial.printf("\tInfo: Destination ID = %02d\r\n", LoRa_ID_destination);

    Serial.println("Configuring WDT...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               // add current thread to WDT watch
    mcu.buzzer_beep(2);
}

/***********************************************************************
 * FUNCTION:    loop
 * DESCRIPTION: loop process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void loop()
{

    if (mcu.Sw1())
    {
        mcu.buzzer_beep(1);
        Serial.printf("Info: send message via RoLa module ....\r\n");
        while (mcu.Sw1())
            ;

        String _JsonStr = "{";
        _JsonStr += "\"id\":\"" + String(unit) + "\",";
        _JsonStr += "\"fw\":\"" + String(FIRMWARE_VERSION) + "\",";
        _JsonStr += "\"topic\":\"" + String(TOPIC) + "\",";
        _JsonStr += "1.1,";
        _JsonStr += "2.2,";
        _JsonStr += "3.3,";
        _JsonStr += "4.4,";
        _JsonStr += "5.5,";
        _JsonStr += "6.6,";
        _JsonStr += "7.7,";
        _JsonStr += "8.8,";
        _JsonStr += "9.9,";
        _JsonStr += "10.1";
        _JsonStr += "}";
        sendMessage(_JsonStr); // Lora Wan send data
        vTaskDelay(1000);
    }
    LoRa_onReceive(LoRa.parsePacket());
    esp_task_wdt_reset();
    vTaskDelay(100);
}

/***********************************************************************
 * FUNCTION:    sendMessage
 * DESCRIPTION: send data to LoRaWan
 * PARAMETERS:  String outgoing  ** ขนาดห้ามเกิน 250 BYTE **
 * RETURNED:    nothing
 ***********************************************************************/
void sendMessage(String outgoing)
{

    Serial.printf("outgoing.length() = %d\r\n", outgoing.length());
    if (outgoing.length() > 250)
    {
        mcu.buzzer_beep(3);
        Serial.println("Error: Message length is over, it can't send via LoRa protocol!");
        return;
    }

    for (int i = 0; i < lora_repeatsend_time; i++) // Repeat send data to LoRa Gateway N times
    {
        LoRa.beginPacket();            // start packet
        LoRa.write(LoRa_ID_Gateway);   // add destination address
        LoRa.write(LoRa_ID_local);     // add sender address
        LoRa.write(msgCount);          // add message ID
        LoRa.write(outgoing.length()); // add payload length
        LoRa.print(outgoing);          // add payload
        LoRa.endPacket();              // finish packet and send it
        vTaskDelay(300);
    }
    msgCount++; // increment message ID
    if (msgCount >= 100)
        msgCount = 0;
}

/***********************************************************************
 * FUNCTION:    LoRa_onReceive
 * DESCRIPTION: receive data to LoRaWan
 * PARAMETERS:  int packetSize
 * RETURNED:    nothing
 ***********************************************************************/
void LoRa_onReceive(int packetSize)
{
    String _sensor = "";
    String _value = "";
    int _index = 0;

    if (packetSize == 0)
        return; // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    byte sender = LoRa.read();         // sender address
    byte incomingMsgId = LoRa.read();  // incoming msg ID
    byte incomingLength = LoRa.read(); // incoming msg length

    String incoming = "";

    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("error: message length does not match length");
        return; // skip rest of function
    }

    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();

    if ((recipient == LoRa_ID_local) && (sender == LoRa_ID_Gateway))
    {
        Serial.println("Info: Get message from Gateway to My ID");
        Serial.println(incoming);
        _sensor = incoming;
        _value = incoming;
        unsigned int _index1 = incoming.indexOf(":");
        unsigned int _index2 = incoming.indexOf("\r");
        // Serial.printf("_index1 = %d\r\n", _index1);
        // Serial.printf("_index2 = %d\r\n", _index2);

        // Get 1's parameter
        _sensor.remove(_index1, _index2);
        _sensor.trim();
        Serial.printf("sensor = %s\r\n", _sensor.c_str());

        // Get 2'nd parameter
        _value.remove(0, _index1 + 1);
        _value.trim();
        Serial.printf("value = %s\r\n", _value);

        // control relay
        if (_sensor == "relay")
        {
            if (_value == "on")
            {
                mcu.Relay(1);
            }
            else if (_value == "off")
            {
                mcu.Relay(0);
            }
        }
    }
    else if ((recipient == LoRa_ID_wildcard) && (sender == LoRa_ID_Gateway))
    {
        Serial.println("Info: Get message from Gateway to Wildcard ID");
        Serial.println(incoming);
        _sensor = incoming;
        _value = incoming;
        unsigned int _index1 = incoming.indexOf(":");
        unsigned int _index2 = incoming.indexOf("\r");
        // Serial.printf("_index1 = %d\r\n", _index1);
        // Serial.printf("_index2 = %d\r\n", _index2);

        // Get 1's parameter
        _sensor.remove(_index1, _index2);
        _sensor.trim();
        Serial.printf("sensor = %s\r\n", _sensor.c_str());

        // Get 2'nd parameter
        _value.remove(0, _index1 + 1);
        _value.trim();
        Serial.printf("value = %s\r\n", _value);

        // control relay
        if (_sensor == "relay")
        {
            if (_value == "on")
            {
                mcu.Relay(1);
            }
            else if (_value == "off")
            {
                mcu.Relay(0);
            }
        }
    }
    else if ((sender == LoRa_ID_local))
    {
        Serial.println("\tWarning: Duplicate LoRa Client ID !!! **");
        return;
    }
    else
    {
        return;
    }

    // json method
    // deserializeJson(doc, incoming);
    // serializeJson(doc, Serial);
    // Serial.println("\r\n");
    // serializeJsonPretty(doc, Serial);

    Serial.println("-----------------------------------");
}