/*
 * Before Use, Select Option:
 * 1. TYPE1SC: EXT_ANT_ON
 * 2. Blynk: USE_WIFI
 * 3. RS485: EC_SENSING_MODE
 */

/* Arduino setting **********************************/
#include <Arduino.h>
#include <ArduinoJson.h>

/* TYPE1SC setting **********************************/
#include <U8x8lib.h>
#include "TYPE1SC.h"

#define SERIAL_BR 115200
#define GSM_SERIAL 1
#define GSM_RX 16
#define GSM_TX 17
#define GSM_BR 115200

#define PWR_PIN 5
#define RST_PIN 18
#define WAKEUP_PIN 19
#define EXT_ANT 4

#define DebugSerial Serial
#define M1Serial Serial2 // ESP32

String APN = "simplio.apn";

TYPE1SC TYPE1SC(M1Serial, DebugSerial, PWR_PIN, RST_PIN, WAKEUP_PIN);

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
#define U8LOG_WIDTH 16
#define U8LOG_HEIGHT 8
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
U8X8LOG u8x8log;

/* EXT_ANT_ON 0 : Use an internal antenna.
 * EXT_ANT_ON 1 : Use an external antenna.
 */
#define EXT_ANT_ON 1
void extAntenna();

/* Blynk setting ************************************/
// Use WIFI? *************************************************
// #define USE_WIFI
// ***********************************************************

#include "config_Blynk.h"

// #include <BlynkSimpleEsp32.h> // wifi header...
#include <BlynkLTECatM1Esp32.h> // edited from BlynkSimpleEsp32.h

// #define BLYNK_PRINT Serial   // #define DebugSerial Serial

char auth[] = BLYNK_AUTH_TOKEN;
BlynkTimer timer; // 함수 주기적 실행 위한 타이머

/* RS485 setting ************************************/
/*
 * https://github.com/CMB27/ModbusRTUMaster/tree/main

 * Function Code: 0x01; readCoils(): modbus.readCoils(slaveId, startAddress, buffer, quantity); 1비트 읽기; 싱글 코일 읽기
 * Function Code: 0x02; readDiscreteInputs(): modbus.readDiscreteInputs(slaveId, startAddress, buffer, quantity); 1비트 읽기; 이산 입력 읽기
 * Function Code: 0x03; readHoldingRegisters(): modbus.readHoldingRegisters(slaveId, startAddress, buffer, quantity); 여러 워드 읽기; 레지스터의 연속 블록 내용 읽기
 * Function Code: 0x04; readInputRegisters(): modbus.readInputRegisters(slaveId, startAddress, buffer, quantity); 여러 워드 읽기; 입력 레지스터 읽기
 *
 * Function Code: 0x05; writeSingleCoil(): modbus.writeSingleCoil(slaveId, address, value); 1비트 쓰기; 원격 장치의 ON/OFF에 단일 출력 쓰기; FF00:ON; 0000:OFF
 * Function Code: 0x06; writeSingleHoldingRegister(): modbus.writeSingleHoldingRegister(slaveId, address, value); 1워드 쓰기; 싱글 홀딩 레지스터 쓰기
 * Function Code: 0x0F; writeMultipleCoils(): modbus.writeMultipleCoils(slaveId, startingAddress, buffer, quantity); 여러 비트 쓰기; 다수 코일 ON/OFF 지정
 * Function Code: 0x10; writeMultipleHoldingRegisters(): modbus.writeMultipleHoldingRegisters(slaveId, startingAddress, buffer, quantity); 여러 워드 쓰기; 연속 레지스터 블록 쓰기
*/
#include <ModbusRTUMaster.h>

#define SLAVE_ID 1
#define START_ADDRESS 0
#define QUANTITY 2
#define SCAN_RATE 10000

// Soil Moisture + Temp + EC *********************************
#define EC_SENSING_MODE // To disable, change to annotation.
// ***********************************************************

unsigned long currentMillis = 0;
unsigned long previousMillis = 0; // Stores last time using Reference time

unsigned int messageID = 0;
#if defined(EC_SENSING_MODE)
float t;
float soil_m; // Soil Moisture
float ec;     // 전기 전도도
#else
float t;
float h;
#endif

const uint8_t rxPin = 33; // RX-RO
const uint8_t txPin = 32; // TX-DI
const uint8_t dePin = 25; // DE+RE

ModbusRTUMaster modbus(Serial1, dePin); // serial port, driver enable pin for rs-485 (optional)
void getSensorData();

#if defined(EC_SENSING_MODE)
uint16_t holdingRegisters[3] = {0xFF, 0xFF, 0xFF};
#else
uint16_t holdingRegisters[2] = {0xFF, 0xFF};
#endif

void setup()
{
    /* CATM1 Modem PowerUp sequence */
    pinMode(PWR_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    pinMode(WAKEUP_PIN, OUTPUT);

    digitalWrite(PWR_PIN, HIGH);
    digitalWrite(WAKEUP_PIN, HIGH);
    digitalWrite(RST_PIN, LOW);
    delay(100);
    digitalWrite(RST_PIN, HIGH);
    delay(2000);

    /********************************/

    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
    u8x8log.setRedrawMode(
        1); // 0: Update screen with newline, 1: Update screen for every char

    // put your setup code here, to run once:
    M1Serial.begin(SERIAL_BR);
    DebugSerial.begin(SERIAL_BR);

    DebugSerial.println("TYPE1SC Module Start!!!");
    u8x8log.print("TYPE1SC Module Start!!!\n");

    extAntenna();

    /* TYPE1SC Module Initialization */
    if (TYPE1SC.init())
    {
        DebugSerial.println("TYPE1SC Module Error!!!");
        u8x8log.print("TYPE1SC Module Error!!!\n");
    }

    /* Network Regsistraiton Check */
    while (TYPE1SC.canConnect() != 0)
    {
        DebugSerial.println("Network not Ready !!!");
        u8x8log.print("Network not Ready!!!\n");
        delay(2000);
    }

    /* Get Time (GMT, (+36/4) ==> Korea +9hour) */
    char szTime[32];
    if (TYPE1SC.getCCLK(szTime, sizeof(szTime)) == 0)
    {
        DebugSerial.print("Time : ");
        DebugSerial.println(szTime);
        u8x8log.print("Time : ");
        u8x8log.print(szTime);
        u8x8log.print("\n");
    }
    delay(1000);

    DebugSerial.println("TYPE1SC Module Ready!!!");
    u8x8log.print("TYPE1SC Module Ready!!!\n");

    /* Enter a DNS address to get an IP address */
    char IPAddr[32];

    while (1)
    {

        if (TYPE1SC.getIPAddr(BLYNK_DEFAULT_DOMAIN, IPAddr, sizeof(IPAddr)) == 0)
        {
            DebugSerial.print("Blynk IP Address : ");
            DebugSerial.println(IPAddr);
            u8x8log.print("Blynk IP Address : ");
            u8x8log.print(IPAddr);
            u8x8log.print("\n");
            break;
        }
        else
        {
            DebugSerial.println("Blynk IP Address Error!!!");
            u8x8log.print("Blynk IP Address Error!!!\n");
        }
        delay(2000);
    }

    // Use TCP Socket
    /********************************/

    int _PORT = BLYNK_DEFAULT_PORT;
    char sckInfo[128];
    char recvBuffer[700];
    int recvSize;

    /* 1 :TCP Socket Create ( 0:UDP, 1:TCP ) */
    if (TYPE1SC.socketCreate(1, IPAddr, _PORT) == 0)
    {
        DebugSerial.println("TCP Socket Create!!!");
        u8x8log.print("TCP Socket Create!!!\n");
    }

INFO:

    /* 2 :TCP Socket Activation */
    if (TYPE1SC.socketActivate() == 0)
    {
        DebugSerial.println("TCP Socket Activation!!!");
        u8x8log.print("TCP Socket Activation!!!\n");
    }

    if (TYPE1SC.socketInfo(sckInfo, sizeof(sckInfo)) == 0)
    {
        DebugSerial.print("Socket Info : ");
        DebugSerial.println(sckInfo);
        u8x8log.print("Socket Info : ");
        u8x8log.print(sckInfo);
        u8x8log.print("\n");

        if (strcmp(sckInfo, "ACTIVATED"))
        {
            delay(3000);
            goto INFO;
        }
    }

    // online forever TCP Socket; auto close if disconnect
    // /* 5 :TCP Socket DeActivation */
    // if (TYPE1SC.socketDeActivate() == 0)
    // {
    //     DebugSerial.println("TCP Socket DeActivation!!!");
    //     u8x8log.print("TCP Socket DeActivation!!!\n");
    // }

    // if (TYPE1SC.socketInfo(sckInfo, sizeof(sckInfo)) == 0)
    // {
    //     DebugSerial.print("Socket Info : ");
    //     DebugSerial.println(sckInfo);
    //     u8x8log.print("Socket Info : ");
    //     u8x8log.print(sckInfo);
    //     u8x8log.print("\n");
    // }

    // /* 6 :TCP Socket DeActivation */
    // if (TYPE1SC.socketClose() == 0)
    // {
    //     DebugSerial.println("TCP Socket Close!!!");
    //     u8x8log.print("TCP Socket Close!!!\n");
    // }

    // /* 7 :Detach Network */
    // if (TYPE1SC.setCFUN(0) == 0)
    // {
    //     DebugSerial.println("detach Network!!!");
    //     u8x8log.print("detach Network!!!\n");
    // }
    // delay(10000); // Detach Setup Time : 10sec

    // Begin Blynk
    /********************************/
#if defined(USE_WIFI)
    Blynk.begin(auth, "dinfo", "daon7521");
#endif

    // WIFI 없이는 쉽지않네
    // IPAddress blynkIP;
    // if (blynkIP.fromString(IPAddr))
    // {
    //     Serial.print("IP Type convert Success, IP Address: ");
    //     Serial.println(blynkIP);
    // }
    // else
    // {
    //     Serial.println("IP Type convert Failed");
    // }

    // Blynk.begin(auth, blynkIP, BLYNK_DEFAULT_PORT);

    /********************************/

    // RS485 Setup
    modbus.setTimeout(12000);
    modbus.begin(9600, SERIAL_8N1, rxPin, txPin); // 직렬 전송 설정 (baud, config, rxPin, txPin, invert)
                                                  // default config : SERIAL_8N1; { 데이터비트 8, 패리티 없음, 1 정지 비트}; E: 짝수 패리티, O: 홀수 패리티
                                                  // rxPin: 직렬 데이터 수신 핀; txPin: 직렬  데이터 전송 핀 (uint8_t)

    // 함수 주기 실행
#if defined(USE_WIFI)
    timer.setInterval(SCAN_RATE, getSensorData);
#endif
}

void loop()
{
#if defined(USE_WIFI)
    Blynk.run();
    timer.run();
#else
    currentMillis = millis();
    if (currentMillis - previousMillis >= SCAN_RATE)
    {
        previousMillis = currentMillis;

        getSensorData();

        /* 3 :TCP Socket Send Data */
        String data = "https//blynk.cloud/external/api/update?token=" BLYNK_AUTH_TOKEN "&";
        data += "V0=" + String(t) + "&" + "V1=" + String(soil_m) + "&" + "V2=" + String(ec); //" GET / update ";

        if (TYPE1SC.socketSend(data.c_str()) == 0)
        {
            DebugSerial.print("[HTTP Send] >> ");
            DebugSerial.println(data);
            u8x8log.print("[HTTP Send] >> ");
            u8x8log.print(data);
            u8x8log.print("\n");
        }
        else
        {
            DebugSerial.println("HTTP Send Fail!!!");
            u8x8log.print("HTTP Send Fail!!!\n");
        }
    }
#endif
}

void extAntenna()
{
    if (EXT_ANT_ON)
    {
        pinMode(EXT_ANT, OUTPUT);
        digitalWrite(EXT_ANT, HIGH);
        delay(100);
    }
}

void getSensorData()
{
#if defined(EC_SENSING_MODE)
    if (modbus.readHoldingRegisters(1, 0, holdingRegisters, 3))
    {
        t = holdingRegisters[0] / 10.0;
        soil_m = holdingRegisters[1] / 10.0;
        ec = holdingRegisters[2] / 1000.0;
        Serial.printf("RK520-02 [messageID]: %d | [TEMP]: %.1f, [Moisture]: %.1f, [EC]: %.2f\n", messageID, t, soil_m, ec);

#if defined(USE_WIFI)
        // Sent to Blynk
        Blynk.virtualWrite(V0, t);
        Blynk.virtualWrite(V1, soil_m);
        Blynk.virtualWrite(V2, ec);
#endif
    }
    else
    {
        Serial.println("Cannot Read Holding Resisters...");
    }

#else
    if (modbus.readHoldingRegisters(1, 0, holdingRegisters, 2))
    {
        t = holdingRegisters[0] / 10.0;
        h = holdingRegisters[1] / 10.0;
        Serial.printf("TZ-THT02 [messageID]: %d | [TEMP]: %.1f, [HUMI]: %.1f\n", messageID, t, h);

        // Sent to Blynk
        Blynk.virtualWrite(V0, t);
        Blynk.virtualWrite(V1, h);
    }
    else
    {
        Serial.println("Cannot Read Holding Resisters...");
    }

#endif

    messageID++;
}