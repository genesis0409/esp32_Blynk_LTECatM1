/*
 * Before Use, Select Option:
 * 1. TYPE1SC: EXT_ANT_ON
 * 2. Blynk: USE_WIFI
 * 3. RS485: TZ_THT02 // 0x03
 *           RK520_02 // 0x03
 *           CONOTEC_CNT_TM100   // 0x04
 */

// Arduino setting **************************************************************************************
#include <Arduino.h>
#include <ArduinoJson.h>

#include "soc/soc.h"          // Disable brownout problems
#include "soc/rtc_cntl_reg.h" // Disable brownout problems
#include "driver/rtc_io.h"

#include <U8x8lib.h>
#include "TYPE1SC.h"

#include "config_Blynk.h"
#include <BlynkSimpleEsp32.h> // wifi header...

#include <ModbusMaster.h>

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

#include "SPIFFS.h" // Fast

// 매개 변수 수정 예정 - device토큰: config_Blynk.h 없애겠네ㄷ + TCP GET요청부분도 고쳐야될듯
// Search for parameter in HTTP POST request
const char *PARAM_INPUT_1 = "camId";
const char *PARAM_INPUT_2 = "slaveMAC";
const char *PARAM_INPUT_3 = "capturePeriod";

// Variables to save values from HTML form
String camId;
String slaveMAC;
String capturePeriod; // Interval at which to take photo

// File paths to save input values permanently
const char *camIdPath = "/camId.txt";
const char *slaveMACPath = "/slaveMAC.txt";
const char *capturePeriodPath = "/capturePeriod.txt";

// Use WIFI? ********************************************************************************************
#define USE_WIFI // To disable, Use HTTP Request

// Use LCD? *********************************************************************************************
// #define USE_LCD

// Choose Sensor ****************************************************************************************
// #define TZ_THT02 // 0x03
// #define RK520_02          // 0x03
// #define CONOTEC_CNT_TM100 // 0x04
#define CONOTEC_CNT_WJ24 // 0x04
// #define TEST_HTTP // Test Http Messages
// ******************************************************************************************************

// TYPE1SC setting **************************************************************************************
#define SERIAL_BR 115200

#define PWR_PIN 5
#define RST_PIN 18
#define WAKEUP_PIN 19
#define EXT_ANT 4

#define DebugSerial Serial
HardwareSerial SerialPort(1); // use ESP32 UART1
HardwareSerial M1Serial(2);   // use ESP32 UART2, Arduino ESP32 v3
// #define M1Serial Serial2      // ESP32, Arduino ESP32 v3 업데이트로 미사용

String APN = "simplio.apn";

#if defined(USE_WIFI)

#else
RTC_DATA_ATTR TYPE1SC TYPE1SC(M1Serial, DebugSerial, PWR_PIN, RST_PIN, WAKEUP_PIN);
#endif

#if defined(USE_LCD)
RTC_DATA_ATTR U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
#define U8LOG_WIDTH 16
#define U8LOG_HEIGHT 8
RTC_DATA_ATTR uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
RTC_DATA_ATTR U8X8LOG u8x8log;
#endif

/* TCP/IP, HTTP setting*/
RTC_DATA_ATTR char IPAddr[32];
RTC_DATA_ATTR int _PORT = BLYNK_DEFAULT_PORT;
RTC_DATA_ATTR char sckInfo[128];
RTC_DATA_ATTR char recvBuffer[700];
RTC_DATA_ATTR int recvSize;

/* EXT_ANT_ON 0 : Use an internal antenna.
 * EXT_ANT_ON 1 : Use an external antenna.
 */
#define EXT_ANT_ON 0
void extAntenna();

// Blynk setting ****************************************************************************************
RTC_DATA_ATTR char auth[] = BLYNK_AUTH_TOKEN;
RTC_DATA_ATTR BlynkTimer timer; // 함수 주기적 실행 위한 타이머

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
// RS485 setting ****************************************************************************************

#define DEEPSLEEP_PERIOD_SEC 600 // 600초 ESP32 Deep Sleep

#define SENSING_PERIOD 60000 // 60만: 10분; wifi 센싱 전용

unsigned long currentMillis = 0;
unsigned long previousMillis = 0; // Stores last time using Reference time

RTC_DATA_ATTR unsigned int messageID = 0;
RTC_DATA_ATTR bool errBit; // CONOTEC 센서에러비트; 0:에러없음, 1:센서에러

RTC_DATA_ATTR float t;
RTC_DATA_ATTR float h;
RTC_DATA_ATTR float ec;             // 전기 전도도
RTC_DATA_ATTR bool isRainy = false; // 비

// Allocate the JSON document
RTC_DATA_ATTR JsonDocument doc;

RTC_DATA_ATTR const uint8_t txPin = 32; // TX-DI
RTC_DATA_ATTR const uint8_t rxPin = 33; // RX-RO
RTC_DATA_ATTR const uint8_t dePin = 13; // DE
RTC_DATA_ATTR const uint8_t rePin = 14; // RE

RTC_DATA_ATTR ModbusMaster modbus;

RTC_DATA_ATTR uint8_t modbus_result = modbus.ku8MBInvalidCRC;

#if defined(TZ_THT02) // 0x03
#define SLAVE_ID 1
#define READ_START_ADDRESS 0
#define READ_QUANTITY 2
#endif
#if defined(RK520_02) // 0x03
#define SLAVE_ID 1
#define READ_START_ADDRESS 0
#define READ_QUANTITY 3
#endif
#if defined(CONOTEC_CNT_TM100) // 0x04
#define SLAVE_ID 10            // 0x10 사전 세팅 필요
#define READ_START_ADDRESS 0
#define READ_QUANTITY 3
#endif
#if defined(CONOTEC_CNT_WJ24) // 0x04
#define SLAVE_ID 2            // 0x10 사전 세팅 필요
#define READ_START_ADDRESS_CONOTEC_CNT_WJ24 0x64
#define READ_QUANTITY_CONOTEC_CNT_WJ24 3
#endif
#if defined(TEST_HTTP) // Test Http Messages
#define SLAVE_ID 1
#endif

// 메소드 선언부 *******************************************************************************************
void preTransmission();  // 전송 방식
void postTransmission(); // 수신 방식

void getSensorData();
void sendSensorData();

void initSPIFFS();                                                 // Initialize SPIFFS
String readFile(fs::FS &fs, const char *path);                     // Read File from SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message); // Write file to SPIFFS
bool isCamConfigDefined();                                         // Is Cam Configuration Defined?

void reconnectWifi(); // wifi 연결 복구 함수

// 재시도 관련 설정
const int maxRetries = 3;       // 최대 재시도 횟수
static int wifiRetryCount = 0;  // WiFi 재연결 시도 횟수 저장
static int blynkRetryCount = 0; // Blynk 재연결 시도 횟수 저장

unsigned long reconnectPreviousMillis = 0; // 이전 시간 저장
const unsigned long interval = 100;        // WiFi 상태 체크 주기: 0.1s

RTC_DATA_ATTR bool allowsLoop = false; // loop() DO or NOT
RTC_DATA_ATTR bool firstRun = true;

void extAntenna()
{
    if (EXT_ANT_ON)
    {
        pinMode(EXT_ANT, OUTPUT);
        digitalWrite(EXT_ANT, HIGH);
        delay(100);
    }
}

void preTransmission()
{
    // 전송 방식
    digitalWrite(dePin, HIGH);
    digitalWrite(rePin, HIGH);
}

void postTransmission()
{
    // 수신 방식
    digitalWrite(dePin, LOW);
    digitalWrite(rePin, LOW);
}

void getSensorData()
{
#if defined(TZ_THT02) // 0x03

    modbus_result = modbus.readHoldingRegisters(READ_START_ADDRESS, READ_QUANTITY); // 0x03

    if (modbus_result == modbus.ku8MBSuccess)
    {
        uint16_t rawTemp = modbus.getResponseBuffer(0); // 원본 데이터
        uint16_t rawHumi = modbus.getResponseBuffer(1); // 원본 데이터

        // 음수 변환 처리
        if (rawTemp >= 0x8000)
        {
            t = float((rawTemp - 0xFFFF - 0x01) / 10.0F); // 계산식
        }
        else
        {
            t = float(rawTemp / 10.0F);
        }

        h = float(rawHumi / 10.0F);

        DebugSerial.printf("TZ-THT02 [messageID]: %d | [TEMP]: %.1f, [HUMI]: %.1f\n", messageID, t, h);

#if defined(USE_WIFI)
        // Send to Blynk
        Blynk.virtualWrite(V0, t);
        Blynk.virtualWrite(V1, h);
#endif
    }

    else
    {
        DebugSerial.println("[MODBUS] Cannot Read Holding Resisters...");
        DebugSerial.println("modbus result: ");
        DebugSerial.println(modbus_result);
    }
#endif

#if defined(RK520_02) // 0x03

    // RK520_02: temp, humi, ec
    modbus_result = modbus.readHoldingRegisters(READ_START_ADDRESS, READ_QUANTITY); // 0x03

    if (modbus_result == modbus.ku8MBSuccess)
    {

        uint16_t rawSoilTemp = modbus.getResponseBuffer(0); // 원본 데이터
        uint16_t rawSoilHumi = modbus.getResponseBuffer(1); // 원본 데이터
        uint16_t rawEC = modbus.getResponseBuffer(2);       // 원본 데이터

        // 음수 변환 처리
        if (rawSoilTemp >= 0x8000)
        {
            t = float((rawSoilTemp - 0xFFFF - 0x01) / 10.0F); // 계산식
        }
        else
        {
            t = float(rawSoilTemp / 10.0F);
        }

        h = float(rawSoilHumi / 10.0F);
        ec = float(rawEC / 1000.0F);

        DebugSerial.printf("RK520-02 [messageID]: %d | [TEMP]: %.1f, [Moisture]: %.1f, [EC]: %.3f\n", messageID, t, h, ec);

#if defined(USE_WIFI)
        // Send to Blynk
        Blynk.virtualWrite(V0, t);
        Blynk.virtualWrite(V1, h);
        Blynk.virtualWrite(V2, ec);
#endif
    }

    else
    {
        DebugSerial.println("[MODBUS] Cannot Read Holding Resisters...");
        DebugSerial.println("modbus result: ");
        DebugSerial.println(modbus_result);
    }

#endif

#if defined(CONOTEC_CNT_TM100) // 0x04

    // CONOTEC CNT-TM100: no ec, has errbit
    modbus_result = modbus.readInputRegisters(READ_START_ADDRESS, READ_QUANTITY); // 0x04

    if (modbus_result == modbus.ku8MBSuccess)
    {
        uint16_t rawTemp = modbus.getResponseBuffer(0); // 원본 데이터
        uint16_t rawHumi = modbus.getResponseBuffer(1); // 원본 데이터

        // 음수 변환 처리
        if (rawTemp >= 0x8000)
        {
            t = float((rawTemp - 0xFFFF - 0x01) / 10.0F); // 계산식
        }
        else
        {
            t = float(rawTemp / 10.0F);
        }

        h = float(rawHumi / 10.0F);
        errBit = modbus.getResponseBuffer(2);

        DebugSerial.printf("CNT-TM100 [messageID]: %d | [TEMP]: %.1f, [Moisture]: %.1f, [ErrBit]: %d\n", messageID, t, h, errBit);

        if (errBit)
        {
            DebugSerial.println("Sensor Open ERROR!!!");
        }

#if defined(USE_WIFI)
        // Send to Blynk
        Blynk.virtualWrite(V0, t);
        Blynk.virtualWrite(V1, h);
        // Blynk.virtualWrite(V2, errBit);
#endif
    }

    else
    {
        DebugSerial.println("[MODBUS] Cannot Read Input Resisters...");
        DebugSerial.println("modbus result: ");
        DebugSerial.println(modbus_result);
    }

#endif

#if defined(CONOTEC_CNT_WJ24) // 0x04

    // CONOTEC CNT-WJ24: 감우 및 온도센서
    modbus_result = modbus.readInputRegisters(READ_START_ADDRESS_CONOTEC_CNT_WJ24, READ_QUANTITY_CONOTEC_CNT_WJ24); // 0x04

    if (modbus_result == modbus.ku8MBSuccess)
    {
        // 감우센서 온습도
        uint16_t rawTemp = modbus.getResponseBuffer(0); // 원본 데이터
        uint16_t rawHumi = modbus.getResponseBuffer(1); // 원본 데이터

        // 음수 변환 처리
        if (rawTemp >= 0x8000)
        {
            t = float((rawTemp - 0xFFFF - 0x01) / 10.0F); // 계산식
        }
        else
        {
            t = float(rawTemp / 10.0F);
        }

        h = float(rawHumi / 10.0F);

        int rainDetectBit = modbus.getResponseBuffer(2);
        // 각 판의 비 감지 상태
        bool plate1Detected = rainDetectBit & (1 << 7);
        bool plate2Detected = rainDetectBit & (1 << 8);
        bool plate3Detected = rainDetectBit & (1 << 9);

        errBit = rainDetectBit & (1 << 10);          // 제품 이상
        bool err_ER1Bit = rainDetectBit & (1 << 12); // ER1 발생 (노이즈, 서지에 의한 메모리오류 -> 공장초기화)

        // 최소 두 개의 감지판에서 비가 감지되는지 확인
        int detectedCount = plate1Detected + plate2Detected + plate3Detected;
        if (detectedCount >= 2)
        {
            isRainy = true; // 비 감지됨
        }
        else
        {
            isRainy = false; // 비 미감지
        }

        DebugSerial.printf("CNT-WJ24 [messageID]: %d | [isRainy]: %s, [TEMP]: %.1f, [HUMI]: %.1f\n", messageID, isRainy ? "Rainy" : "X", t, h);

        if (errBit)
        {
            DebugSerial.println("CNT-WJ24 ERROR!!!");
        }
        if (err_ER1Bit)
        {
            DebugSerial.println("ER1 ERROR!!!");
        }

#if defined(USE_WIFI)
        // Send to Blynk
        Blynk.virtualWrite(V0, t);
        Blynk.virtualWrite(V1, h);
        Blynk.virtualWrite(V2, isRainy ? 1 : 0);
#endif
    }

    else
    {
        DebugSerial.println("[MODBUS] Cannot Read Input Resisters...");
        DebugSerial.println("modbus result: ");
        DebugSerial.println(modbus_result);
    }

#endif

#if defined(TEST_HTTP)
    t = 69.74;
    h = 44; // 정수
    DebugSerial.printf("TEST HTTP [messageID]: %d | [TEMP]: %.1f, [HUMI]: %.1f\n", messageID, t, h);
#endif

    messageID++;
}

void sendSensorData()
{
    getSensorData();

#if defined(USE_WIFI)

#else

    // Use TCP Socket
    /********************************/

    /* 1 :TCP Socket Create ( 0:UDP, 1:TCP ) */
    if (TYPE1SC.socketCreate(1, IPAddr, _PORT) == 0)
    {
        DebugSerial.println("TCP Socket Create!!!");
#if defined(USE_LCD)
        u8x8log.print("TCP Socket Create!!!\n");
#endif
    }

INFO:

    /* 2 :TCP Socket Activation */
    if (TYPE1SC.socketActivate() == 0)
    {
        DebugSerial.println("TCP Socket Activation!!!");
#if defined(USE_LCD)
        u8x8log.print("TCP Socket Activation!!!\n");
#endif
    }

    if (TYPE1SC.socketInfo(sckInfo, sizeof(sckInfo)) == 0)
    {
        DebugSerial.print("Socket Info : ");
        DebugSerial.println(sckInfo);
#if defined(USE_LCD)
        u8x8log.print("Socket Info : ");
        u8x8log.print(sckInfo);
        u8x8log.print("\n");
#endif

        if (strcmp(sckInfo, "ACTIVATED"))
        {
            delay(3000);
            goto INFO;
        }
    }

    /* 3-1 :TCP Socket Send Data */
    String data = "GET /external/api/batch/update";
    data += "?token=" BLYNK_AUTH_TOKEN "&";
    // data += "v0=" + String(t) + "&" + "v1=" + String(h) + "&" + "v2=" + String(ec, 3); // String 기본 소숫점 2자리까지만 변환, second param으로 3자리 명시

    data += "v0=" + String(t) + "&" + "v1=" + String(h);

    data += " HTTP/1.1\r\n";
    data += "Host: sgp1.blynk.cloud\r\n";
    data += "Connection: keep-alive\r\n\r\n";

    // String data = "https//sgp1.blynk.cloud/external/api/batch/update?token="BLYNK_AUTH_TOKEN&;
    // data += "v0=" + String(t) + "&" + "v1=" + String(h) + "&" + "v2=" + String(ec);

    if (TYPE1SC.socketSend(data.c_str()) == 0)
    {
        DebugSerial.print("[HTTP Send] >> ");
        DebugSerial.println(data);
#if defined(USE_LCD)
        u8x8log.print("[HTTP Send] >> ");
        u8x8log.print(data);
        u8x8log.print("\n");
#endif
    }
    else
    {
        DebugSerial.println("Send Fail!!!");
#if defined(USE_LCD)
        u8x8log.print("Send Fail!!!\n");
#endif
    }

    /* 4-1 :TCP Socket Recv Data */
    if (TYPE1SC.socketRecv(recvBuffer, sizeof(recvBuffer), &recvSize) == 0)
    {
        DebugSerial.print("[RecvSize] >> ");
        DebugSerial.println(recvSize);
        DebugSerial.print("[Recv] >> ");
        DebugSerial.println(recvBuffer);
#if defined(USE_LCD)
        u8x8log.print("[RecvSize] >> ");
        u8x8log.print(recvSize);
        u8x8log.print("\n");
        u8x8log.print("[Recv] >> ");
        u8x8log.print(recvBuffer);
        u8x8log.print("\n");
#endif
    }
    else
    {
        DebugSerial.println("Recv Fail!!!");
#if defined(USE_LCD)
        u8x8log.print("Recv Fail!!!\n");
#endif
    }

    /* 5 :TCP Socket DeActivation */
    if (TYPE1SC.socketDeActivate() == 0)
    {
        DebugSerial.println("TCP Socket DeActivation!!!");
#if defined(USE_LCD)
        u8x8log.print("TCP Socket DeActivation!!!\n");
#endif
    }

    if (TYPE1SC.socketInfo(sckInfo, sizeof(sckInfo)) == 0)
    {
        DebugSerial.print("Socket Info : ");
        DebugSerial.println(sckInfo);
#if defined(USE_LCD)
        u8x8log.print("Socket Info : ");
        u8x8log.print(sckInfo);
        u8x8log.print("\n");
#endif
    }

    /* 6 :TCP Socket DeActivation */
    if (TYPE1SC.socketClose() == 0)
    {
        DebugSerial.println("TCP Socket Close!!!");
#if defined(USE_LCD)
        u8x8log.print("TCP Socket Close!!!\n");
#endif
    }

    /* 7 :Detach Network */
    if (TYPE1SC.setCFUN(0) == 0)
    {
        DebugSerial.println("detach Network!!!");
#if defined(USE_LCD)
        u8x8log.print("detach Network!!!\n");
#endif
    }

    delay(10000); // Detach Setup Time : 10sec

#endif

    // 값 초기화
    t = 0;
    h = 0;
    ec = 0;
}

// Initialize SPIFFS
void initSPIFFS()
{
    if (!SPIFFS.begin())
    {
        DebugSerial.println("An error has occurred while mounting SPIFFS");
    }
    DebugSerial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char *path)
{
    DebugSerial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        DebugSerial.println("- failed to open file for reading");
        return String();
    }

    String fileContent;
    while (file.available())
    {
        fileContent = file.readStringUntil('\n');
        break;
    }
    return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message)
{
    DebugSerial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        DebugSerial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        DebugSerial.println("- file written");
    }
    else
    {
        DebugSerial.println("- write failed");
    }
}

// bool isCamConfigDefined()
// {
//     if (camId == "" || slaveMAC == "" || capturePeriod == "")
//     {
//         DebugSerial.println("Undefined Cam ID or slaveMac or Capture Period.");
//         return false;
//     }
//     return true;
// }

// void modbusWork(void *para)
// {
//     char buff[50];

//     while (true)
//     {
//         /* code */
//         bool res = modbus.readHoldingRegisters(1, 0, holdingRegisters, 2);
//         if (res)
//         {
//             DebugSerial.println("modsucces");
//             temper = (float)(holdingRegisters[0] / 10);
//             humi = (float)(holdingRegisters[1] / 10.0f);
//             DebugSerial.print("Temper:");
//             DebugSerial.print(temper);
//             DebugSerial.print("Humi:");
//             DebugSerial.println(humi);

//             int gval = map(temper, -30, 120, -30, 220);
//             if (gval < 0)
//                 gval = 350 + gval;
//             myNex.writeNum("z0.val", gval);

//             int gval2 = map(humi, 0, 100, -30, 220);
//             if (gval2 < 0)
//                 gval2 = 350 + gval2;
//             myNex.writeNum("z1.val", gval2);

//             myNex.writeStr("t0.txt", String(temper));
//             myNex.writeStr("t3.txt", String(humi));

//             sprintf(buff, "&%3.1f&%3.1f&", temper, humi);
//             DebugSerial.println(buff);
//             Udp.beginPacket(IP_Remote, 11000);
//             Udp.write((const uint8_t *)buff, 50);
//             Udp.endPacket();
//         }
//         else
//         {
//             DebugSerial.println("modfail");
//             if (modbus.getTimeoutFlag())
//             {
//                 DebugSerial.println("TimeOut");
//             }
//             if (modbus.getExceptionResponse() > 0)
//             {
//                 DebugSerial.println(modbus.getExceptionResponse());
//             }
//         }

//         Count = random(-30, 120);
//         myNex.writeStr("t2.txt", String(Count));
//         DebugSerial.println("Count=" + String(Count));

//         delay(5000);
//     }
// }

#if defined(USE_WIFI)
void reconnectWifi()
{
    // WiFi 연결 상태 확인
    if (WiFi.status() != WL_CONNECTED)
    {
        unsigned long currentMillis = millis(); // 현재 시간
        if (currentMillis - reconnectPreviousMillis >= interval)
        {
            reconnectPreviousMillis = currentMillis;
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();                // 기존 연결 끊기
            WiFi.begin(WIFI_SSID, WIFI_PASS); // WiFi 재연결 시도

            unsigned long startAttemptTime = millis(); // 타임아웃 시작 시간
            const unsigned long timeout = 10000;       // 10초 타임아웃

            // 비동기 타임아웃 처리
            while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout)
            {
                // delay() 대신 매 루프에서 확인
                if (millis() - reconnectPreviousMillis >= interval)
                {
                    reconnectPreviousMillis = millis();
                    Serial.print(".");
                }
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.println("\nWiFi ReConnected.");
                Serial.println(WiFi.localIP());
                wifiRetryCount = 0; // WiFi 연결 성공 시 재시도 횟수 초기화
            }
            else
            {
                Serial.println("\nFailed to connect to WiFi within timeout.");
                wifiRetryCount++;
                if (wifiRetryCount >= maxRetries)
                {
                    Serial.println("Max WiFi retries reached. Restarting ESP...");
                    ESP.restart(); // 재시도 횟수 초과 시 보드 재시작
                }
            }
        }
    }
    else
    {
        Serial.println("WiFi OK");
        wifiRetryCount = 0; // WiFi가 연결되어 있으면 재시도 횟수 초기화
    }

    // Blynk 연결 상태 확인
    if (!Blynk.connected())
    {
        Serial.println("Lost Blynk server connection");
        unsigned long startAttemptTime = millis(); // 타임아웃 시작 시간
        const unsigned long blynkTimeout = 10000;  // Blynk 타임아웃 10초

        while (!Blynk.connected() && millis() - startAttemptTime < blynkTimeout)
        {
            if (millis() - reconnectPreviousMillis >= interval)
            {
                reconnectPreviousMillis = millis();
                Blynk.connect(); // Blynk 재연결 시도
                Serial.print(".");
            }
        }

        if (Blynk.connected())
        {
            Serial.println("\nBlynk Connected.");
            blynkRetryCount = 0; // Blynk 연결 성공 시 재시도 횟수 초기화
        }
        else
        {
            Serial.println("\nFailed to connect to Blynk server.");
            blynkRetryCount++;
            if (blynkRetryCount >= maxRetries)
            {
                Serial.println("Max Blynk retries reached. Considering further actions...");
                // 여기서 추가적인 처리 로직을 넣을 수 있습니다.
                // 예: 경고 메시지 출력, 장비 재시작 등
            }
        }
    }
    else
    {
        Serial.println("Blynk OK");
        blynkRetryCount = 0; // Blynk가 연결되어 있으면 재시도 횟수 초기화
    }
}
#endif

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

#if defined(USE_WIFI)
    DebugSerial.begin(SERIAL_BR);
#else
    /* CAT.M1 Modem PowerUp sequence */
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
#if defined(USE_LCD)
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
    u8x8log.setRedrawMode(
        1); // 0: Update screen with newline, 1: Update screen for every char
#endif

    // put your setup code here, to run once:
    // M1Serial.begin(SERIAL_BR);  // ESP32, Arduino ESP32 v3 업데이트로 미사용
    M1Serial.begin(SERIAL_BR, SERIAL_8N1, 16, 17); // Arduino ESP32 v3
    DebugSerial.begin(SERIAL_BR);

    //****************************************************************************************************************************************

    // // 설정 안된 상태: AP모드 진입(wifi config reset): softAP() 메소드
    // if (!isWMConfigDefined())
    // {
    //     // Connect to Wi-Fi network with SSID and pass
    //     DebugSerial.println("Setting AP (Access Point)");
    //     // NULL sets an open Access Point
    //     WiFi.softAP("Daon Blynk ESP32 Manager", NULL);

    //     IPAddress IP = WiFi.softAPIP(); // Software enabled Access Point : 가상 라우터, 가상의 액세스 포인트
    //     DebugSerial.print("AP IP address: ");
    //     DebugSerial.println(IP);

    //     // Web Server Root URL
    //     // GET방식
    //     server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    //               { request->send(SPIFFS, "/wifimanager.html", "text/html"); });

    //     server.serveStatic("/", SPIFFS, "/");
    //     // POST방식
    //     server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
    //               {
    //   int params = request->params();
    //   for(int i=0;i<params;i++){
    //     AsyncWebParameter* p = request->getParam(i);
    //     if(p->isPost()){
    //       // HTTP POST houseId value
    //       if (p->name() == PARAM_INPUT_1) {
    //         houseId = p->value().c_str();
    //         DebugSerial.print("Cam ID set to: ");
    //         DebugSerial.println(houseId);
    //         // Write file to save value
    //         writeFile(SPIFFS, houseIdPath, houseId.c_str());
    //       }
    //       // HTTP POST ssid value
    //       if (p->name() == PARAM_INPUT_2)
    //       {
    //         ssid = p->value().c_str();
    //         DebugSerial.print("SSID set to: ");
    //         DebugSerial.println(ssid);
    //         // Write file to save value
    //         writeFile(SPIFFS, ssidPath, ssid.c_str());
    //       }
    //       // HTTP POST pass value
    //       if (p->name() == PARAM_INPUT_3)
    //       {
    //         pass = p->value().c_str();
    //         DebugSerial.print("Password set to: ");
    //         DebugSerial.println(pass);
    //         // Write file to save value
    //         writeFile(SPIFFS, passPath, pass.c_str());
    //       }
    //       DebugSerial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    //     }
    //   }
    //   // ESP가 양식 세부 정보를 수신했음을 알 수 있도록 일부 텍스트가 포함된 응답을 send
    //   request->send(200, "text/plain", "Done. ESP will restart.");
    //   delay(3000);
    //   ESP.restart(); });
    //     server.begin();
    // }

    // // 설정 완료 후: wifi 연결
    // else
    // {
    //     DebugSerial.println("LED Panel STARTED");
    //     initLED();

    //     initWiFi();
    //     DebugSerial.print("RSSI: ");
    //     DebugSerial.println(WiFi.RSSI());

    //     configTime(3600 * timeZone, 3600 * summerTime, ntpServer);
    //     printLocalTime();
    //     Udp.begin(11000);

    //     xTaskCreatePinnedToCore(timeWork, "timeWork", 10000, NULL, 0, &thWork, 0);
    //     allowsLoop = true;
    // }

    //****************************************************************************************************************************************

    DebugSerial.println("TYPE1SC Module Start!!!");
#if defined(USE_LCD)
    u8x8log.print("TYPE1SC Module Start!!!\n");
#endif

    extAntenna();

    /* TYPE1SC Module Initialization */
    if (TYPE1SC.init())
    {
        DebugSerial.println("TYPE1SC Module Error!!!");
#if defined(USE_LCD)
        u8x8log.print("TYPE1SC Module Error!!!\n");
#endif
    }

    /* Network Registration Check */
    while (TYPE1SC.canConnect() != 0)
    {
        DebugSerial.println("Network not Ready !!!");
#if defined(USE_LCD)
        u8x8log.print("Network not Ready!!!\n");
#endif

        delay(2000);
    }

    /* Get Time (GMT, (+36/4) ==> Korea +9hour) */
    char szTime[32];
    if (TYPE1SC.getCCLK(szTime, sizeof(szTime)) == 0)
    {
        DebugSerial.print("Time : ");
        DebugSerial.println(szTime);
#if defined(USE_LCD)
        u8x8log.print("Time : ");
        u8x8log.print(szTime);
        u8x8log.print("\n");
#endif
    }
    delay(1000);

    DebugSerial.println("TYPE1SC Module Ready!!!");
#if defined(USE_LCD)
    u8x8log.print("TYPE1SC Module Ready!!!\n");
#endif

    int ipErrCount = 0;
    /* Enter a DNS address to get an IP address */
    while (1)
    {

        if (TYPE1SC.getIPAddr("sgp1.blynk.cloud", IPAddr, sizeof(IPAddr)) == 0)
        {
            DebugSerial.print("Blynk IP Address : ");
            DebugSerial.println(IPAddr);
#if defined(USE_LCD)
            u8x8log.print("Blynk IP Address : ");
            u8x8log.print(IPAddr);
            u8x8log.print("\n");
#endif

            break;
        }
        else
        {
            if (ipErrCount++ > 60)
            {
                DebugSerial.println("Cannot Connect to Blynk... ESP Restart.");
                ESP.restart();
            }
            DebugSerial.print("Blynk IP Address Error!!!");
            DebugSerial.print("; count: ");
            DebugSerial.println(ipErrCount);
#if defined(USE_LCD)
            u8x8log.print("Blynk IP Address Error!!!");
            u8x8log.print("; count: ");
            u8x8log.print(ipErrCount);
            u8x8log.print("\n");

#endif
        }
        delay(2000);
    }

#endif

    // RS485 제어 핀 초기화; modbus.begin() 이전 반드시 선언해 주어야!
    pinMode(dePin, OUTPUT);
    pinMode(rePin, OUTPUT);

    // RE 및 DE를 비활성화 상태로 설정 (RE=LOW, DE=LOW)
    digitalWrite(dePin, LOW);
    digitalWrite(rePin, LOW);

    // RS485 Setup
    SerialPort.begin(9600, SERIAL_8N1, rxPin, txPin); // RXD1 : 33, TXD1 : 32
    modbus.begin(SLAVE_ID, SerialPort);

    // Callbacks allow us to configure the RS485 transceiver correctly
    // Auto FlowControl - NULL
    modbus.preTransmission(preTransmission);
    modbus.postTransmission(postTransmission);

    DebugSerial.println("Modbus Settings Done.");

#if defined(USE_WIFI)
    // Begin Blynk
    Blynk.begin(auth, WIFI_SSID, WIFI_PASS);
    // 함수 주기 실행
    timer.setInterval(SENSING_PERIOD, getSensorData);
    timer.setInterval(5 * 60 * 1000, reconnectWifi); // 5분마다 wifi 상태 점검 및 복구 시도

    DebugSerial.println("Blynk WiFi Settings Done.");
#else
    sendSensorData();

    DebugSerial.println("Going to sleep now");
    esp_sleep_enable_timer_wakeup(DEEPSLEEP_PERIOD_SEC * 1000000); // 10min; 1s = 1,000,000us
    esp_deep_sleep_start();
#endif
}

void loop()
{

#if defined(USE_WIFI)
    Blynk.run();
    timer.run();
#else

#endif
}
