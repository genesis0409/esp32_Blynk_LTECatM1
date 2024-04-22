/*
 * Before Use, Select Option:
 * 1. TYPE1SC: EXT_ANT_ON
 * 2. Blynk: USE_WIFI
 * 3. RS485: EC_SENSING_MODE
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

#include <ModbusRTUMaster.h>

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
// #define USE_WIFI // To disable, Use HTTP Request

// Use LCD? *********************************************************************************************
// #define USE_LCD

// Soil Moisture + Temp + EC ****************************************************************************
#define EC_SENSING_MODE // To disable, change to annotation.
// ******************************************************************************************************

// TYPE1SC setting **************************************************************************************
#define SERIAL_BR 115200

#define PWR_PIN 5
#define RST_PIN 18
#define WAKEUP_PIN 19
#define EXT_ANT 4

#define DebugSerial Serial
#define M1Serial Serial2 // ESP32

String APN = "simplio.apn";

RTC_DATA_ATTR TYPE1SC TYPE1SC(M1Serial, DebugSerial, PWR_PIN, RST_PIN, WAKEUP_PIN);
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
#define EXT_ANT_ON 1
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
#define SLAVE_ID 1
#define START_ADDRESS 0
#define QUANTITY 2
#define SCAN_RATE 60000          // 60만: 10분; 미사용
#define DEEPSLEEP_PERIOD_SEC 600 // 600초

unsigned long currentMillis = 0;
unsigned long previousMillis = 0; // Stores last time using Reference time

RTC_DATA_ATTR unsigned int messageID = 0;
RTC_DATA_ATTR bool errBit; // TM100 센서에러비트; 0:에러없음, 1:센서에러
#if defined(EC_SENSING_MODE)
RTC_DATA_ATTR float t;
RTC_DATA_ATTR float soil_m; // Soil Moisture
RTC_DATA_ATTR float ec;     // 전기 전도도
#else
RTC_DATA_ATTR float t;
RTC_DATA_ATTR float h;
#endif

RTC_DATA_ATTR const uint8_t rxPin = 33; // RX-RO
RTC_DATA_ATTR const uint8_t txPin = 32; // TX-DI
RTC_DATA_ATTR const uint8_t dePin = 25; // DE+RE

RTC_DATA_ATTR ModbusRTUMaster modbus(Serial1, dePin); // serial port, driver enable pin for rs-485 (optional)
#if defined(EC_SENSING_MODE)
RTC_DATA_ATTR uint16_t holdingRegisters[3] = {0xFF, 0xFF, 0xFF};
RTC_DATA_ATTR uint16_t inputRegisters[3] = {0xFF, 0xFF, 0xFF};
#else
RTC_DATA_ATTR uint16_t holdingRegisters[2] = {0xFF, 0xFF};
#endif

// 메소드 선언부 *******************************************************************************************
void getSensorData();
void sendSensorData();

void initSPIFFS();                                                 // Initialize SPIFFS
String readFile(fs::FS &fs, const char *path);                     // Read File from SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message); // Write file to SPIFFS
bool isCamConfigDefined();                                         // Is Cam Configuration Defiend?

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

void getSensorData()
{
#if defined(EC_SENSING_MODE)
    while (true)
    {
        // CONOTEC CNT-TM100
        if (modbus.readInputRegisters(10, 0, inputRegisters, 3))
        {
            t = inputRegisters[0] / 10.0;
            soil_m = inputRegisters[1] / 10.0;
            errBit = inputRegisters[2];
            Serial.printf("RK520-02 [messageID]: %d | [TEMP]: %.1f, [Moisture]: %.1f, [ErrBit]: %d\n", messageID, t, soil_m, errBit);

            if (!errBit)
            {
                Serial.println("Sensor Open ERROR!!!");
            }

            break;
        }
        else
        {
            Serial.println("[MODBUS] Cannot Read Holding Resisters...");
            if (modbus.getTimeoutFlag())
            {
                Serial.println("TimeOut");
            }
            if (modbus.getExceptionResponse() > 0)
            {
                Serial.print("getExceptionResponse: ");
                Serial.println(modbus.getExceptionResponse());
            }

            delay(5000);
        }
    }

#else
    while (true)
    {
        if (modbus.readHoldingRegisters(1, 0, holdingRegisters, 2))
        {
            t = holdingRegisters[0] / 10.0;
            h = holdingRegisters[1] / 10.0;
            Serial.printf("TZ-THT02 [messageID]: %d | [TEMP]: %.1f, [HUMI]: %.1f\n", messageID, t, h);

            // Sent to Blynk
            Blynk.virtualWrite(V0, t);
            Blynk.virtualWrite(V1, h);

            break;
        }
        else
        {
            Serial.println("[MODBUS] Cannot Read Holding Resisters...");
            if (modbus.getTimeoutFlag())
            {
                Serial.println("TimeOut");
            }
            if (modbus.getExceptionResponse() > 0)
            {
                Serial.print("getExceptionResponse: ");
                Serial.println(modbus.getExceptionResponse());
            }

            delay(5000);
        }
    }

#endif

    messageID++;
}

void sendSensorData()
{
    getSensorData();

#if defined(USE_WIFI)
    // Sent to Blynk
    Blynk.virtualWrite(V0, t);
    Blynk.virtualWrite(V1, soil_m);
    Blynk.virtualWrite(V2, ec);
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
    // data += "v0=" + String(t) + "&" + "v1=" + String(soil_m) + "&" + "v2=" + String(ec, 3); // String 기본 소숫점 2자리까지만 변환, second param으로 3자리 명시

    data += "v0=" + String(t) + "&" + "v1=" + String(soil_m);

    data += " HTTP/1.1\r\n";
    data += "Host: sgp1.blynk.cloud\r\n";
    data += "Connection: keep-alive\r\n\r\n";

    // String data = "https//sgp1.blynk.cloud/external/api/batch/update?token=" BLYNK_AUTH_TOKEN;
    // data += "v0=" + String(t) + "&" + "v1=" + String(soil_m) + "&" + "v2=" + String(ec);

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

    //     /* 3-2 :TCP Socket Send Data: Event Message */
    //     // 온도 경고 알림
    //     if (t > 50)
    //     {
    //         String data = "GET /external/api/logEvent";
    //         data += "?token=" BLYNK_AUTH_TOKEN;
    //         data += "&code=CODE_HighTempAlert";
    //         data += "&description=TEMP_WARNING:HOT"; // 앱에서 한글 미지원

    //         data += " HTTP/1.1\r\n";
    //         data += "Host: sgp1.blynk.cloud\r\n";
    //         data += "Connection: keep-alive\r\n\r\n";

    //         // String data = "https//sgp1.blynk.cloud/external/api/batch/update?token=" BLYNK_AUTH_TOKEN;
    //         // data += "v0=" + String(t) + "&" + "v1=" + String(soil_m) + "&" + "v2=" + String(ec);

    //         if (TYPE1SC.socketSend(data.c_str()) == 0)
    //         {
    //             DebugSerial.print("[HTTP Send] >> ");
    //             DebugSerial.println(data);
    // #if defined(USE_LCD)
    //             u8x8log.print("[HTTP Send] >> ");
    //             u8x8log.print(data);
    //             u8x8log.print("\n");
    // #endif
    //         }
    //         else
    //         {
    //             DebugSerial.println("Send Fail!!!");
    // #if defined(USE_LCD)
    //             u8x8log.print("Send Fail!!!\n");
    // #endif
    //         }

    //         /* 4-2 :TCP Socket Recv Data */
    //         if (TYPE1SC.socketRecv(recvBuffer, sizeof(recvBuffer), &recvSize) == 0)
    //         {
    //             DebugSerial.print("[RecvSize] >> ");
    //             DebugSerial.println(recvSize);
    //             DebugSerial.print("[Recv] >> ");
    //             DebugSerial.println(recvBuffer);
    // #if defined(USE_LCD)
    //             u8x8log.print("[RecvSize] >> ");
    //             u8x8log.print(recvSize);
    //             u8x8log.print("\n");
    //             u8x8log.print("[Recv] >> ");
    //             u8x8log.print(recvBuffer);
    //             u8x8log.print("\n");
    // #endif
    //         }
    //         else
    //         {
    //             DebugSerial.println("Recv Fail!!!");
    // #if defined(USE_LCD)
    //             u8x8log.print("Recv Fail!!!\n");
    // #endif
    //         }
    //     }

    //     // 온도 경고 알림
    //     else if (t < 30)
    //     {
    //         String data = "GET /external/api/logEvent";
    //         data += "?token=" BLYNK_AUTH_TOKEN;
    //         data += "&code=CODE_LowTempAlert";
    //         data += "&description=TEMP_WARNING:COLD"; // 앱에서 한글 미지원

    //         data += " HTTP/1.1\r\n";
    //         data += "Host: sgp1.blynk.cloud\r\n";
    //         data += "Connection: keep-alive\r\n\r\n";

    //         // String data = "https//sgp1.blynk.cloud/external/api/batch/update?token=" BLYNK_AUTH_TOKEN;
    //         // data += "v0=" + String(t) + "&" + "v1=" + String(soil_m) + "&" + "v2=" + String(ec);

    //         if (TYPE1SC.socketSend(data.c_str()) == 0)
    //         {
    //             DebugSerial.print("[HTTP Send] >> ");
    //             DebugSerial.println(data);
    // #if defined(USE_LCD)
    //             u8x8log.print("[HTTP Send] >> ");
    //             u8x8log.print(data);
    //             u8x8log.print("\n");
    // #endif
    //         }
    //         else
    //         {
    //             DebugSerial.println("Send Fail!!!");
    // #if defined(USE_LCD)
    //             u8x8log.print("Send Fail!!!\n");
    // #endif
    //         }

    //         /* 4-2 :TCP Socket Recv Data */
    //         if (TYPE1SC.socketRecv(recvBuffer, sizeof(recvBuffer), &recvSize) == 0)
    //         {
    //             DebugSerial.print("[RecvSize] >> ");
    //             DebugSerial.println(recvSize);
    //             DebugSerial.print("[Recv] >> ");
    //             DebugSerial.println(recvBuffer);
    // #if defined(USE_LCD)
    //             u8x8log.print("[RecvSize] >> ");
    //             u8x8log.print(recvSize);
    //             u8x8log.print("\n");
    //             u8x8log.print("[Recv] >> ");
    //             u8x8log.print(recvBuffer);
    //             u8x8log.print("\n");
    // #endif
    //         }
    //         else
    //         {
    //             DebugSerial.println("Recv Fail!!!");
    // #if defined(USE_LCD)
    //             u8x8log.print("Recv Fail!!!\n");
    // #endif
    //         }
    //     }

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

#endif
}

// Initialize SPIFFS
void initSPIFFS()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("An error has occurred while mounting SPIFFS");
    }
    Serial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("- failed to open file for reading");
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
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("- file written");
    }
    else
    {
        Serial.println("- write failed");
    }
}

// bool isCamConfigDefined()
// {
//     if (camId == "" || slaveMAC == "" || capturePeriod == "")
//     {
//         Serial.println("Undefined Cam ID or slaveMac or Capture Period.");
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
//             Serial.println("modsucces");
//             temper = (float)(holdingRegisters[0] / 10);
//             humi = (float)(holdingRegisters[1] / 10.0f);
//             Serial.print("Temper:");
//             Serial.print(temper);
//             Serial.print("Humi:");
//             Serial.println(humi);

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
//             Serial.println(buff);
//             Udp.beginPacket(IP_Remote, 11000);
//             Udp.write((const uint8_t *)buff, 50);
//             Udp.endPacket();
//         }
//         else
//         {
//             Serial.println("modfail");
//             if (modbus.getTimeoutFlag())
//             {
//                 Serial.println("TimeOut");
//             }
//             if (modbus.getExceptionResponse() > 0)
//             {
//                 Serial.println(modbus.getExceptionResponse());
//             }
//         }

//         Count = random(-30, 120);
//         myNex.writeStr("t2.txt", String(Count));
//         Serial.println("Count=" + String(Count));

//         delay(5000);
//     }
// }

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
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
#if defined(USE_LCD)
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
    u8x8log.setRedrawMode(
        1); // 0: Update screen with newline, 1: Update screen for every char
#endif

    // put your setup code here, to run once:
    M1Serial.begin(SERIAL_BR);
    DebugSerial.begin(SERIAL_BR);

    //****************************************************************************************************************************************

    // // 설정 안된 상태: AP모드 진입(wifi config reset): softAP() 메소드
    // if (!isWMConfigDefined())
    // {
    //     // Connect to Wi-Fi network with SSID and pass
    //     Serial.println("Setting AP (Access Point)");
    //     // NULL sets an open Access Point
    //     WiFi.softAP("Daon Blynk ESP32 Manager", NULL);

    //     IPAddress IP = WiFi.softAPIP(); // Software enabled Access Point : 가상 라우터, 가상의 액세스 포인트
    //     Serial.print("AP IP address: ");
    //     Serial.println(IP);

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
    //         Serial.print("Cam ID set to: ");
    //         Serial.println(houseId);
    //         // Write file to save value
    //         writeFile(SPIFFS, houseIdPath, houseId.c_str());
    //       }
    //       // HTTP POST ssid value
    //       if (p->name() == PARAM_INPUT_2)
    //       {
    //         ssid = p->value().c_str();
    //         Serial.print("SSID set to: ");
    //         Serial.println(ssid);
    //         // Write file to save value
    //         writeFile(SPIFFS, ssidPath, ssid.c_str());
    //       }
    //       // HTTP POST pass value
    //       if (p->name() == PARAM_INPUT_3)
    //       {
    //         pass = p->value().c_str();
    //         Serial.print("Password set to: ");
    //         Serial.println(pass);
    //         // Write file to save value
    //         writeFile(SPIFFS, passPath, pass.c_str());
    //       }
    //       Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
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
    //     Serial.println("LED Panel STARTED");
    //     initLED();

    //     initWiFi();
    //     Serial.print("RSSI: ");
    //     Serial.println(WiFi.RSSI());

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

    /* Network Regsistraiton Check */
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

#if defined(USE_WIFI)
    // Begin Blynk
    Blynk.begin(auth, "dinfo", "daon7521");
    // 함수 주기 실행
    timer.setInterval(SCAN_RATE, sendSensorData);
#else

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
            DebugSerial.println("Blynk IP Address Error!!!");
#if defined(USE_LCD)
            u8x8log.print("Blynk IP Address Error!!!\n");
#endif
        }
        delay(2000);
    }

#endif

    // RS485 Setup
    modbus.setTimeout(12000);
    modbus.begin(9600, SERIAL_8N1, rxPin, txPin); // 직렬 전송 설정 (baud, config, rxPin, txPin, invert)
                                                  // default config : SERIAL_8N1; { 데이터비트 8, 패리티 없음, 1 정지 비트}; E: 짝수 패리티, O: 홀수 패리티
                                                  // rxPin: 직렬 데이터 수신 핀; txPin: 직렬  데이터 전송 핀 (uint8_t)

    sendSensorData();

    /* 7 :Detach Network */
    if (TYPE1SC.setCFUN(0) == 0)
    {
        DebugSerial.println("detach Network!!!");
#if defined(USE_LCD)
        u8x8log.print("detach Network!!!\n");
#endif
    }
    delay(10000); // Detach Setup Time : 10sec

    Serial.println("Going to sleep now");
    esp_sleep_enable_timer_wakeup(DEEPSLEEP_PERIOD_SEC * 1000000); // 10min; 1s = 1,000,000us
    esp_deep_sleep_start();
}

void loop()
{

#if defined(USE_WIFI)
    Blynk.run();
    timer.run();
#else
    // if (firstRun)
    // {
    //     sendSensorData();
    //     firstRun = false;
    // }

    // currentMillis = millis();
    // if (currentMillis - previousMillis >= SCAN_RATE)
    // {
    //     previousMillis = currentMillis;

    //     sendSensorData();
    // }
#endif
}
