/*
// CAPTURE AND SEND IMAGE OVER ESP NOW
// Code by: Tal Ofer
// https://github.com/talofer99/ESP32CAM-Capture-and-send-image-over-esp-now
//
// This is the camera portion of the code.
//
// for more information
// https://www.youtube.com/watch?v=0s4Bm9Ar42U
*/
#include "Arduino.h"
#include "FS.h" // SD Card ESP32
// #include "SD_MMC.h"           // SD Card ESP32
#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
#include "driver/rtc_io.h"

#include <esp_now.h>
#include <WiFi.h>
#define ONBOADLED 4
// #define RXPIN 3
#include "esp_camera.h"

// Select File System, with 'board.build.filesystem' in 'platformio.ini'
#include "SPIFFS.h" // Fast
// #include "LittleFS.h" // Slow

#if defined(LittleFS)
#define SPIFFS LITTLEFS
#endif

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

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

unsigned long currentMillis = 0;
unsigned long previousMillis = 0; // Stores last time using Reference time
unsigned long tempMillis = 0;

// #define CONFIG_ARDUINO_LOOP_STACK_SIZE 16 * 1024 // 16KB
// uxTaskGetStackHighWaterMark
// #define INCLUDE_uxTaskGetStackHighWaterMark 1
// UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

// =========================================================
// ****************** Select camera model ******************
// =========================================================

// #define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define TTGO_T_Camera_plus // Has PSRAM

// =========================================================
// ****************** Select camera model ******************
// =========================================================

// Pin definition for CAMERA_MODEL_AI_THINKER, esp32 cam 핀 배열
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Pin definition for TTGO T-Camera plus (cv-jh640-750v2 : 2MP cam)
#elif defined(TTGO_T_Camera_plus)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 4
#define SIOD_GPIO_NUM 18 // SDA
#define SIOC_GPIO_NUM 23 // SCL

#define Y9_GPIO_NUM 36
#define Y8_GPIO_NUM 37
#define Y7_GPIO_NUM 38
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 35
#define Y4_GPIO_NUM 26
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM 27
#define PCLK_GPIO_NUM 25

// SD card
#define MISO_PIN_NUM 22 // Master In, Slave Out
#define MOSI_PIN_NUM 19 // Master Out, Slave In
#define SCLK_PIN_NUM 21

#define SDCARD_CS_PIN_NUM 0 // Chip Select

// Display
#define TFT_MISO_PIN_NUM 22
#define TFT_MOSI_PIN_NUM 19
#define TFT_SCLK_PIN_NUM 21

#define TFT_CS_PIN_NUM 12
#define TFT_DC_PIN_NUM 15
#define TFT_BK_PIN_NUM 2

#else
#error "Camera model not selected"
#endif

#define fileDatainMessage 240.0 // ESPNOW 최대 250Bytes전송 고려 data cut-down
#define UARTWAITHANDSHACK 1000

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 1

// 시리얼 모니터에 많은 내용을 출력하는 경우 메모리 공간을 많이 차지할 수 있습니다. 문자열은 플래시 메모리에 포함된 뒤 출력을 하기 위해 RAM의 일정 부분을 차지하게 됩니다.
// 아두이노가 플래시 메모리 영역에 있는 문자열을 바로 사용할 수 있다면 RAM를 소모하지 않아 메모리를 절약할 수 있습니다. 이때 사용하는 지시어가 F()입니다.
// using F() Macro
#define Fprint(x) print(F(x))
#define Fprintln(x) println(F(x))

// for esp now connect
unsigned long lastConnectNowAttempt = 0;
unsigned long nextConnectNowGap = 1000;
bool isPaired = 0;

// for photo name
int pictureNumber = 1;
const uint32_t n_zero = 7; // zero padding
bool takeNextPhotoFlag = false;

// for photo transmit
int currentTransmitCurrentPosition = 0;
int currentTransmitTotalPackages = 0;
byte sendNextPackageFlag = 0;
String fileName = "/1.jpg";

// for connection type
// bool useUartRX = 0;

// 메소드 선언부
void takePhoto();
// void initSD(); // Disabled -> using SPIFFS
void initCamera();
void startTransmit();
void sendNextPackage();
void sendData(const uint8_t *dataArray, uint8_t dataArrayLength);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void InitESPNow();
void ScanAndConnectToSlave();
bool manageSlave();
void deletePeer();
// void blinkIt(int delayTime, int times);

void initSPIFFS();                                                 // Initialize SPIFFS
String readFile(fs::FS &fs, const char *path);                     // Read File from SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message); // Write file to SPIFFS
bool isCamConfigDefined();                                         // Is Cam Configuration Defiend?
bool allowsLoop = false;                                           // loop() DO or NOT

float getCurrFreeHeapRatio(); // 가용 힙 비율

// espnow 지연시간 검증 변수
int packageArrayNum = 0;

void setup()
{

  // NEEDED ????
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
  // start serial

  Serial.begin(115200);

  Serial.printf("총 힙: %d\n", ESP.getHeapSize());
  Serial.printf("사용 가능한 힙: %d\n", ESP.getFreeHeap());
  Serial.printf("가용 힙 비율 : %.1f\n", getCurrFreeHeapRatio());
  Serial.printf("총 PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("여유 PSRAM: %d\n", ESP.getFreePsram());

  Serial.printf("Curr freeHeap Size1: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING
  initSPIFFS();                                                           // init SPIFFS

  Serial.printf("Curr freeHeap Size2: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

  // Load values saved in SPIFFS
  camId = readFile(SPIFFS, camIdPath);
  slaveMAC = readFile(SPIFFS, slaveMACPath);
  capturePeriod = readFile(SPIFFS, capturePeriodPath);
  Serial.Fprint("CamID in SPIFFS: ");
  Serial.println(camId);
  Serial.Fprint("SlaveMAC in SPIFFS: ");
  Serial.println(slaveMAC);
  Serial.Fprint("CapturePeriod in SPIFFS: ");
  Serial.println(capturePeriod);

  Serial.printf("Curr freeHeap Size3: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

  // AP모드 진입(cam config reset): softAP() 메소드
  if (!isCamConfigDefined())
  {
    // Connect to Wi-Fi network with SSID and password
    Serial.Fprintln("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER Master0", NULL);

    IPAddress IP = WiFi.softAPIP(); // Software enabled Access Point : 가상 라우터, 가상의 액세스 포인트
    Serial.Fprint("AP IP address: ");
    Serial.println(IP);

    // Print Chip Info.
    Serial.printf("Total Heap Size = %d\n\n", ESP.getHeapSize());

    Serial.printf("Curr freeHeap Size4: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

    // Web Server Root URL
    // GET방식
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/wifimanager.html", "text/html"); });

    server.serveStatic("/", SPIFFS, "/");
    // POST방식
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
              {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST camId value
          if (p->name() == PARAM_INPUT_1) {
            camId = p->value().c_str();
            Serial.Fprint("Cam ID set to: ");
            Serial.println(camId);
            // Write file to save value
            writeFile(SPIFFS, camIdPath, camId.c_str());
          }
          // HTTP POST slaveMAC value
          if (p->name() == PARAM_INPUT_2) {
            slaveMAC = p->value().c_str();
            Serial.Fprint("Dest. MAC set to: ");
            Serial.println(slaveMAC);
            // Write file to save value
            writeFile(SPIFFS, slaveMACPath, slaveMAC.c_str());
          }
          // HTTP POST capturePeriod value
          if (p->name() == PARAM_INPUT_3) {
            capturePeriod = p->value().c_str();
            Serial.Fprint("Capture Period set to: ");
            Serial.println(capturePeriod);
            // Write file to save value
            writeFile(SPIFFS, capturePeriodPath, capturePeriod.c_str());
          }
          
          Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      // ESP가 양식 세부 정보를 수신했음을 알 수 있도록 일부 텍스트가 포함된 응답을 send
      request->send(200, "text/plain", "Done. ESP will restart, Take photo periodcally, and then Send it to Slave Device: " + slaveMAC);
      Serial.printf("Curr freeHeap Size5: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING
      delay(3000);
      ESP.restart(); });
    server.begin();
    Serial.printf("Curr freeHeap Size6: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING
  }
  else
  {
    Serial.Fprintln("CAMERA MASTER STARTED"); // tarted : 시작되다; 자동사인듯?
    initCamera();                             // init camera
    // initSD();                                 // init sd

    Serial.printf("Curr freeHeap Size7: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

    /*
    // Not use

    // init onboad led
    pinMode(ONBOADLED, OUTPUT);
    digitalWrite(ONBOADLED, LOW);

    // we now test to see if we got serial communication
    unsigned long testForUart = millis();
    Serial.Fprint("WAIT UART");
    while (testForUart + UARTWAITHANDSHACK > millis() && !Serial.available())
    {
      Serial.Fprint(".");
      delay(50);
    }

    if (Serial.available())
    {
      Serial.Fprintln("We are using Serial!!");
      while (Serial.available())
      {
        Serial.Fprintln(Serial.read());
      }
      // useUartRX = 1;
    }
    */
    /*
        if (1) //! useUartRX
        {*/
    // set RX as pullup for safety
    // pinMode(RXPIN, INPUT_PULLUP);
    // Serial.Fprintln("We are using the button");

    // Set device in STA mode to begin with
    WiFi.mode(WIFI_STA);
    // This is the mac address of the Master in Station Mode
    Serial.Fprint("STA MAC: ");
    Serial.println(WiFi.macAddress());

    Serial.printf("Curr freeHeap Size8: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

    // Init ESPNow with a fallback logic
    InitESPNow();
    Serial.printf("Curr freeHeap Size9: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    /*}*/
    allowsLoop = true;
    Serial.printf("Curr freeHeap Size10: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING
  }
}

void loop()
{

  // if we are:
  // 1. NOT USING UART AS CONNECTION (ESP NOW WORKING)
  // 2. NOT PARIED
  // 3. OUR LAST CONNECT ATTMEPT WAS OVER DUE 마지막 연결 시도가 종료된 상태면
  if (!isPaired && lastConnectNowAttempt + nextConnectNowGap < millis() && allowsLoop) //! useUartRX &&
  {
    // 재시도
    Serial.Fprintln("NOT CONNECTED -> TRY TO CONNECT");
    ScanAndConnectToSlave();
    // if we connected
    if (isPaired)
    {
      // blinkIt(150, 2);
    }
    else
    {
      nextConnectNowGap *= 2; // 다음 연결 시도 간격을 두배로 1, 2, 4...
      // blinkIt(150, 3);        // blink 3 times
    }

    // save last attempt
    lastConnectNowAttempt = millis();
  }

  // if we are :
  // 1. NOT using the UART - IE we control from the button
  // 2. The button is LOW (pressed)
  // 3. we are currently not have currentTransmitTotalPackages set
  // 4. the sendNextPackageFlag is not set.
  // 버튼 눌렀을 때 사진 찍는 flow -> 주기 자동촬영으로 개선 예정

  currentMillis = millis();
  if (currentMillis - previousMillis >= (long)capturePeriod.toInt() && !currentTransmitTotalPackages && !sendNextPackageFlag && allowsLoop) //! useUartRX && !digitalRead(RXPIN) &&
  {
    // Serial.printf("Interval Check1: %lu\n", currentMillis - previousMillis);
    previousMillis = currentMillis;
    takeNextPhotoFlag = true;
  }
  // if the sendNextPackageFlag is set:
  if (sendNextPackageFlag)
  {
    sendNextPackage(); // 다음 패키지를 보냄:
  }

  // if takeNextPhotoFlag is set: 사진 촬영 변수
  if (takeNextPhotoFlag && allowsLoop)
  {
    takePhoto(); // >> startTransmit()로 이어짐 (currentTransmitTotalPackages 변수 설정)
                 // >> startTransmit() >> sendData() >> esp_now_send();

    // currentMillis = millis();
    // Serial.printf("Interval Check2: %lu\n", currentMillis - previousMillis);
  }

  /*
    // we only read serial if we use the uart: 직렬통신으로 serial에 명령 입력 가능
    if (Serial.available()) //  && useUartRX
    {
      switch (Serial.read())
      {
      case 'p':
      case 'P':
        takeNextPhotoFlag = true;
        break;
      case 's':
      case 'S':
        ScanAndConnectToSlave();
        break;
      case 't':
      case 'T':
        startTransmit();
        break;
      default:
        Serial.Fprintln("not supported!!!");
        break;
      } // end switch
    }   // end if
    */
}

/* ***************************************************************** */
/*                  CAMERA RELATED FUNCTIONS                         */
/* ***************************************************************** */

/* ***************************************************************** */
/* TAKE PHOTO                                                        */
/* ***************************************************************** */
// 사진 촬영 후 파일로 저장, 그리고 전송
void takePhoto()
{
  takeNextPhotoFlag = false;

  camera_fb_t *fb = NULL; // 카메라 버퍼 구조체

  // Take Picture with Camera
  fb = esp_camera_fb_get(); // 포인터에서 프레임 버퍼를 얻음

  if (!fb) // 버퍼가 비어있으면 카매라 캡쳐 실패
  {
    Serial.Fprintln("Camera capture failed");
    return;
  }

  /*
    // 동일 이름으로 작성해보자 -> SD 카드에서 파일이름 14쯤부터 지연시간 누적
    // 기능추가: 파일명 zero padding
    std::string old_str = std::to_string(pictureNumber);
    std::string new_str = std::string(n_zero - std::min(n_zero, old_str.length()), '0') + old_str;

  // Path where new picture will be saved in SD Card
  std::string path = "/picture" + new_str + ".jpg";
  */

  // Path where new picture will be saved in SD Card -> Flash Memory
  String path = "/" + String(1) + ".jpg";

  Serial.printf("Picture file name: %s\n", path.c_str());

  // SD card parts -> SPIFFS 플래시 기록으로 대체
  // fs::FS &fs = SD_MMC;
  // fs.remove(path.c_str()); // 이미 있으면 지우고

  SPIFFS.remove(path.c_str()); // 이미 있으면 지우고

  // File file = fs.open(path.c_str(), FILE_WRITE);  // SD Card
  File file = SPIFFS.open(path.c_str(), FILE_WRITE); // SPIFFS
  if (!file)
  {
    Serial.Fprintln("Failed to open file in writing mode");
  }
  else
  {
    // 프레임버퍼에서 크기만큼 파일쓰기
    file.write(fb->buf, fb->len); // payload (image), payload length, '->': 포인터를 통해 멤버 접근( a->b == (*a).b ), '.' 클래스의 멤버 직접 접근
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();
  esp_camera_fb_return(fb);

  fileName = path.c_str();
  if (isPaired) // -> manageslave()로 이어짐; slave와 페어링된 상태면 전송
    startTransmit();

  ++pictureNumber;
}

/* ***************************************************************** */
/* INIT SD Card                                                      */
/* ***************************************************************** */
// Disabled SD Card
/*
void initSD()
{
  Serial.Fprintln("Starting SD Card");

#if defined(TTGO_T_Camera_plus)
  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.Fprintln("SD Card Mount Failed");
    return;
  }
#else
  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.Fprintln("SD Card Mount Failed");
    return;
  }
#endif

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.Fprintln("No SD Card attached");
    return;
  }
  if (cardType == CARD_MMC)
  {
    Serial.println("cardType: MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("cardType: SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("cardType: SDHC");
  }
  else
  {
    Serial.println("cardType: UNKNOWN");
  }
}
*/

/* ***************************************************************** */
/* INIT CAMERA                                                       */
/* ***************************************************************** */
void initCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // no psram
  Serial.Fprint("psramFound() = ");
  Serial.println(String(psramFound()));

#if defined(CAMERA_MODEL_AI_THINKER)
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_SVGA; // FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA //FRAMESIZE_QVGA
    config.jpeg_quality = 16;
    config.fb_count = 2;
  }
#elif defined(TTGO_T_Camera_plus)
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_SVGA; // FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA //FRAMESIZE_QVGA
    config.jpeg_quality = 16;
    config.fb_count = 2;
  }
#endif
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 16;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

/* ***************************************************************** */
/*                  ESP NOW RELATED FUNCTIONS                        */
/* ***************************************************************** */

/* ***************************************************************** */
/* START TRASMIT                                                     */
/* ***************************************************************** */
void startTransmit()
{
  // fs.open() 갈수록 지연됨

  tempMillis = millis(); // debugging set0

  Serial.Fprintln("Starting transmit");

  // SD card parts -> SPIFFS 플래시 기록으로 대체
  // fs::FS &fs = SD_MMC;
  // File file = fs.open(fileName.c_str(), FILE_READ); // 파일을 읽어서; SD Card

  File file = SPIFFS.open(fileName.c_str(), FILE_READ); // 파일을 읽어서; SPIFFS
  if (!file)
  {
    Serial.Fprintln("Failed to open file in writing mode");
    return;
  }

  currentMillis = millis();                                                                              // debugging set0
  Serial.printf("picnum: %d / Interval 00000: %lu ms\n", pictureNumber - 1, currentMillis - tempMillis); // debugging set0

  Serial.Fprint("File Size: ");
  Serial.println(file.size());
  int fileSize = file.size(); // 크기를 구하고
  file.close();

  // ESP NOW는 한 번에 250바이트 전송; 때문에 데이터 컷팅 必; package: 전송 단위
  currentTransmitCurrentPosition = 0;
  // 총 전송 단위 수
  currentTransmitTotalPackages = ceil(fileSize / fileDatainMessage); // data cut-down; ceil(): 소숫점 올림함수
  Serial.Fprint("Total Packages: ");
  Serial.println(currentTransmitTotalPackages);
  // package 총량이 255를 넘어갈 수 있으므로 2 segment로 분리해야함.
  // int(32bit) 슬라이스 -> {01, uint8_t(상위 24비트), 하위 8비트(축소변환)}
  // 결과적으로 {01, 하위 16~9비트, 하위 8비트}; (int 하위 16비트만 남음)
  // 대충 240*2^16=15.7MB정도까지 한계 -> 오버플로 가능성 있지만 카메라 한계상 가능성없을듯
  uint8_t message[] = {uint8_t(camId.toInt()), 0x01, currentTransmitTotalPackages >> 8, (byte)currentTransmitTotalPackages};

  // sendData() >> esp_now_send(); 4Bytes 메시지 전송 {camId, slaveCheck, filesize 2nd Byte, fileSize 1st Byte}
  // 몇 패키지를 받을지 미리 고지한다고 보면 될듯
  sendData(message, sizeof(message));

  // 파일 내용 전송은 이후 flow
}

/* ***************************************************************** */
/* SEND NEXT PACKAGE                                                 */
/* ***************************************************************** */
void sendNextPackage()
{
  // clear the flag
  sendNextPackageFlag = 0; // OnDataSent() 콜백에서 1로 변경됨

  // if got to AFTER the last package
  // 마지막 패키지 전송 이후: 현재위치 = Total Packages 수 (전송 절차 이후 count++ 하므로)
  // e.g) TP: 256개면 변수는 0~255; index 256이면 255째(마지막) 전송 후 count++ 상태
  if (currentTransmitCurrentPosition == currentTransmitTotalPackages)
  {
    // reset
    currentTransmitCurrentPosition = 0;
    currentTransmitTotalPackages = 0;
    Serial.Fprintln("Done submitting files"); // 파일 전송이 완료됨 표시

    // currentMillis = millis();
    // Serial.printf("Interval Check4: %lu\n", currentMillis - previousMillis);

    // takeNextPhotoFlag = true; // 타이머로 주기적 촬영 구현해서 필요없어짐
    return;
  } // end if

  // 그렇지 않거나 첫 전송 때: 파일 읽음; first read the data.
  // SD card parts -> SPIFFS 플래시 기록으로 대체
  // fs::FS &fs = SD_MMC;

  // tempMillis = millis(); // debugging set1

  // File file = fs.open(fileName.c_str(), FILE_READ);     // SD Card
  File file = SPIFFS.open(fileName.c_str(), FILE_READ); // SPIFFS

  // currentMillis = millis();                                                                              // debugging set1
  // Serial.printf("picnum: %d / Interval 11111: %lu ms\n", pictureNumber - 1, currentMillis - tempMillis); // debugging set1

  if (!file)
  {
    Serial.Fprintln("Failed to open file in writing mode");
    return;
  }

  // set array size.
  int fileDataSize = fileDatainMessage;

  // if its the last package - we adjust the size !!!
  // 마지막 패키지 때 크기 분석해서 전송 크기 조정
  // e.g) TP: 256개면 변수는 0~255; index 255이면 254째(마지막 직전) 전송 후 count++; 마지막 패키지 전송 직전
  if (currentTransmitCurrentPosition == currentTransmitTotalPackages - 1)
  {
    Serial.Fprintln("*************************");
    Serial.Fprint("File Size: ");
    Serial.println(file.size());
    Serial.Fprint("current Transmit packages: ");
    Serial.println(currentTransmitTotalPackages - 1);
    Serial.Fprint("Transmitted Data:");
    Serial.println((currentTransmitTotalPackages - 1) * fileDatainMessage);                // 256개; 255 * 240, [지금까지 전송 Bytes 수 표시]
    fileDataSize = file.size() - ((currentTransmitTotalPackages - 1) * fileDatainMessage); // 전체 fileSize에서 지금까지 전송한 양 차감 -> 남은 데이터 < fileDatainMessage [자투리 데이터 계산]
    Serial.Fprint("Last package Data: ");
    Serial.println(fileDataSize);
  }

  // define message array
  uint8_t messageArray[fileDataSize + 4]; // 0:Camid; 1:msg; 2,3:data bytes

  messageArray[0] = uint8_t(camId.toInt()); // 0~

  messageArray[1] = 0x02; // 나머지 2개 원소는? -> 아래 [1], [2] 설정

  // seek() 함수로 파일 탐색 포인트 이동: 다음 보낼 데이터 시작점 지정
  file.seek(currentTransmitCurrentPosition * fileDatainMessage);
  ++currentTransmitCurrentPosition; // set to current (after seek!!!)

  // slave가 알도록 현재위치 2Bytes 메시지에 저장
  messageArray[2] = currentTransmitCurrentPosition >> 8;  // 비트연산, 8비트 right shift, 2nd 8bits 남김
  messageArray[3] = (byte)currentTransmitCurrentPosition; // 1st 8bits; 총 하위 16비트
  for (int i = 0; i < fileDataSize; ++i)
  {
    if (file.available())
    {
      messageArray[4 + i] = file.read(); // [3]원소부터 파일내용 기록
    }
    // end if available

    else
    {
      Serial.Fprintln("END !!!");
      break;
    }
  } // end for

  // 데이터 본격 전송
  sendData(messageArray, sizeof(messageArray));
  file.close();
}

/* ***************************************************************** */
/* SEND DATA                                                         */
/* ***************************************************************** */
void sendData(const uint8_t *dataArray, uint8_t dataArrayLength)
{

  const uint8_t *peer_addr = slave.peer_addr;
  // Serial.Fprint("Sending: "); Serial.println(data);
  // Serial.Fprint("length: "); Serial.println(dataArrayLength);

  esp_err_t result = esp_now_send(peer_addr, dataArray, dataArrayLength);

  // Serial.Fprint("Send Status: ");
  if (result == ESP_OK)
  {
    // Serial.Fprintln("Success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.Fprintln("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.Fprintln("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.Fprintln("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.Fprintln("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.Fprintln("Peer not found.");
  }
  else
  {
    Serial.Fprintln("Not sure what happened");
  }
}

/* ***************************************************************** */
/* callback when data is sent from Master to Slave                   */
/* ***************************************************************** */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // print info
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.Fprint("마지막 패킷 목적지: ");
  // Serial.println(macStr);
  // Serial.Fprint("마지막 패킷 전송 상태: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS);

  if (currentTransmitTotalPackages) // 보낼 패키지가 남아있으면
  {
    sendNextPackageFlag = 1; // 보냄 신호 on
    // if nto success 0 resent the last one
    if (status != ESP_NOW_SEND_SUCCESS) // 온전히 전송되지 않았으면
      currentTransmitCurrentPosition--; // count를 되돌려 재전송하도록 함
  }                                     // end if
}

/* ***************************************************************** */
/* Init ESP Now with fallback                                        */
/* ***************************************************************** */
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.Fprintln("ESPNow Init Success");
  }
  else
  {
    Serial.Fprintln("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

/* ***************************************************************** */
/* Scan for slaves in AP mode                                        */
/* ***************************************************************** */
void ScanAndConnectToSlave()
{
  int8_t scanResults = WiFi.scanNetworks(); // scanNetworks(): 한 번에 사용 가능한 wifi 네트워크를 검색하고 발견된 네트워크 수 반환
  // reset on each scan
  bool slaveFound = false;
  memset(&slave, 0, sizeof(slave));

  Serial.Fprintln("");
  if (scanResults == 0)
  {
    Serial.Fprintln("No WiFi devices(Slaves) in AP Mode found");
  }
  else
  {
    Serial.Fprint("Found ");
    Serial.print(scanResults);
    Serial.Fprintln(" devices ");

    // 찾은 장치만큼 반복
    for (int i = 0; i < scanResults; ++i)
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);         // SSID
      int32_t RSSI = WiFi.RSSI(i);        // Received Signal Strength Indication, 수신 신호 강도 수치
      String BSSIDstr = WiFi.BSSIDstr(i); // Basic Service Set Identification == MAC주소 / .BSSID(): 포인터 반환

      if (PRINTSCANRESULTS)
      {
        Serial.print(i + 1);
        Serial.Fprint(": ");
        Serial.print(SSID);
        Serial.Fprint(" (");
        Serial.print(RSSI);
        Serial.Fprint(")");
        Serial.Fprintln("");
      }
      delay(10);

      // Check if the current device starts with `Slave`
      // 현재 장치의 ssid가 Slave로 시작하는지 체크
      if (SSID.indexOf("Slave") == 0) // indexOf(): 찾은 문자열의 시작 인덱스 반환
      {
        // SSID of interest
        Serial.Fprintln("Found a Slave.");
        Serial.print(i + 1);
        Serial.Fprint(": ");
        Serial.print(SSID);
        Serial.Fprint(" [");
        Serial.print(BSSIDstr);
        Serial.Fprint("]");
        Serial.Fprint(" (");
        Serial.print(RSSI);
        Serial.Fprint(")");
        Serial.Fprintln("");
        // Get BSSID => Mac Address of the Slave
        unsigned int mac[6];
        // scanf()와 동일, 입력 대상이 표준 입력이 아닌 매개변수로 전달되는 [문자열 버퍼] / 1st param: 입력한 문자열: string->const char* 변환
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
        {
          for (int ii = 0; ii < 6; ++ii)
          {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0;       // no encryption

        slaveFound = true;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
      /*
      else if (slaveMAC.length() == 17) // 찬솔 추가: index 대신 MAC주소 입력값으로 연결
      {
        // SSID of interest
        // Serial.Fprint("Input Slave MAC Addr: ");
        // Serial.println(slaveMAC);

        int mac[6];
        // scanf()와 동일, 입력 대상이 표준 입력이 아닌 매개변수로 전달되는 [문자열 버퍼] / 1st param: 입력한 문자열: c_str(): string->const char* 변환
        if (6 == sscanf(slaveMAC.c_str(), "%x:%x:%x:%x:%x:%x%c", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
        {
          for (int ii = 0; ii < 6; ++ii)
          {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
          slave.channel = CHANNEL; // pick a channel
          slave.encrypt = 0;       // no encryption

          slaveFound = 1;
          // we are planning to have only one slave in this example;
          // Hence, break after we find one, to be a bit efficient
          break;
        }
        Serial.Fprintln("Invalid MAC Addr.");
      } // else if end
      */

    } // for end
  }

  if (slaveFound)
  {
    Serial.Fprintln("Slave Found, processing...");
    if (slave.channel == CHANNEL)
    { // check if slave channel is defined
      // `slave` is defined
      // Add slave as peer if it has not been added already
      // slave를 peer로 추가 / 이미 peer가 추가되지 않았다면
      isPaired = manageSlave();
      if (isPaired)
      {
        Serial.Fprintln("Slave pair success!");
      }
      else
      {
        // slave pair failed
        Serial.Fprintln("Slave pair failed!");
      }
    }
  }
  else
  {
    Serial.Fprintln("Slave Not Found, trying again.");
  }
  // clean up ram, Delete the last scan result from memory.
  WiFi.scanDelete();
} // end function

/* ***************************************************************** */
/* master와 이미 페어링된 slave인지 검사                               */
/* 아니라면, master와 pairing                                         */
/* ***************************************************************** */
bool manageSlave() // bool이네?
{
  if (slave.channel == CHANNEL)
  {
    if (DELETEBEFOREPAIR) // 페어링 하기 전 삭제 변수 on이면 먼저 피어 삭제
    {
      deletePeer();
    }

    Serial.Fprint("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave; // peer 포인터를 슬레이브 주소 가리키도록
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if (exists) // peer 존재하면
    {
      // Slave already paired.
      Serial.Fprintln("Already Paired");
      return true;
    }
    else // peer 존재하지 않으면 paring 시도
    {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) // peer 추가 시 == paring 성공 시
      {
        // Pair success
        Serial.Fprintln("Pair success");
        return true;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
      {
        // How did we get so far!!: 어쩌다 여기까지 왔니 ㅋㅋ
        Serial.Fprintln("ESPNOW Not Init");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_ARG)
      {
        Serial.Fprintln("Invalid Argument");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_FULL)
      {
        Serial.Fprintln("Peer list FULL");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
      {
        Serial.Fprintln("Out of memory");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_EXIST)
      {
        Serial.Fprintln("Peer Exists");
        return true;
      }
      else
      {
        Serial.Fprintln("Not sure what happened");
        return false;
      }
    }
  }
  else // 채널비교 다르면
  {
    // No slave found to process
    Serial.Fprintln("No Slave found to process");
    return false;
  }
}

/* ***************************************************************** */
/* DELETE PEER                                                       */
/* ***************************************************************** */
void deletePeer()
{
  // const esp_now_peer_info_t *peer = &slave; //never used
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.Fprint("Slave Delete Status: ");
  if (delStatus == ESP_OK)
  {
    // Delete success
    Serial.Fprintln("Success");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.Fprintln("ESPNOW Not Init");
  }
  else if (delStatus == ESP_ERR_ESPNOW_ARG)
  {
    Serial.Fprintln("Invalid Argument");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.Fprintln("Peer not found.");
  }
  else
  {
    Serial.Fprintln("Not sure what happened");
  }
}

/* ***************************************************************** */
/*                  HELPERS RELATED FUNCTIONS                        */
/* ***************************************************************** */
// Onboard LED Bilnk: Never used
/*
void blinkIt(int delayTime, int times)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(ONBOADLED, HIGH);
    delay(delayTime);
    digitalWrite(ONBOADLED, LOW);
    delay(delayTime);
  }
}
*/

// Initialize SPIFFS
void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.Fprintln("An error has occurred while mounting SPIFFS");
  }
  Serial.Fprintln("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory())
  {
    Serial.Fprintln("- failed to open file for reading");
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
    Serial.Fprintln("- failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.Fprintln("- file written");
  }
  else
  {
    Serial.Fprintln("- write failed");
  }
}

bool isCamConfigDefined()
{
  if (camId == "" || slaveMAC == "" || capturePeriod == "")
  {
    Serial.Fprintln("Undefined Cam ID or slaveMac or Capture Period.");
    return false;
  }
  return true;
}

float getCurrFreeHeapRatio()
{
  return (float)ESP.getFreeHeap() / (float)ESP.getHeapSize() * 100;
}