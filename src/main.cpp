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
#include "FS.h"               // SD Card ESP32
#include "SD_MMC.h"           // SD Card ESP32
#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
#include "driver/rtc_io.h"

#include <esp_now.h>
#include <WiFi.h>
#define ONBOADLED 4
// #define RXPIN 3
#include "esp_camera.h"

#include "SPIFFS.h"
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

unsigned long previousMillis = 0; // Stores last time using Reference time

// #define CONFIG_ARDUINO_LOOP_STACK_SIZE 16 * 1024 // 16KB
// uxTaskGetStackHighWaterMark

// Pin definition for CAMERA_MODEL_AI_THINKER, esp32 cam 핀 배열
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

#define fileDatainMessage 240.0 // ESPNOW 최대 250Bytes전송 고려 data cut-down
#define UARTWAITHANDSHACK 1000

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 1

// for esp now connect
unsigned long lastConnectNowAttempt = 0;
unsigned long nextConnectNowGap = 1000;
bool isPaired = 0;

// for photo name
int pictureNumber = 1;
const uint32_t n_zero = 7; // zero padding
byte takeNextPhotoFlag = 0;

// for photo transmit
int currentTransmitCurrentPosition = 0;
int currentTransmitTotalPackages = 0;
byte sendNextPackageFlag = 0;
String fileName = "/moon.jpg";

// for connection type
// bool useUartRX = 0;

// 메소드 선언부
void takePhoto();
void initSD();
void initCamera();
void startTransmit();
void sendNextPackage();
void sendData(uint8_t *dataArray, uint8_t dataArrayLength);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void InitESPNow();
void ScanAndConnectToSlave();
bool manageSlave();
void deletePeer();
void blinkIt(int delayTime, int times);

void initSPIFFS();                                                 // Initialize SPIFFS
String readFile(fs::FS &fs, const char *path);                     // Read File from SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message); // Write file to SPIFFS
bool initCamConfig();                                              // For Reset Cam Configuration
bool allowsLoop = false;                                           // loop() DO or NOT

float getCurrFreeHeapRatio();

void setup()
{
  // NEEDED ????
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
  // start serial

  Serial.begin(115200);

  initSPIFFS(); // init SPIFFS

  // Load values saved in SPIFFS
  camId = readFile(SPIFFS, camIdPath);
  slaveMAC = readFile(SPIFFS, slaveMACPath);
  capturePeriod = readFile(SPIFFS, capturePeriodPath);
  Serial.print("CamID in SPIFFS: ");
  Serial.println(camId);
  Serial.print("SlaveMAC in SPIFFS: ");
  Serial.println(slaveMAC);
  Serial.print("CapturePeriod in SPIFFS: ");
  Serial.println(capturePeriod);

  // AP모드 진입(cam config reset): softAP() 메소드
  if (initCamConfig())
  {
    // Connect to Wi-Fi network with SSID and password
    Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP(); // Software enabled Access Point : 가상 라우터, 가상의 액세스 포인트
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Print Chip Info.
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.println();
    Serial.printf("Flash Chip Size = %d byte\n", ESP.getFlashChipSize());
    Serial.printf("Flash Frequency = %d Hz\n", ESP.getFlashChipSpeed());
    Serial.println();
    Serial.printf("Total Heap Size = %d\n", ESP.getHeapSize());
    Serial.println();

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
            Serial.print("Cam ID set to: ");
            Serial.println(camId);
            // Write file to save value
            writeFile(SPIFFS, camIdPath, camId.c_str());
          }
          // HTTP POST slaveMAC value
          if (p->name() == PARAM_INPUT_2) {
            slaveMAC = p->value().c_str();
            Serial.print("Dest. MAC set to: ");
            Serial.println(slaveMAC);
            // Write file to save value
            writeFile(SPIFFS, slaveMACPath, slaveMAC.c_str());
          }
          // HTTP POST capturePeriod value
          if (p->name() == PARAM_INPUT_3) {
            capturePeriod = p->value().c_str();
            Serial.print("Capture Period set to: ");
            Serial.println(capturePeriod);
            // Write file to save value
            writeFile(SPIFFS, capturePeriodPath, capturePeriod.c_str());
          }
          
          Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      // ESP가 양식 세부 정보를 수신했음을 알 수 있도록 일부 텍스트가 포함된 응답을 send
      request->send(200, "text/plain", "Done. ESP will restart, Take photo periodcally, and then Send it to Slave Device: " + slaveMAC);
      delay(3000);
      ESP.restart(); });
    server.begin();
  }
  else
  {
    Serial.println("CAMERA MASTER STARTED"); // tarted : 시작되다; 자동사인듯?
    initCamera();                            // init camera
    initSD();                                // init sd

    // init onboad led
    pinMode(ONBOADLED, OUTPUT);
    digitalWrite(ONBOADLED, LOW);

    // we now test to see if we got serial communication
    unsigned long testForUart = millis();
    Serial.print("WAIT UART");
    while (testForUart + UARTWAITHANDSHACK > millis() && !Serial.available())
    {
      Serial.print(".");
      delay(50);
    }

    if (Serial.available())
    {
      Serial.println("We are using Serial!!");
      while (Serial.available())
      {
        Serial.println(Serial.read());
      }
      // useUartRX = 1;
    }
    /*
        if (1) //! useUartRX
        {*/
    // set RX as pullup for safety
    // pinMode(RXPIN, INPUT_PULLUP);
    // Serial.println("We are using the button");

    // Set device in STA mode to begin with
    WiFi.mode(WIFI_STA);
    // This is the mac address of the Master in Station Mode
    Serial.print("STA MAC: ");
    Serial.println(WiFi.macAddress());

    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    /*}*/
    allowsLoop = true;
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
    Serial.println("NOT CONNECTED -> TRY TO CONNECT");
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

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= (long)capturePeriod.toInt() && !currentTransmitTotalPackages && !sendNextPackageFlag && allowsLoop) //! useUartRX && !digitalRead(RXPIN) &&
  {
    previousMillis = currentMillis;
    takeNextPhotoFlag = 1;
    Serial.print("Input Slave MAC LENGTH:");
    Serial.println(slaveMAC.length());
  }
  // if the sendNextPackageFlag is set:
  if (sendNextPackageFlag)
  {
    sendNextPackage(); // 다음 패키지를 보냄:
  }

  // if takeNextPhotoFlag is set: 사진 촬영 변수
  if (takeNextPhotoFlag && allowsLoop)
  {
    takePhoto();                                                            // >> startTransmit()로 이어짐 (currentTransmitTotalPackages 변수 설정)
                                                                            // >> startTransmit() >> sendData() >> esp_now_send();
    Serial.printf("Curr freeHeap Size3: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING
  }

  /*
    // we only read serial if we use the uart: 직렬통신으로 serial에 명령 입력 가능
    if (Serial.available()) //  && useUartRX
    {
      switch (Serial.read())
      {
      case 'p':
      case 'P':
        takeNextPhotoFlag = 1;
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
        Serial.println("not supported!!!");
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
  takeNextPhotoFlag = 0;

  // digitalWrite(4, HIGH);
  // delay(50);
  camera_fb_t *fb = NULL; // 카메라 버퍼 구조체

  Serial.printf("Curr freeHeap Size1: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

  // Take Picture with Camera
  fb = esp_camera_fb_get(); // 포인터에서 프레임 버퍼를 얻음

  Serial.printf("Curr freeHeap Size2: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

  if (!fb) // 버퍼가 비어있으면 카매라 캡쳐 실패
  {
    Serial.println("Camera capture failed");
    return;
  }
  // digitalWrite(4, LOW);

  // 찬솔-기능추가: 파일명 zero padding
  std::string old_str = std::to_string(pictureNumber);
  std::string new_str = std::string(n_zero - std::min(n_zero, old_str.length()), '0') + old_str;

  // Path where new picture will be saved in SD Card
  std::string path = "/picture" + new_str + ".jpg";

  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  fs.remove(path.c_str()); // 이미 있으면 지우고

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file in writing mode");
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

  pictureNumber++;
}

/* ***************************************************************** */
/* INIT SD card                                                      */
/* ***************************************************************** */
void initSD()
{
  Serial.println("Starting SD Card");
  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached");
    return;
  }
}

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

  Serial.println("psramFound() = " + String(psramFound()));

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_QVGA; // FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA //FRAMESIZE_QVGA
    config.jpeg_quality = 2;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
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
  Serial.println("Starting transmit");
  fs::FS &fs = SD_MMC;
  File file = fs.open(fileName.c_str(), FILE_READ); // 파일을 읽어서
  if (!file)
  {
    Serial.println("Failed to open file in writing mode");
    return;
  }
  Serial.print("File Size: ");
  Serial.println(file.size());
  int fileSize = file.size(); // 크기를 구하고
  file.close();

  // ESP NOW는 한 번에 250바이트 전송; 때문에 데이터 컷팅 必; package: 전송 단위
  currentTransmitCurrentPosition = 0;
  // 총 전송 단위 수
  currentTransmitTotalPackages = ceil(fileSize / fileDatainMessage); // data cut-down; ceil(): 소숫점 올림함수
  Serial.print("Total Packages: ");
  Serial.println(currentTransmitTotalPackages);
  // package 총량이 255를 넘어갈 수 있으므로 2 segment로 분리해야함.
  // int(32bit) 슬라이스 -> {01, uint8_t(상위 24비트), 하위 8비트(축소변환)}
  // 결과적으로 {01, 하위 16~9비트, 하위 8비트}; (int 하위 16비트만 남음)
  // 대충 240*2^16=15.7MB정도까지 한계 -> 오버플로 가능성 있지만 카메라 한계상 가능성없을듯
  uint8_t message[] = {0x01, currentTransmitTotalPackages >> 8, (byte)currentTransmitTotalPackages};

  // sendData() >> esp_now_send(); 3Bytes 메시지 전송 {slaveCheck, filesize 2nd Byte, fileSize 1st Byte}
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
    Serial.println("Done submitting files"); // 파일 전송이 완료됨 표시

    Serial.printf("Curr freeHeap Size4: %.1f%%\n", getCurrFreeHeapRatio()); // DEBUGGING

    // takeNextPhotoFlag = 1; // 타이머로 주기적 촬영 구현해서 필요없어짐
    return;
  } // end if

  // 그렇지 않거나 첫 전송 때: 파일 읽음; first read the data.
  fs::FS &fs = SD_MMC;
  File file = fs.open(fileName.c_str(), FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file in writing mode");
    return;
  }

  // set array size.
  int fileDataSize = fileDatainMessage;

  // if its the last package - we adjust the size !!!
  // 마지막 패키지 때 크기 분석해서 전송 크기 조정
  // e.g) TP: 256개면 변수는 0~255; index 255이면 254째(마지막 직전) 전송 후 count++; 마지막 패키지 전송 직전
  if (currentTransmitCurrentPosition == currentTransmitTotalPackages - 1)
  {
    Serial.println("*************************");
    Serial.print("File Size: ");
    Serial.println(file.size());
    Serial.print("current Transmit packages: ");
    Serial.println(currentTransmitTotalPackages - 1);
    Serial.print("Transmitted Data:");
    Serial.println((currentTransmitTotalPackages - 1) * fileDatainMessage);                // 256개; 255 * 240, [지금까지 전송 Bytes 수 표시]
    fileDataSize = file.size() - ((currentTransmitTotalPackages - 1) * fileDatainMessage); // 전체 fileSize에서 지금까지 전송한 양 차감 -> 남은 데이터 < fileDatainMessage [자투리 데이터 계산]
    Serial.print("Last package Data: ");
    Serial.println(fileDataSize);
  }

  // Serial.println("fileDataSize=" + String(fileDataSize));  //613라인과 같이 주석설정/해제

  // define message array
  uint8_t messageArray[fileDataSize + 3];
  messageArray[0] = 0x02; // 나머지 2개 원소는? -> 아래 [1], [2] 설정

  // seek() 함수로 파일 탐색 포인트 이동: 다음 보낼 데이터 시작점 지정
  file.seek(currentTransmitCurrentPosition * fileDatainMessage);
  ++currentTransmitCurrentPosition; // set to current (after seek!!!)
  // Serial.println("PACKAGE - " + String(currentTransmitCurrentPosition));

  // slave가 알도록 현재위치 2Bytes 메시지에 저장
  messageArray[1] = currentTransmitCurrentPosition >> 8;  // 비트연산, 8비트 right shift, 2nd 8bits 남김
  messageArray[2] = (byte)currentTransmitCurrentPosition; // 1st 8bits; 총 하위 16비트
  for (int i = 0; i < fileDataSize; i++)
  {
    if (file.available())
    {
      messageArray[3 + i] = file.read(); // [3]원소부터 파일내용 기록
    }
    // end if available

    else
    {
      Serial.println("END !!!");
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
void sendData(uint8_t *dataArray, uint8_t dataArrayLength)
{
  const uint8_t *peer_addr = slave.peer_addr;
  // Serial.print("Sending: "); Serial.println(data);
  // Serial.print("length: "); Serial.println(dataArrayLength);

  esp_err_t result = esp_now_send(peer_addr, dataArray, dataArrayLength);

  // Serial.print("Send Status: ");
  if (result == ESP_OK)
  {
    // Serial.println("Success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
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
  // Serial.print("마지막 패킷 목적지: ");
  // Serial.println(macStr);
  // Serial.print("마지막 패킷 전송 상태: ");
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
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
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
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0)
  {
    Serial.println("No WiFi devices(Slaves) in AP Mode found");
  }
  else
  {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");

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
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);

      // Check if the current device starts with `Slave`
      // 현재 장치의 ssid가 Slave로 시작하는지 체크
      if (SSID.indexOf("Slave") == 0) // indexOf(): 찾은 문자열의 시작 인덱스 반환
      {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
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

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
      /*
      else if (slaveMAC.length() == 17) // 찬솔 추가: index 대신 MAC주소 입력값으로 연결
      {
        // SSID of interest
        // Serial.print("Input Slave MAC Addr: ");
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
        Serial.println("Invalid MAC Addr.");
      } // else if end
      */

    } // for end
  }

  if (slaveFound)
  {
    Serial.println("Slave Found, processing...");
    if (slave.channel == CHANNEL)
    { // check if slave channel is defined
      // `slave` is defined
      // Add slave as peer if it has not been added already
      // slave를 peer로 추가 / 이미 peer가 추가되지 않았다면
      isPaired = manageSlave();
      if (isPaired)
      {
        Serial.println("Slave pair success!");
      }
      else
      {
        // slave pair failed
        Serial.println("Slave pair failed!");
      }
    }
  }
  else
  {
    Serial.println("Slave Not Found, trying again.");
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

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave; // peer 포인터를 슬레이브 주소 가리키도록
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if (exists) // peer 존재하면
    {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    }
    else // peer 존재하지 않으면 paring 시도
    {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) // peer 추가 시 == paring 성공 시
      {
        // Pair success
        Serial.println("Pair success");
        return true;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
      {
        // How did we get so far!!: 어쩌다 여기까지 왔니 ㅋㅋ
        Serial.println("ESPNOW Not Init");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_ARG)
      {
        Serial.println("Invalid Argument");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_FULL)
      {
        Serial.println("Peer list FULL");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
      {
        Serial.println("Out of memory");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_EXIST)
      {
        Serial.println("Peer Exists");
        return true;
      }
      else
      {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  }
  else // 채널비교 다르면
  {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

/* ***************************************************************** */
/* DELETE PEER                                                       */
/* ***************************************************************** */
void deletePeer()
{
  const esp_now_peer_info_t *peer = &slave;
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK)
  {
    // Delete success
    Serial.println("Success");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  }
  else if (delStatus == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
}

/* ***************************************************************** */
/*                  HELPERS RELATED FUNCTIONS                        */
/* ***************************************************************** */
// Onboard LED Bilnk
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

bool initCamConfig()
{
  if (camId == "" || slaveMAC == "" || capturePeriod == "")
  {
    Serial.println("Undefined Cam ID or slaveMac or Capture Period.");
    return true;
  }
  return false;
}

float getCurrFreeHeapRatio()
{
  return (float)ESP.getFreeHeap() / (float)ESP.getHeapSize();
}