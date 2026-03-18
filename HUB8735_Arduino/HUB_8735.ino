#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include <AmebaServo.h>

// 定義巨集
#define CHANNEL   0
#define CHANNELNN 3
#define NNWIDTH  576
#define NNHEIGHT 320

// 設置視訊參數
VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);

// 創建物件
NNObjectDetection ObjDet;
RTSP rtsp;
AmebaServo myservo1;  // 只有一個馬達
IPAddress ip;

// 處理視訊流的輸入和輸出
StreamIO videoStreamer(1, 1);
// 處理神經網絡模型的輸入和輸出視訊流
StreamIO videoStreamerNN(1, 1);

// 定義網路名稱和密碼
char ssid[] = "67";
char pass[] = "12340000";

// 代表網路的連接狀態
int status = WL_IDLE_STATUS;

// 定義名為rtsp_portnum的整數變數
int rtsp_portnum;

// 定義按鈕腳位和按鈕狀態
const int buttonPin = 2;
int buttonState = LOW;
int lastButtonState = LOW;  // 上一次按鈕狀態
unsigned long buttonPressStartTime = 0;  // 記錄按鈕按下的開始時間
bool isUnlocked = false;  // 標記是否解鎖
unsigned long unlockStartTime = 0;  // 記錄解鎖時間

void setup()
{
    // 定義鮑率
    Serial.begin(115200);

    // 定義伺服馬達的腳位
    myservo1.attach(9);  // 只有一個伺服馬達

    // 設置按鈕腳位為輸入
    pinMode(buttonPin, INPUT);

    // 連結到Wi-Fi網路,直到成功為止
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        // wait 2 seconds for connection:
        delay(2000);
    }
    ip = WiFi.localIP();

    // 配置相機的視頻通道和RTSP相關參數
    config.setBitrate(2 * 1024 * 1024);
    Camera.configVideoChannel(CHANNEL, config);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();
    rtsp.configVideo(config);
    rtsp.begin();
    rtsp_portnum = rtsp.getPort();
    Serial.print("RTSP Stream URL: rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);
    delay(10000);
    // 定義物件檢測的視訊設定(包含解析度、幀率、模型等)
    ObjDet.configVideo(configNN);
    ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
    // 開始執行物件檢測
    ObjDet.begin();

    // 設置視訊串流的輸入和輸出,並開始視訊串流
    videoStreamer.registerInput(Camera.getStream(CHANNEL));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // 啟動CHANNEL通道,獲得該通道中捕捉的視訊串流
    Camera.channelBegin(CHANNEL);

    // 配置並啟動另一個視訊串流,專門用於物件檢測
    videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerNN.setStackSize();
    videoStreamerNN.setTaskPriority();
    videoStreamerNN.registerOutput(ObjDet);
    if (videoStreamerNN.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // 啟動CHANNELNN通道,獲得該通道中捕捉的視訊串流
    Camera.channelBegin(CHANNELNN);

    // 配置和啟動OSD功能
    OSD.configVideo(CHANNEL, config);
    OSD.begin();
}

// 主要的程式循環
void loop()
{
    // ===== 讀按鈕 =====
    buttonState = digitalRead(buttonPin);  // LOW = 按下

    // ===== 影像辨識 =====
    std::vector<ObjectDetectionResult> results = ObjDet.getResult();

    uint16_t im_h = config.height();
    uint16_t im_w = config.width();

    OSD.createBitmap(CHANNEL);

    bool detectedMaster = false;

    if (ObjDet.getResultCount() > 0) {
        for (int i = 0; i < ObjDet.getResultCount(); i++) {

            int obj_type = results[i].type();

            if (itemList[obj_type].filter) {

                ObjectDetectionResult item = results[i];

                int xmin = (int)(item.xMin() * im_w);
                int xmax = (int)(item.xMax() * im_w);
                int ymin = (int)(item.yMin() * im_h);
                int ymax = (int)(item.yMax() * im_h);

                OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

                char text_str[20];
                snprintf(text_str, sizeof(text_str), "%s %d",
                         itemList[obj_type].objectName, item.score());

                OSD.drawText(CHANNEL, xmin,
                             ymin - OSD.getTextHeight(CHANNEL),
                             text_str, OSD_COLOR_CYAN);

                // ===== 判斷 master =====
                if (strcmp(itemList[obj_type].objectName, "master") == 0) {
                    detectedMaster = true;
                }
            }
        }
    }

    // ===== 解鎖邏輯（只看影像）=====
    if (detectedMaster && !isUnlocked) {
        myservo1.write(0);   // 解鎖
        isUnlocked = true;

        Serial.println("Unlocked (Master detected)");
    }

    // ===== 按鈕「長按」偵測 =====

    // 按下瞬間
    if (buttonState == LOW && lastButtonState == HIGH) {
        buttonPressStartTime = millis();
    }

    // 持續按
    if (buttonState == LOW) {
        if (millis() - buttonPressStartTime > 500) {

            // 👉 只要長按，就上鎖（但避免重複觸發）
            if (isUnlocked) {
                myservo1.write(90);  // 上鎖
                isUnlocked = false;

                Serial.println("Locked (Button long press)");

                // 👉 防止一直觸發（關鍵）
                buttonPressStartTime = millis() + 100000;
            }
        }
    }

    // 更新狀態
    lastButtonState = buttonState;

    OSD.update(CHANNEL);

    delay(50);
}