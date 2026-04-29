#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <SCServo.h>
#include "Gyro_QMI8658.h"

// ------------------- 全局开关 -------------------
#define IS_LEFT_SHOE

// ------------------- 差异化配置 -------------------
#ifdef IS_LEFT_SHOE
  #define BLE_DEVICE_NAME     "A_Shoe_LEFT"
  #define POS_BACK_BRAKE      3200
  #define POS_BACK_RELEASE    3800
  #define POS_FRONT_BRAKE     2800
  #define POS_FRONT_RELEASE   1860
  // 左鞋自己的 Server UUID
  #define MY_SERVICE_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
  #define MY_CHAR_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
  // 右鞋的 Server UUID（左鞋 Client 用这个去找右鞋）
  #define RIGHT_SERVICE_UUID  "6E400011-B5A3-F393-E0A9-E50E24DCCA9E"
  #define RIGHT_CHAR_UUID     "6E400013-B5A3-F393-E0A9-E50E24DCCA9E"
#else
  #define BLE_DEVICE_NAME     "A_Shoe_RIGHT"
  #define POS_BACK_BRAKE      2760
  #define POS_BACK_RELEASE    3300
  #define POS_FRONT_BRAKE     2950
  #define POS_FRONT_RELEASE   1920
  // 右鞋用不同的 UUID，避免左鞋 Client 误匹配自身s
  #define MY_SERVICE_UUID     "6E400011-B5A3-F393-E0A9-E50E24DCCA9E"
  #define MY_CHAR_UUID        "6E400013-B5A3-F393-E0A9-E50E24DCCA9E"
#endif

#define RIGHT_SHOE_NAME     "A_Shoe_RIGHT"
#define SERVO_BACK_ID       1
#define SERVO_FRONT_ID      2
#define SERVO_SPEED         100
#define SERVO_ACC           100
#define SERVO_TORQUE        300

// ------------------- 指令队列（解决多指令并发丢失） -------------------
#define CMD_QUEUE_SIZE 20
String cmdQueue[CMD_QUEUE_SIZE];
volatile int cmdQueueHead = 0;
volatile int cmdQueueTail = 0;

// ==================== IMU 校准与滤波 ====================
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;  // 零偏

const float FILTER_ALPHA = 0.08;  // 滤波系数，越小越稳

// 陀螺仪静止校准（开机时调用，必须保持不动）
void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  // 连续读200次求平均
  for (int i = 0; i < 200; i++) {
    getAccelerometer();
    getGyroscope();
    sumX += Gyro.x;
    sumY += Gyro.y;
    sumZ += Gyro.z;
    delay(5);
  }
  gyroOffsetX = sumX / 200;
  gyroOffsetY = sumY / 200;
  gyroOffsetZ = sumZ / 200;

  Serial.print("✅ 陀螺仪校准完成：");
  Serial.print(gyroOffsetY);  // 你会看到这里输出 ~5.1 左右
  Serial.println(" °/s");
}

// 入队
bool enqueueCmd(const String& cmd) {
  int next = (cmdQueueHead + 1) % CMD_QUEUE_SIZE;
  if (next == cmdQueueTail) return false; // 队列满
  cmdQueue[cmdQueueHead] = cmd;
  cmdQueueHead = next;
  return true;
}

// 出队
bool dequeueCmd(String& cmd) {
  if (cmdQueueHead == cmdQueueTail) return false;
  cmd = cmdQueue[cmdQueueTail];
  cmdQueueTail = (cmdQueueTail + 1) % CMD_QUEUE_SIZE;
  return true;
}



bool Flag_startDataTrans = false;

// ======================= 舵机状态 ============================
struct ServoState { bool busy; unsigned long finishTime; bool keepTorque; };
ServoState servoFront = {false, 0, false};
ServoState servoBack  = {false, 0, false};
HLSCL hlscl;

unsigned long lastSendTime = 0;
const long IMU_SEND_INTERVAL = 10;

// ======================= 右鞋数据缓存 ========================
#ifdef IS_LEFT_SHOE
struct RightShoeData { float ax, ay, az, gx, gy, gz; };
RightShoeData rightData = {0, 0, 0, 0, 0, 0};
portMUX_TYPE rightDataMux = portMUX_INITIALIZER_UNLOCKED;
String rightFrameBuffer = "";

void parseRightFrame(const String& frame) {
    int start = frame.indexOf('$');
    int end   = frame.indexOf(';');
    if (start < 0 || end < 0 || end <= start) return;

    String body = frame.substring(start + 1, end);
    float vals[6] = {0};
    int idx = 0, from = 0;
    for (int i = 0; i <= (int)body.length() && idx < 6; i++) {
        if (i == (int)body.length() || body[i] == ',') {
            vals[idx++] = body.substring(from, i).toFloat();
            from = i + 1;
        }
    }
    if (idx < 6) return;

    portENTER_CRITICAL(&rightDataMux);
    rightData = {vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]};
    portEXIT_CRITICAL(&rightDataMux);
}
#endif

// ======================= 舵机控制 ============================
void startServoMove(uint8_t id, uint16_t targetPos, bool keepTorque) {
    int current = hlscl.ReadPos(id);
    hlscl.EnableTorque(id, 1);
    hlscl.WritePosEx(id, targetPos, SERVO_SPEED, SERVO_ACC, SERVO_TORQUE);
    unsigned long moveTime = abs(targetPos - current) * 2;
    if (moveTime < 50) moveTime = 50;
    if (id == SERVO_FRONT_ID) servoFront = {true, millis() + moveTime, keepTorque};
    else                       servoBack  = {true, millis() + moveTime, keepTorque};
}

void updateServoState() {
    unsigned long now = millis();
    if (servoFront.busy && now >= servoFront.finishTime) {
        servoFront.busy = false;
        if (!servoFront.keepTorque) hlscl.EnableTorque(SERVO_FRONT_ID, 0);
    }
    if (servoBack.busy && now >= servoBack.finishTime) {
        servoBack.busy = false;
        if (!servoBack.keepTorque) hlscl.EnableTorque(SERVO_BACK_ID, 0);
    }
}

// ======================= 前向声明 ============================
#ifdef IS_LEFT_SHOE
void sendCmdToRight(const String& cmd);
#endif

// ======================= BLE 服务器 ==========================
BLEServer*         pServer = nullptr;
BLECharacteristic* pChar   = nullptr;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override    { deviceConnected = true; }
    void onDisconnect(BLEServer* s) override {
        deviceConnected = false;
        s->startAdvertising();
    }
};

void brake_front();   void brake_back();   void brake_all();
void release_front(); void release_back(); void release_all();

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        String cmd = c->getValue().c_str();
        cmd.trim();
        //Serial.print("✅ 收到指令："); Serial.println(cmd);
        // 直接入队，立刻返回
        enqueueCmd(cmd);


    }
};

// ======================= 左鞋客户端 ==========================
#ifdef IS_LEFT_SHOE
BLEClient*               pClient        = nullptr;
BLERemoteService*        pRemoteService = nullptr;
BLERemoteCharacteristic* pRemoteChar    = nullptr;
volatile bool rightShoeConnected = false;
volatile bool rightShoeFound     = false;
BLEAdvertisedDevice rightDevice;

void notifyCallback(BLERemoteCharacteristic* pRC,
                    uint8_t* pData, size_t length, bool isNotify) {
    // 打印收到的原始数据
    //Serial.printf("📥 收到右鞋数据 长度=%d: ", length);
    for (size_t i = 0; i < length; i++) Serial.print((char)pData[i]);
    Serial.println();

    for (size_t i = 0; i < length; i++) rightFrameBuffer += (char)pData[i];

    int start = rightFrameBuffer.indexOf('$');
    int end   = rightFrameBuffer.indexOf(';');

    Serial.printf("    buffer=%s start=%d end=%d\n",
        rightFrameBuffer.c_str(), start, end);

    while (start >= 0 && end > start && end - start < 120) {
        String frame = rightFrameBuffer.substring(start, end + 1);
        //Serial.print("    ✅ 解析帧: "); Serial.println(frame);
        parseRightFrame(frame);
        rightFrameBuffer = rightFrameBuffer.substring(end + 1);
        start = rightFrameBuffer.indexOf('$');
        end   = rightFrameBuffer.indexOf(';');
    }
    if (rightFrameBuffer.length() > 200) {rightFrameBuffer.remove(0, rightFrameBuffer.length() - 100);}
}

class RightClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient* c) override    { rightShoeConnected = true; }
    void onDisconnect(BLEClient* c) override {
        rightShoeConnected = false;
        pRemoteService = nullptr;
        pRemoteChar    = nullptr;
    }
};

class ScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        if (dev.getName() == RIGHT_SHOE_NAME) {
            rightDevice    = dev;
            rightShoeFound = true;
            BLEDevice::getScan()->stop();
        }
    }
};

void connectRightShoe() {
    if (!rightShoeFound || rightShoeConnected) return;
    rightShoeFound = false;

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new RightClientCB());

    if (!pClient->connect(&rightDevice)) {
        Serial.println("❌ 右鞋连接失败"); return;
    }
    pClient->setMTU(128);   // ⭐ 加这一句
    // 用右鞋专属的 SERVICE UUID 查找，不会误匹配左鞋自身
    pRemoteService = pClient->getService(RIGHT_SERVICE_UUID);
    if (!pRemoteService) {
        Serial.println("❌ 右鞋服务未找到");
        pClient->disconnect(); return;
    }
    pRemoteChar = pRemoteService->getCharacteristic(RIGHT_CHAR_UUID);
    if (!pRemoteChar) {
        Serial.println("❌ 右鞋特征值未找到");
        pClient->disconnect(); return;
    }
    if (pRemoteChar->canNotify()) {
        pRemoteChar->registerForNotify(notifyCallback);
        Serial.println("✅ 已订阅右鞋 notify");
    }
    if (pRemoteChar->canNotify()) {
        pRemoteChar->registerForNotify(notifyCallback);  // 去掉 bool ok =
        Serial.println("✅ 注册 notify 成功");
    } else {
        Serial.println("❌ 右鞋特征值不支持 notify");
    }
    // 连上立即让右鞋开始持续发送 IMU 数据
    sendCmdToRight("start_data");
    Serial.println("✅ 右鞋连接成功，已开始数据流");
}

void sendCmdToRight(const String& cmd) {
    if (rightShoeConnected && pRemoteChar && pRemoteChar->canWrite()) {
        pRemoteChar->writeValue((uint8_t*)cmd.c_str(), cmd.length(), false);
    }
}

void rightShoeTask(void* param) {
    for (;;) {
        if (!rightShoeConnected) {
            rightShoeFound = false;
            BLEScan* pScan = BLEDevice::getScan();
            pScan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
            pScan->setActiveScan(true);
            pScan->start(1, false);
            pScan->clearResults();
            if (rightShoeFound) connectRightShoe();
        }
        vTaskDelay(pdMS_TO_TICKS(rightShoeConnected ? 2000 : 1000));
    }
}
#endif

// ======================= 动作函数 ============================
void brake_front()   { startServoMove(SERVO_FRONT_ID, POS_FRONT_BRAKE,   true);  }
void brake_back()    { startServoMove(SERVO_BACK_ID,  POS_BACK_BRAKE,    true);  }
void brake_all()     { brake_front(); brake_back(); }
void release_front() { startServoMove(SERVO_FRONT_ID, POS_FRONT_RELEASE, false); }
void release_back()  { startServoMove(SERVO_BACK_ID,  POS_BACK_RELEASE,  false); }
void release_all()   { release_front(); release_back(); }

// ======================= BLE 发送 IMU 数据 ====================
void sendIMUviaBLE() {
    // 诊断日志（确认后可删除）
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 1000) {
        lastLog = millis();
        //Serial.printf("📊 connCount=%d flag=%d accel=(%.2f,%.2f,%.2f)\n",pServer->getConnectedCount(),Flag_startDataTrans,Accel.x, Accel.y, Accel.z);
    }

    if (pServer->getConnectedCount() == 0) return;
    if (millis() - lastSendTime < IMU_SEND_INTERVAL) return;
    lastSendTime = millis();
    static bool printed = false;
    if (!printed) {
        //Serial.printf("右鞋 MTU: %d\n", BLEDevice::getMTU());
        printed = true;
    }

    String data = "$";
    data += String(Accel.x, 2) + "," + String(Accel.y, 2) + "," + String(Accel.z, 2) + ",";
    data += String(Gyro.x,  2) + "," + String(Gyro.y,  2) + "," + String(Gyro.z,  2);

#ifdef IS_LEFT_SHOE
    float rax, ray, raz, rgx, rgy, rgz;
    if (rightShoeConnected) {
        portENTER_CRITICAL(&rightDataMux);
        rax = rightData.ax; ray = rightData.ay; raz = rightData.az;
        rgx = rightData.gx; rgy = rightData.gy; rgz = rightData.gz;
        portEXIT_CRITICAL(&rightDataMux);
    } else {
        rax = ray = raz = rgx = rgy = rgz = 0.0f;
    }
    data += "," + String(rax, 2) + "," + String(ray, 2) + "," + String(raz, 2) + ",";
    data += String(rgx, 2) + "," + String(rgy, 2) + "," + String(rgz, 2);
#endif

    data += ";\r\n";

    // 打印发送内容（确认后可删除）
    //Serial.print("📤 "); Serial.print(data);

    int len = data.length();
    pChar->setValue(data.c_str());
    pChar->notify();
}
// ======================= 初始化 ==============================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial1.begin(1000000, SERIAL_8N1, 11, 10);
    hlscl.pSerial = &Serial1;
    delay(100);

    I2C_Init();
    QMI8658_Init();
    calibrateGyro(); 

    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(128);  // 协商大 MTU，减少分包

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(MY_SERVICE_UUID);
    pChar = pService->createCharacteristic(
        MY_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE  |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    pChar->addDescriptor(new BLE2902());
    pChar->setCallbacks(new MyCallbacks());

    pService->start();
    pServer->getAdvertising()->start();

#ifdef IS_LEFT_SHOE
    xTaskCreate(rightShoeTask, "RightShoeTask", 8192, NULL, 1, NULL);
#endif

    hlscl.EnableTorque(SERVO_FRONT_ID, 0);
    hlscl.EnableTorque(SERVO_BACK_ID, 0);
    Serial.println("✅ 系统启动完成");
    Serial.printf("实际 MTU: %d\n", BLEDevice::getMTU());
}

// ======================= 主循环 ==============================
void loop() {
    updateServoState();
    getAccelerometer();
    getGyroscope();
    Gyro.x -= gyroOffsetX;
    Gyro.y -= gyroOffsetY;
    Gyro.z -= gyroOffsetZ;
    
    //Serial.printf("IMU Gyro : %.2f, %.2f, %.2f\r\n",Gyro.x, Gyro.y, Gyro.z);
    Serial.printf("IMU Accel : %.2f, %.2f, %.2f\r\n",Accel.x, Accel.y, Accel.z);
    // 从指令队列取出并执行（一次执行一条，不阻塞）
    String cmd;
    if (dequeueCmd(cmd)) {
        if (cmd == "brake")      brake_all();
        else if (cmd == "release")    release_all();
        else if (cmd == "brake_1")    brake_front();
        else if (cmd == "release_1")  release_front();
        else if (cmd == "brake_2")    brake_back();
        else if (cmd == "release_2")  release_back();
        else if (cmd == "start_data") Flag_startDataTrans = true;
        else if (cmd == "stop_data")  Flag_startDataTrans = false;

    #ifdef IS_LEFT_SHOE
        if (cmd == "brake"   || cmd == "release"   ||
            cmd == "brake_1" || cmd == "release_1" ||
            cmd == "brake_2" || cmd == "release_2") {
            sendCmdToRight(cmd); 
        }
    #endif
    }
    if (Flag_startDataTrans) sendIMUviaBLE();
    delay(10);
}