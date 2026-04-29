#pragma once
#include <WiFi.h>
#include <esp_now.h>

typedef struct
{
  char cmd[20];
} ESPNOW_CMD;

ESPNOW_CMD msg;

// ===================== 【必须写的回调函数】====================
// 发送结果回调
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 空函数也必须写，否则内核崩溃
}

// 接收回调：新版 IDF 5.1 格式（修复报错关键！）
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *inData, int data_len) {
  // 空函数即可，必须存在
}

void espnowInit()
{
    WiFi.mode(WIFI_STA);
    printf("[ESP-NOW] 初始化中...\r\n");
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    printf("[ESP-NOW] 初始OK...\r\n");
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
#ifdef IS_LEFT_SHOE

    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, rightShoeMAC, 6);
    printf("[ESP-NOW] 初始..\r\n");
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);
    printf("[ESP-NOW] 初始done\r\n");
#endif

}