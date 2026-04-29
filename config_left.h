#pragma once

// ================= 左右鞋切换 =================
#define IS_LEFT_SHOE

// ================= BLE =================
#define BLE_DEVICE_NAME "A_Shoe_LEFT"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Nordic UART Service UUID
//#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
//#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
//#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ================= 舵机 =================
#define SERVO_BACK_ID 1
#define SERVO_FRONT_ID 2

#define SERVO_SPEED 100
#define SERVO_ACC 100
#define SERVO_TORQUE 300

// ================= 左鞋位置 =================
#define POS_BACK_BRAKE 3200
#define POS_BACK_RELEASE 3800

#define POS_FRONT_BRAKE 2570
#define POS_FRONT_RELEASE 1860

// ================= 右鞋MAC =================
uint8_t rightShoeMAC[] = {0xE8,0xF6,0x0A,0x92,0x50,0x60};