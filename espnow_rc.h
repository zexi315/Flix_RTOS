#ifndef ESPNOW_DATA_H
#define ESPNOW_DATA_H

#include <WiFi.h>
#include <ESP32_NOW.h>
#include <esp_mac.h>
#include <vector>
#include "util.h"

#define ESPNOW_WIFI_CHANNEL 6

// 与遥控器端保持一致的数据结构
typedef struct Data_Package {
  byte j1PotX;  // 左杆左右（油门）
  byte j1PotY;  // 左杆前后（偏航）
  byte j2PotX;  // 右杆左右（横滚）
  byte j2PotY;  // 右杆前后（俯仰）
  bool buttonR1;
  bool buttonR2;
  bool j1Button;
  bool j2Button;
  bool buttonLB;
  bool buttonRB;
  bool tSwitch1;
} Data_Package;

Data_Package rxData;

// 外部变量（飞控核心控制量）
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;
extern float controlTime;
extern float t;



// 映射函数：把 0–255 转换成 [-1,1] 或 [0,1]
void mapControlsFromEspNow() {
  // 油门：LX → 0~1
  controlThrottle = rxData.j1PotX / 255.0f;
  if (controlThrottle < 0.05f) controlThrottle = 0.0f;

  // 偏航：LY → -1~1，向前推减小 → 取反
  controlYaw = -(rxData.j1PotY - 127) / 128.0f;
  if (fabs(controlYaw) < 0.05f) controlYaw = 0.0f;

  // 横滚：RX → -1~1
  controlRoll = (rxData.j2PotX - 127) / 128.0f;
  if (fabs(controlRoll) < 0.05f) controlRoll = 0.0f;

  // 俯仰：RY → -1~1，向前推减小 → 取反
  controlPitch = -(rxData.j2PotY - 127) / 128.0f;
  if (fabs(controlPitch) < 0.05f) controlPitch = 0.0f;

  // 模式开关：简单映射成 0/1
  controlMode = rxData.tSwitch1 ? 0.0f : 1.0f;

  // 更新时间戳（用于 failsafe）
  controlTime = t;
}

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == sizeof(rxData)) {
      memcpy(&rxData, data, sizeof(rxData));
      mapControlsFromEspNow();
    //   // 打印接收到的数据
    //   Serial.println("=== ESP-NOW Data Received ===");
    //   Serial.print("j1PotX: "); Serial.print(rxData.j1PotX);
    //   Serial.print(" j1PotY: "); Serial.print(rxData.j1PotY);
    //   Serial.print(" j2PotX: "); Serial.print(rxData.j2PotX);
    //   Serial.print(" j2PotY: "); Serial.print(rxData.j2PotY);
    //   Serial.print(" R1: "); Serial.print(rxData.buttonR1);
    //   Serial.print(" R2: "); Serial.print(rxData.buttonR2);
    //   Serial.print(" LB: "); Serial.print(rxData.buttonLB);
    //   Serial.print(" RB: "); Serial.print(rxData.buttonRB);
    //   Serial.print(" J1: "); Serial.print(rxData.j1Button); 
    //   Serial.print(" J2: "); Serial.print(rxData.j2Button);
    //   Serial.print(" T1: "); Serial.println(rxData.tSwitch1);
    //   Serial.println("=============================");
    // } else {
    //   Serial.print("Unexpected data length: ");
    //   Serial.println(len);
    // }
    // 打印映射后的控制量
      Serial.println("=== 控制量更新 ===");
      Serial.print("Throttle: "); Serial.println(controlThrottle);
      Serial.print("Yaw: "); Serial.println(controlYaw);
      Serial.print("Roll: "); Serial.println(controlRoll);
      Serial.print("Pitch: "); Serial.println(controlPitch);
      Serial.print("Mode: "); Serial.println(controlMode);
      Serial.println("=================");
    } else {
      Serial.print("Unexpected data length: ");
      Serial.println(len);
    }
  }
};

// // 接收回调
// void onEspNowRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
//   if (len == sizeof(rxData)) {
//     memcpy(&rxData, incomingData, sizeof(rxData));
//     mapControlsFromEspNow();
//   }
// }


std::vector<ESP_NOW_Peer_Class *> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (!new_master->add_peer()) {
      Serial.println("Failed to register the new master");
      delete new_master;
      return;
    }
    masters.push_back(new_master);
    Serial.printf("Successfully registered master " MACSTR " (total masters: %zu)\n", MAC2STR(new_master->addr()), masters.size());
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}


#endif