// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 参考嘉立创开源项目：ESP32迷你无人机 https://oshwhub.com/malagis/esp32-mini-plane
// Flix_RTOS: https://github.com/zexi315/Flix_RTOS

#include "vector.h"
#include "quaternion.h"
#include "util.h"

#define WIFI_ENABLED 1

float t = NAN; // current step time, s
float dt; // time delta from previous step, s
float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot's inputs, range [-1, 1]
float controlMode = NAN;
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]

// ================= FreeRTOS 任务 =================

// 控制
void TaskControl(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t dt_ms = pdMS_TO_TICKS(2); // 2ms

    while (1) {
        readIMU();
        step();
        estimate();
        control();
        sendMotors();
        
    }
}

Rate debugRate(5); // 每秒 5 次

// 传感器
void TaskSensors(void *pvParameters) {
	const TickType_t dt_ms = pdMS_TO_TICKS(10); // 10ms
    while (1) {
        readVL53L0X();   // TOF
        readFlow();      // 光流
        computeFlowVelocity(); // 计算速度
        // if (debugRate) debug();  // 每秒 5 次
        vTaskDelay(dt_ms); 
    }
}


// // 通信
// void TaskComm(void *pvParameters) {
// 	const TickType_t dt_ms = pdMS_TO_TICKS(10); // 10ms
//     while (1) {
//         handleInput();
// #if WIFI_ENABLED
//         processMavlink();
// #endif
//         vTaskDelay(dt_ms); 
//     }
// }

// // 日志
// void TaskLog(void *pvParameters) {
// 	const TickType_t dt_ms = pdMS_TO_TICKS(50); // 50ms
//     while (1) {
//         logData();
//         syncParameters();
//         vTaskDelay(dt_ms); 
//     }
// }



void setup() {
	Serial.begin(115200);
	// resetParameters(); //清空存储并重启
	print("程序初始化！\n");
	disableBrownOut();
	setupParameters();
	// dumpParameters(); //存储值
	setupLED();
	setupVL53L0X();   //tof定高初始化
	setupFlow();     		 //光流传感器初始化
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	// setupRC();     		//sbus与光流传感器d4引脚冲突
	setLED(false);

	// 创建任务
  xTaskCreatePinnedToCore(TaskControl, "Control", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 3, NULL, 0);
//   xTaskCreatePinnedToCore(TaskComm, "Comm", 4096, NULL, 2, NULL, 0);
//   xTaskCreatePinnedToCore(TaskLog, "Log", 4096, NULL, 1, NULL, 0);

	print("初始化完成！\n");

}

void loop() {
	// readRC();
    // readIMU();
    // step();
    // estimate();
    // control();
    // sendMotors();
    // readVL53L0X();   // TOF
    // readFlow();      // 光流
//     handleInput();
#if WIFI_ENABLED
    processMavlink();
#endif
    logData();
    syncParameters();
}
