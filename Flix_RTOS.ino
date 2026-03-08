// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Main firmware file 主程序文件20251226
// 参考嘉立创开源项目：ESP32迷你无人机 https://oshwhub.com/malagis/esp32-mini-plane


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
	print("初始化完成！\n");
}

void loop() {
	static uint32_t last = micros();
	uint32_t start = micros();
	uint32_t loop_dt = start - last;
	last = start;

	readIMU();
	step();
	// readRC();					//sbus与光流传感器d4引脚冲突
	estimate();
	// readVL53L0X();   	//tof定高
	// readFlow();        		//光流传感器读取
	control();
	sendMotors();
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();

	uint32_t end = micros();
	uint32_t exec_time = end - start;
static int counter = 0;
if (++counter >= 200) {
    Serial.print("loop exec = ");
    Serial.print(exec_time);
    Serial.print(" us, period = ");
    Serial.print(loop_dt);
    Serial.println(" us");
    counter = 0;
}
}
