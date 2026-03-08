// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix


#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_VL53L0X.h>
#include <Bitcraze_PMW3901.h>

// ============== PMW3901参数 ==============
SPIClass hspi(HSPI);  // 声明 HSPI
Bitcraze_PMW3901 flow(15, &hspi);  // CS=GPIO4，使用 HSPI
int16_t deltaX = 0, deltaY = 0;


// ============== VL53L0x参数 ==============
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000 // 400kHz
#define ALT_P 0
#define ALT_I 0
#define ALT_D 0
#define ALT_I_LIM 0.3
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float hoverEnabled = 0;   //悬停开关 0 = false, 1 = true

float heightMeasured = NAN; // 滤波后高度，单位 m 
const float EMA_ALPHA = 0.6f; // 0..1，越大越平滑（0.6~0.85 常用） 
unsigned long lastTofMs = 0; 
const unsigned long TOF_TIMEOUT_MS = 200;

PID altPID(ALT_P, ALT_I, ALT_D, ALT_I_LIM);
float altTarget = NAN;   // 目标高度（米）



// ============== 角速率环（内环）参数 ==============
#define PITCHRATE_P 1.5 // 增大P值提高响应速度
#define PITCHRATE_I 0.008 // 中等I值补偿电机差异
#define PITCHRATE_D 0.09 // 小D值抑制震荡
#define PITCHRATE_I_LIM 0.3 // 限制积分积累
#define ROLLRATE_P PITCHRATE_P // 横滚和俯仰使用相同参数
#define ROLLRATE_I PITCHRATE_I 
#define ROLLRATE_D PITCHRATE_D 
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 3 // 偏航需要更高的P值（惯性较小）
#define YAWRATE_I 0.0 // 中等I值补偿
#define YAWRATE_D 0.0 // 小D值
#define YAWRATE_I_LIM 0.3
// ============== 角度环（外环）参数 ==============
#define ROLL_P 4 // 较高的P值快速响应
#define ROLL_I 0 // 角度环通常不需要I项
#define ROLL_D 0 // 角度环通常不需要D项
#define PITCH_P ROLL_P // 横滚和俯仰相同
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3 // 偏航响应稍慢

// ============== 限制值 ==============
#define PITCHRATE_MAX radians(360) // 高转速限制（1000°/s）
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300) // 偏航转速稍低
#define TILT_MAX radians(30) // 最大倾斜角30°
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

const int RAW = 0, ACRO = 1, STAB = 2, AUTO = 3; // flight modes
int mode = STAB;
bool armed = false;

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;


void setupFlow() {
	// 初始化 HSPI 总线，指定引脚
   hspi.begin(14, 4, 13, 15); // HSPI: SCK=14, MISO=4, MOSI=13, CS=15
	// 初始化光流传感器
  if (!flow.begin()) {
    Serial.println("PMW3901 init failed!");
    while(1);
  }
  Serial.println("PMW3901 initialized");
}

void readFlow() {
    // 读取光流位移
    flow.readMotionCount(&deltaX, &deltaY);

    // 调试输出
    // Serial.print("Flow ΔX: ");
    // Serial.print(deltaX);
    // Serial.print(" ΔY: ");
    // Serial.println(deltaY);
}


void setupVL53L0X(){  //初始化TOF测距
	Serial.print("Setup VL53L0X");
	
	Wire.begin(SDA_PIN, SCL_PIN); // 指定 SDA 和 SCL 
	Wire.setClock(I2C_FREQ); // 可选：设置时钟频率
	delay(20);

	Serial.print("初始化I2C总线 (SDA=GPIO");
  Serial.print(SDA_PIN);
  Serial.print(", SCL=GPIO");
  Serial.print(SCL_PIN);
  Serial.println(")...");

	const int MAX_TRIES = 3; 
	for (int i = 0; i < MAX_TRIES; i++) { 
		if (lox.begin()) { 
			lox.startRangeContinuous(); // 启动连续测距 
			Serial.println("VL53L0X initialized"); 
		} 
			Serial.print("VL53L0X init failed, retry ");
		 	Serial.println(i + 1); 
		 	delay(100); 
		 } 

		 Serial.println(F("Failed to boot VL53L0X after retries")); 

  // if (!lox.begin()) {
  //   Serial.println(F("Failed to boot VL53L0X"));
  //   while(1);
  // }

  // lox.startRangeContinuous();
	// Serial.print("VL53L0X initialized\n");
}

void readVL53L0X(){
	// if (lox.isRangeComplete()) {
  //   Serial.print("Distance in mm: ");
  //   Serial.println(lox.readRange());
  // }
	if (!lox.isRangeComplete()) return; 
	uint16_t mm = lox.readRange(); // mm 

	if (lox.timeoutOccurred()) { 
		heightMeasured = NAN; 
		return; 
	} 
	// 过滤异常值（可选） 
	if (mm == 0 || mm > 2000) { // 例：忽略 0 或超出 2m 的读数 
		return; 
	} 

	float m = mm / 1000.0f; // 转换为米 
	if (isnan(heightMeasured)) heightMeasured = m; 
	else heightMeasured = EMA_ALPHA * heightMeasured + (1.0f - EMA_ALPHA) * m; 

	lastTofMs = millis();

	// Serial.print("Distance in m: ");
  // Serial.println(heightMeasured,3);
}

void control() {
	interpretControls();
	failsafe();
	controlAttitude();
	controlAltitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = STAB;
	if (controlMode < 0.75) mode = STAB;
	if (controlMode > 0.75) mode = STAB;

	if (!hoverEnabled) {
    	altTarget = NAN;   // 退出悬停时重置
	}

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

	if (controlThrottle < 0.05 && controlYaw > 0.95) armed = true; // arm gesture
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	thrustTarget = controlThrottle;

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax, controlPitch * tiltMax, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < 0.1) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}


void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < 0.1) return; // skip rates control

	Vector error = ratesTarget - rates;//ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (thrustTarget < 0.1) {
		motors[0] = 0.1; // idle thrust
		motors[1] = 0.1;
		motors[2] = 0.1;
		motors[3] = 0.1;
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

const char* getModeName() {
	switch (mode) {
		case RAW: return "RAW";
		case ACRO: return "ACRO";
		case STAB: return "STAB";
		case AUTO: return "AUTO";
		default: return "UNKNOWN";
	}
}

float hoverThrust = 0.52f;   // 悬停基准油门

void controlAltitude() {

    if (hoverEnabled == 0) return;
		if (!armed) return;

    // TOF 无效则不控制
    if (isnan(heightMeasured)) return;

    // 初始化目标高度（第一次进入定高时）
    if (isnan(altTarget)) {
        altTarget = heightMeasured;
        return;
    }

    // 计算高度误差
    float error = altTarget - heightMeasured;

    // PID 输出为推力修正量
    float correction = altPID.update(error);

    // thrustTarget 是 0~1 的归一化推力
    thrustTarget = hoverThrust + correction;

    // 限制推力范围，避免过大或掉落
    thrustTarget = constrain(thrustTarget, 0.1f, 0.8f);
}