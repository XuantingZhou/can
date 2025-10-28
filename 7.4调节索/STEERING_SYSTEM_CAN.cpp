/*
 * @Author: ZTX
 * @Date: 2025-06-29 13:46:32
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-04 08:59:37
 * @FilePath: /6.29/STEERING_SYSTEM_CAN.cpp
 * @Description: 操舵系统模块代码
 */
#include "STEERING_SYSTEM_CAN.h"
#include <stdexcept>
#include <cstring>
#include <iostream>
// 初始化静态成员变量
STEERING_SYSTEM_CAN::TimingThresholds STEERING_SYSTEM_CAN::currentTimingThresholds = {
    200000,  // output_no_response
    80000,   // output_too_fast
    120000,  // output_too_slow
    400,     // feedback_normal
    500,     // feedback_slow
    5000     // feedback_no_response
};
STEERING_SYSTEM_CAN::RPMThresholds STEERING_SYSTEM_CAN::currentRPMThresholds = {
    -300,    // min_rpm (舵角范围)
    300,     // max_rpm (舵角范围)
    true     // allow_negative (允许负舵角)
};
STEERING_SYSTEM_CAN::VoltageThresholds STEERING_SYSTEM_CAN::currentVoltageThresholds = {
    220,     // min_voltage
    240,     // high_voltage_threshold
    260      // max_voltage
};
STEERING_SYSTEM_CAN::CurrentThresholds STEERING_SYSTEM_CAN::currentCurrentThresholds = {
    100,     // max_motor_current
    50       // max_input_current
};
// 设置时间阈值
void STEERING_SYSTEM_CAN::setTimingThresholds(const TimingThresholds& thresholds) {
    currentTimingThresholds = thresholds;
}
// 获取当前时间阈值
STEERING_SYSTEM_CAN::TimingThresholds STEERING_SYSTEM_CAN::getCurrentThresholds() {
    return currentTimingThresholds;
}
// 设置舵角和舵速阈值
void STEERING_SYSTEM_CAN::setRPMThresholds(const RPMThresholds& thresholds) {
    currentRPMThresholds = thresholds;
}
// 获取当前舵角和舵速阈值
STEERING_SYSTEM_CAN::RPMThresholds STEERING_SYSTEM_CAN::getCurrentRPMThresholds() {
    return currentRPMThresholds;
}
// 设置电压阈值
void STEERING_SYSTEM_CAN::setVoltageThresholds(const VoltageThresholds& thresholds) {
    currentVoltageThresholds = thresholds;
}
// 获取当前电压阈值
STEERING_SYSTEM_CAN::VoltageThresholds STEERING_SYSTEM_CAN::getCurrentVoltageThresholds() {
    return currentVoltageThresholds;
}
// 设置电流阈值
void STEERING_SYSTEM_CAN::setCurrentThresholds(const CurrentThresholds& thresholds) {
    currentCurrentThresholds = thresholds;
}
// 获取当前电流阈值
STEERING_SYSTEM_CAN::CurrentThresholds STEERING_SYSTEM_CAN::getCurrentCurrentThresholds() {
    return currentCurrentThresholds;
}
// 判断输出指令状态
int STEERING_SYSTEM_CAN::checkOutputTiming(int64_t timeInterval) {
    auto thresholds = getCurrentThresholds();
    if (timeInterval > thresholds.output_no_response) {
        return 3;
    } else if (timeInterval < thresholds.output_too_fast) {
        return 2;
    } else if (timeInterval > thresholds.output_too_slow) {
        return 1;
    }
    return 0;
}
// 判断反馈状态
int STEERING_SYSTEM_CAN::checkFeedbackTiming(int64_t timeBack) {
    auto thresholds = getCurrentThresholds();
    if (timeBack > thresholds.feedback_no_response) {
        return 2;
    } else if (timeBack > thresholds.feedback_slow) {
        return 1;
    }
    return 0;
}
// 从原始数据解析舵角
int16_t STEERING_SYSTEM_CAN::parseRudderAngle(const uint8_t* data, size_t offset) { 
    return static_cast<int16_t>((data[offset+1] << 8) | data[offset]); 
}
// 从原始数据解析舵速
float STEERING_SYSTEM_CAN::parseRudderSpeed(const uint8_t* data, size_t offset) { 
    uint16_t raw = (data[offset+1] << 8) | data[offset]; 
    return static_cast<float>(raw) / 10.0f; 
}
float STEERING_SYSTEM_CAN::parseBusVoltage(const uint8_t* data, size_t offset) {
    // 假设母线电压数据是大端存储的16位整数，位于指定的offset
    return (data[offset + 1] << 8) | data[offset];
}
// 检查RPM状态
int STEERING_SYSTEM_CAN::checkRPMStatus(const uint8_t* data, size_t offset) {
    int16_t rpm = parseRudderAngle(data, offset);
    auto thresholds = getCurrentRPMThresholds();
    if (!thresholds.allow_negative && rpm < 0) return 2;
    if (rpm < thresholds.min_rpm || rpm > thresholds.max_rpm) return 1;
    return 0;
}
// 检查电压状态
int STEERING_SYSTEM_CAN::checkVoltageStatus(const uint8_t* data, size_t offset) {
    uint16_t voltage = parseVoltage(data, offset);
    auto thresholds = getCurrentVoltageThresholds();
    if (voltage > thresholds.max_voltage) return 3;
    if (voltage > thresholds.high_voltage_threshold) return 2;
    if (voltage < thresholds.min_voltage) return 1;
    return 0;
}
// 检查电流状态
int STEERING_SYSTEM_CAN::checkCurrentStatus(const uint8_t* data, size_t offset, bool isMotorCurrent) {
    uint16_t current = parseCurrent(data, offset);
    auto thresholds = getCurrentCurrentThresholds();
    uint16_t max = isMotorCurrent ? thresholds.max_motor_current : thresholds.max_input_current;
    return current > max ? 1 : 0;
}
// 解析电压
float STEERING_SYSTEM_CAN::parseVoltage(const uint8_t* data, size_t offset) { 
    return (data[offset+1] << 8) | data[offset]; 
}
// 解析电流
float STEERING_SYSTEM_CAN::parseCurrent(const uint8_t* data, size_t offset) { 
    return (data[offset+1] << 8) | data[offset]; 
}
// 检查舵角状态
int STEERING_SYSTEM_CAN::checkRudderAngleStatus(const uint8_t* data, size_t offset) {
    int16_t angle = parseRudderAngle(data, offset);
    auto thresholds = getCurrentRPMThresholds();
    if (angle < thresholds.min_rpm || angle > thresholds.max_rpm) return 1;
    return 0;
}
// 检查舵速状态
int STEERING_SYSTEM_CAN::checkRudderSpeedStatus(const uint8_t* data, size_t offset) {
    float speed = parseRudderSpeed(data, offset);
    if (speed < 0.0f || speed > 10.0f) return 1; // 超出范围
    return 0; // 正常
}
// 检查伸缩指令状态
int STEERING_SYSTEM_CAN::checkTelescopicCommandStatus(const uint8_t* data, size_t offset) {
    uint8_t command = parseTelescopicCommand(data, offset);
    if (command > 4) return 1; // 超出范围
    return 0; // 正常
}
// 检查授权状态
int STEERING_SYSTEM_CAN::checkExceedAuthStatus(const uint8_t* data, size_t offset) {
    uint8_t auth = parseExceedAuth(data, offset);
    if (auth > 1) return 1; // 超出范围
    return 0; // 正常
}
// 解析伸缩指令
uint8_t STEERING_SYSTEM_CAN::parseTelescopicCommand(const uint8_t* data, size_t offset) {
    return data[offset + 4];  // 直接返回第 5 字节的值
}
// 解析授权状态
uint8_t STEERING_SYSTEM_CAN::parseExceedAuth(const uint8_t* data, size_t offset) {
    return data[offset + 5];  // 直接返回第 6 字节的值
}
// 检查母线电压状态
int STEERING_SYSTEM_CAN::checkBusVoltageStatus(const uint8_t* data, size_t offset) {
    uint16_t voltage = parseVoltage(data, offset);
    auto thresholds = getCurrentVoltageThresholds();
    if (voltage > thresholds.max_voltage) return 1;
    return 0;
}
// 检查电机电流状态
int STEERING_SYSTEM_CAN::checkMotorCurrentStatus(const uint8_t* data, size_t offset) {
    uint16_t current = parseCurrent(data, offset);
    auto thresholds = getCurrentCurrentThresholds();
    if (current > thresholds.max_motor_current) return 1;
    return 0;
}
// 检查直流电流状态
int STEERING_SYSTEM_CAN::checkIdCurrentStatus(const uint8_t* data, size_t offset) {
    uint16_t current = parseCurrent(data, offset);
    auto thresholds = getCurrentCurrentThresholds();
    if (current > thresholds.max_input_current) return 1;
    return 0;
}
// 检查交流电流状态
int STEERING_SYSTEM_CAN::checkIqCurrentStatus(const uint8_t* data, size_t offset) {
    uint16_t current = parseCurrent(data, offset);
    // 假设Iq电流阈值与电机电流相同
    auto thresholds = getCurrentCurrentThresholds();
    if (current > thresholds.max_motor_current) return 1;
    return 0;
}
// 解析电机电流
float STEERING_SYSTEM_CAN::parseMotorCurrent(const uint8_t* data, size_t offset) {
    return (data[offset+1] << 8) | data[offset];
}
// 解析直流电流
float STEERING_SYSTEM_CAN::parseIdCurrent(const uint8_t* data, size_t offset) {
    return (data[offset+1] << 8) | data[offset];
}
// 解析交流电流
float STEERING_SYSTEM_CAN::parseIqCurrent(const uint8_t* data, size_t offset) {
    return (data[offset+1] << 8) | data[offset];
}
// 检查压力状态
int STEERING_SYSTEM_CAN::checkPressureStatus(const uint8_t* data, size_t offset) {
    uint8_t pressure = parsePressure(data, offset);
    if (pressure < 0 || pressure > 40) {
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
uint8_t STEERING_SYSTEM_CAN::parsePressure(const uint8_t* data, size_t offset) {
    return data[offset + 3];  // 直接返回第 4 字节的值
}
// 检查IGBT温度状态
int STEERING_SYSTEM_CAN::checkIGBTTemperatureStatus(const uint8_t* data, size_t offset) {
    float temperature = parseIGBTTemperature(data, offset);
    if (temperature < -555 || temperature > 200) {
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
// 检查电机温度状态
int STEERING_SYSTEM_CAN::checkMotorTemperatureStatus(const uint8_t* data, size_t offset) {
    float temperature = parseMotorTemperature(data, offset);
    if (temperature < -55 || temperature > 200) {
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
// 解析IGBT温度
float STEERING_SYSTEM_CAN::parseIGBTTemperature(const uint8_t* data, size_t offset) {
    uint8_t raw_temp = data[offset + 3];  // 获取原始温度值
    return (raw_temp - 55) * 1.0;  // 偏移55，单位为摄氏度
}

// 解析电机温度
float STEERING_SYSTEM_CAN::parseMotorTemperature(const uint8_t* data, size_t offset) {
    uint8_t raw_temp = data[offset + 4];  // 获取原始温度值
    return (raw_temp - 55) * 1.0;  // 偏移55，单位为摄氏度
}
// 检查阀状态
int STEERING_SYSTEM_CAN::checkValveStatus(const uint8_t* data, size_t offset) {
    uint8_t status = parseValveStatus(data, offset);
    if (status < 0 || status > 15) {
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
uint8_t STEERING_SYSTEM_CAN::parseValveStatus(const uint8_t* data, size_t offset) {
    return data[offset];  // 直接返回第 1 字节的值
}
// 检查缸角度状态
int STEERING_SYSTEM_CAN::checkCylinderAngleStatus(const uint8_t* data, size_t offset) {
    uint16_t angle = parseCylinderAngle(data, offset);
    if (angle < 0 || angle > 100) {
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
// 检查油位状态
int STEERING_SYSTEM_CAN::checkOilLevelStatus(const uint8_t* data, size_t offset) {
    uint16_t level = parseOilLevel(data, offset);
    if (level < 0 || level > 100) {
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
// 解析缸角度
uint16_t STEERING_SYSTEM_CAN::parseCylinderAngle(const uint8_t* data, size_t offset) {
    return (data[offset + 1] << 8) | data[offset];  // 解析16位整数
}
// 解析油位
uint16_t STEERING_SYSTEM_CAN::parseOilLevel(const uint8_t* data, size_t offset) {
    return (data[offset + 1] << 8) | data[offset];  // 解析16位整数
}
// 检查阀电流状态
int STEERING_SYSTEM_CAN::checkValveCurrentStatus(const uint8_t* data, size_t offset) {
    uint16_t current = parseValveCurrent(data, offset);
    if (current < 0 || current > 500) { // 假设使用0-500表示0-5A
        return 1; // 错误状态
    }
    return 0; // 正常状态
}
// 解析阀电流
uint16_t STEERING_SYSTEM_CAN::parseValveCurrent(const uint8_t* data, size_t offset) {
    return (data[offset + 1] << 8) | data[offset];  // 解析16位整数
}
// 检查自检状态值
int STEERING_SYSTEM_CAN::checkSelfcheckStatus(const uint8_t* data, size_t offset) {
    uint8_t status = data[offset];
    if (status != 0) return 1; // 自检失败
    return 0; // 自检成功
}
uint8_t STEERING_SYSTEM_CAN::parseSelfcheckStatus(const uint8_t* data, size_t offset) {
    return data[offset];  // 直接返回第 1 字节的值
}
// 检查模式状态
int STEERING_SYSTEM_CAN::checkModeStatus(const uint8_t* data, size_t offset) {
    uint8_t mode = data[offset];
    if (mode > 2) return 1; // 超出范围
    return 0; // 正常
}
uint8_t STEERING_SYSTEM_CAN::parseModeStatus(const uint8_t* data, size_t offset) {
    return data[offset + 1];  // 直接返回第 2 字节的值
}