#include "MAIN_PROPULSION_CAN.h"
#include <stdexcept>
#include <cstring>

// 初始化静态阈值
MAIN_PROPULSION_CAN::TimingThresholds MAIN_PROPULSION_CAN::currentTimingThresholds = {200000, 80000, 120000, 400, 500, 5000};
MAIN_PROPULSION_CAN::RPMThresholds MAIN_PROPULSION_CAN::currentRPMThresholds = {-1000, 1000, true};
MAIN_PROPULSION_CAN::VoltageThresholds MAIN_PROPULSION_CAN::currentVoltageThresholds = {0, 800, 750};
MAIN_PROPULSION_CAN::CurrentThresholds MAIN_PROPULSION_CAN::currentCurrentThresholds = {600, 500};
MAIN_PROPULSION_CAN::TemperatureThresholds MAIN_PROPULSION_CAN::currentTemperatureThresholds = {150, 90};

// 阈值设置实现
void MAIN_PROPULSION_CAN::setTimingThresholds(const TimingThresholds& thresholds) { currentTimingThresholds = thresholds; }
void MAIN_PROPULSION_CAN::setRPMThresholds(const RPMThresholds& thresholds) { currentRPMThresholds = thresholds; }
void MAIN_PROPULSION_CAN::setVoltageThresholds(const VoltageThresholds& thresholds) { currentVoltageThresholds = thresholds; }
void MAIN_PROPULSION_CAN::setCurrentThresholds(const CurrentThresholds& thresholds) { currentCurrentThresholds = thresholds; }
void MAIN_PROPULSION_CAN::setTemperatureThresholds(const TemperatureThresholds& thresholds) { currentTemperatureThresholds = thresholds; }

// 阈值获取实现
MAIN_PROPULSION_CAN::TimingThresholds MAIN_PROPULSION_CAN::getCurrentThresholds() { return currentTimingThresholds; }
MAIN_PROPULSION_CAN::RPMThresholds MAIN_PROPULSION_CAN::getCurrentRPMThresholds() { return currentRPMThresholds; }
MAIN_PROPULSION_CAN::VoltageThresholds MAIN_PROPULSION_CAN::getCurrentVoltageThresholds() { return currentVoltageThresholds; }
MAIN_PROPULSION_CAN::CurrentThresholds MAIN_PROPULSION_CAN::getCurrentCurrentThresholds() { return currentCurrentThresholds; }
MAIN_PROPULSION_CAN::TemperatureThresholds MAIN_PROPULSION_CAN::getCurrentTemperatureThresholds() { return currentTemperatureThresholds; }

// 状态检查实现
int MAIN_PROPULSION_CAN::checkOutputTiming(int64_t timeInterval) {
    auto thresholds = getCurrentThresholds();
    if (timeInterval > thresholds.output_no_response) return 3;
    if (timeInterval < thresholds.output_too_fast) return 2;
    if (timeInterval > thresholds.output_too_slow) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkFeedbackTiming(int64_t timeBack) {
    auto thresholds = getCurrentThresholds();
    if (timeBack > thresholds.feedback_no_response) return 2;
    if (timeBack > thresholds.feedback_slow) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkRPMStatus(const uint8_t* data, size_t offset) {
    int16_t rpm = parseRPM(data, offset);
    auto thresholds = getCurrentRPMThresholds();
    if (!thresholds.allow_negative && rpm < 0) return 2;
    if (rpm < thresholds.min_rpm || rpm > thresholds.max_rpm) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkVoltageStatus(const uint8_t* data, size_t offset) {
    uint16_t voltage = parseVoltage(data, offset);
    auto thresholds = getCurrentVoltageThresholds();
    if (voltage > thresholds.max_voltage) return 3;
    if (voltage > thresholds.high_voltage_threshold) return 2;
    if (voltage < thresholds.min_voltage) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkCurrentStatus(const uint8_t* data, size_t offset, bool isMotorCurrent) {
    uint16_t current = parseCurrent(data, offset);
    auto thresholds = getCurrentCurrentThresholds();
    uint16_t max = isMotorCurrent ? thresholds.max_motor_current : thresholds.max_input_current;
    return current > max ? 1 : 0;
}

int MAIN_PROPULSION_CAN::checkTemperatureStatus(const uint8_t* data, size_t offset) {
    uint8_t temp = parseTemperature(data, offset);
    auto thresholds = getCurrentTemperatureThresholds();
    if (temp > thresholds.max_temp) return 2;
    if (temp > thresholds.warning_temp) return 1;
    return 0;
}

// 数据解析实现
int16_t MAIN_PROPULSION_CAN::parseRPM(const uint8_t* data, size_t offset) { 
    return static_cast<int16_t>((data[offset+1] << 8) | data[offset]); 
}

uint16_t MAIN_PROPULSION_CAN::parseVoltage(const uint8_t* data, size_t offset) { 
    return (data[offset+1] << 8) | data[offset]; 
}

uint16_t MAIN_PROPULSION_CAN::parseCurrent(const uint8_t* data, size_t offset) { 
    return (data[offset+1] << 8) | data[offset]; 
}

uint8_t MAIN_PROPULSION_CAN::parseTemperature(const uint8_t* data, size_t offset) { 
    return data[offset]; 
}

uint8_t MAIN_PROPULSION_CAN::parseCommandBits(const uint8_t* data, size_t offset) { 
    return data[offset]; 
}

uint8_t MAIN_PROPULSION_CAN::parseStatusBits(const uint8_t* data, size_t offset) { 
    return data[offset]; 
}

// 温度相关状态检查
int MAIN_PROPULSION_CAN::checkBearingTempStatus(const uint8_t* data, size_t offset) {
    uint8_t temp = parseBearingTemp(data, offset);
    if(temp > getCurrentTemperatureThresholds().max_temp) return 2;
    if(temp > getCurrentTemperatureThresholds().warning_temp) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkWindingTempStatus(const uint8_t* data, size_t offset) {
    return checkBearingTempStatus(data, offset); // 共用轴承温度阈值
}

int MAIN_PROPULSION_CAN::checkOilTempStatus(const uint8_t* data, size_t offset) {
    uint8_t temp = parseBearingTemp(data, offset);
    if(temp > 90) return 2;  // 油液特殊阈值
    if(temp > 80) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkInverterTempStatus(const uint8_t* data, size_t offset) {
    return checkBearingTempStatus(data, offset); // 共用轴承温度阈值
}

// 压力检查（单位：kPa）
int MAIN_PROPULSION_CAN::checkInletPressureStatus(const uint8_t* data, size_t offset) {
    uint16_t pressure = parsePressure(data, offset);
    if(pressure > 1000) return 2;
    if(pressure > 900) return 1;
    return 0;
}

int MAIN_PROPULSION_CAN::checkOutletPressureStatus(const uint8_t* data, size_t offset) {
    return checkInletPressureStatus(data, offset); // 共用进口压力阈值
}

// 液位检查（单位：%）
int MAIN_PROPULSION_CAN::checkCompensatorLevelStatus(const uint8_t* data, size_t offset) {
    uint8_t level = parseCompensatorLevel(data, offset);
    if(level < 10) return 2;
    if(level < 20) return 1;
    return 0;
}

// 数据解析实现
uint8_t MAIN_PROPULSION_CAN::parseBearingTemp(const uint8_t* data, size_t offset) {
    return data[offset]; // 单位1℃
}

uint16_t MAIN_PROPULSION_CAN::parsePressure(const uint8_t* data, size_t offset) {
    return (data[offset+1] << 8) | data[offset]; // 单位1kPa
}

uint8_t MAIN_PROPULSION_CAN::parseCompensatorLevel(const uint8_t* data, size_t offset) {
    return data[offset]; // 单位1%
}