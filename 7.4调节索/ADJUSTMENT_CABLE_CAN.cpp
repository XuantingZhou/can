#include "ADJUSTMENT_CABLE_CAN.h"
#include <stdexcept>
#include <cstring>

// 初始化静态阈值
ADJUSTMENT_CABLE_CAN::TimingThresholds ADJUSTMENT_CABLE_CAN::currentTimingThresholds = {200000, 80000, 120000, 400, 500, 5000};
ADJUSTMENT_CABLE_CAN::ChainLengthThresholds ADJUSTMENT_CABLE_CAN::currentChainLengthThresholds = {5, 35};
ADJUSTMENT_CABLE_CAN::SpeedThresholds ADJUSTMENT_CABLE_CAN::currentSpeedThresholds = {15};
ADJUSTMENT_CABLE_CAN::PressureThresholds ADJUSTMENT_CABLE_CAN::currentPressureThresholds = {23};
ADJUSTMENT_CABLE_CAN::ValveStatusThresholds ADJUSTMENT_CABLE_CAN::currentValveStatusThresholds = {100, -100};

// 阈值设置实现
void ADJUSTMENT_CABLE_CAN::setTimingThresholds(const TimingThresholds& thresholds) { currentTimingThresholds = thresholds; }
void ADJUSTMENT_CABLE_CAN::setChainLengthThresholds(const ChainLengthThresholds& thresholds) { currentChainLengthThresholds = thresholds; }
void ADJUSTMENT_CABLE_CAN::setSpeedThresholds(const SpeedThresholds& thresholds) { currentSpeedThresholds = thresholds; }
void ADJUSTMENT_CABLE_CAN::setPressureThresholds(const PressureThresholds& thresholds) { currentPressureThresholds = thresholds; }
void ADJUSTMENT_CABLE_CAN::setValveStatusThresholds(const ValveStatusThresholds& thresholds) { currentValveStatusThresholds = thresholds; }

// 阈值获取实现
ADJUSTMENT_CABLE_CAN::TimingThresholds ADJUSTMENT_CABLE_CAN::getCurrentTimingThresholds() { return currentTimingThresholds; }
ADJUSTMENT_CABLE_CAN::ChainLengthThresholds ADJUSTMENT_CABLE_CAN::getCurrentChainLengthThresholds() { return currentChainLengthThresholds; }
ADJUSTMENT_CABLE_CAN::SpeedThresholds ADJUSTMENT_CABLE_CAN::getCurrentSpeedThresholds() { return currentSpeedThresholds; }
ADJUSTMENT_CABLE_CAN::PressureThresholds ADJUSTMENT_CABLE_CAN::getCurrentPressureThresholds() { return currentPressureThresholds; }
ADJUSTMENT_CABLE_CAN::ValveStatusThresholds ADJUSTMENT_CABLE_CAN::getCurrentValveStatusThresholds() { return currentValveStatusThresholds; }

// 状态检查实现
int ADJUSTMENT_CABLE_CAN::checkOutputTiming(int64_t timeInterval) {
    auto thresholds = getCurrentTimingThresholds();
    if (timeInterval > thresholds.output_no_response) return 3;
    if (timeInterval < thresholds.output_too_fast) return 2;
    if (timeInterval > thresholds.output_too_slow) return 1;
    return 0;
}

int ADJUSTMENT_CABLE_CAN::checkCommandStatus(const uint8_t* data, size_t offset) {
    return data[offset] == 0x01 ? 1 : 0;
}

int ADJUSTMENT_CABLE_CAN::checkEmergencyDropLinkStatus(const uint8_t* data, size_t offset) {
    return data[offset] == 0x01 ? 1 : 0;
}

int ADJUSTMENT_CABLE_CAN::checkSystemStatus(const uint8_t* data, size_t offset) {
    return data[offset] > 0 ? 1 : 0;
}

int ADJUSTMENT_CABLE_CAN::checkZLXStatus(const uint8_t* data, size_t offset) {
    return data[offset] > 0 ? 1 : 0;
}

int ADJUSTMENT_CABLE_CAN::checkChainLengthStatus(const uint8_t* data, size_t offset) {
    double chain_length = parseChainLength(data, offset);
    auto thresholds = getCurrentChainLengthThresholds();
    if (chain_length < thresholds.min_length) return 2;
    if (chain_length > thresholds.max_length) return 1;
    return 0;
}

int ADJUSTMENT_CABLE_CAN::checkSpeedStatus(const uint8_t* data, size_t offset) {
    double speed = parseSpeed(data, offset);
    auto thresholds = getCurrentSpeedThresholds();
    if (speed > thresholds.max_speed) return 1;
    return 0;
}

int ADJUSTMENT_CABLE_CAN::checkPressureStatus(const uint8_t* data, size_t offset) {
    double pressure = parsePressure(data, offset);
    auto thresholds = getCurrentPressureThresholds();
    if (pressure > thresholds.max_pressure) return 1;
    return 0;
}

int ADJUSTMENT_CABLE_CAN::checkValveStatus(const uint8_t* data, size_t offset) {
    double valve_status = parseValveStatus(data, offset);
    auto thresholds = getCurrentValveStatusThresholds();
    if (valve_status > thresholds.max_valve_status) return 1;
    if (valve_status < thresholds.min_valve_status) return 2;
    return 0;
}

int ADJUSTMENT_CABLE_CAN::checkAlarmGroup1Status(const uint8_t* data, size_t offset) {
    return data[offset] > 0 ? 1 : 0;
}

int ADJUSTMENT_CABLE_CAN::checkAlarmGroup2Status(const uint8_t* data, size_t offset) {
    return data[offset] > 0 ? 1 : 0;
}

// 数据解析实现
int8_t ADJUSTMENT_CABLE_CAN::parseOnoffCmd(const uint8_t* data, size_t offset) {
    return data[offset];
}

int8_t ADJUSTMENT_CABLE_CAN::parseRScable(const uint8_t* data, size_t offset) {
    return data[offset];
}

double ADJUSTMENT_CABLE_CAN::parseProportionValveStatus(const uint8_t* data, size_t offset) {
    return static_cast<double>(data[offset]);
}

int8_t ADJUSTMENT_CABLE_CAN::parseOnoffCtrl1(const uint8_t* data, size_t offset) {
    return data[offset];
}

int8_t ADJUSTMENT_CABLE_CAN::parseOnoffCtrl2(const uint8_t* data, size_t offset) {
    return data[offset];
}

double ADJUSTMENT_CABLE_CAN::parseChainLength(const uint8_t* data, size_t offset) {
    return static_cast<double>((data[offset+1] << 8) | data[offset]) / 100.0;
}

double ADJUSTMENT_CABLE_CAN::parseSpeed(const uint8_t* data, size_t offset) {
    return static_cast<double>(data[offset]);
}

double ADJUSTMENT_CABLE_CAN::parsePressure(const uint8_t* data, size_t offset) {
    return static_cast<double>((data[offset+1] << 8) | data[offset]) / 10.0;
}

double ADJUSTMENT_CABLE_CAN::parseValveStatus(const uint8_t* data, size_t offset) {
    return static_cast<double>(data[offset]);
}

int32_t ADJUSTMENT_CABLE_CAN::parseAlarmGroup1(const uint8_t* data, size_t offset) {
    return data[offset];
}

int32_t ADJUSTMENT_CABLE_CAN::parseAlarmGroup2(const uint8_t* data, size_t offset) {
    return data[offset];
}
