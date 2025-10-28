#include "EMERGENCY_AUXILIARY_PROPULSION_CAN.h"
#include <stdexcept>
#include <cstring>

// 初始化静态阈值
EMERGENCY_AUXILIARY_PROPULSION_CAN::TimingThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::currentTimingThresholds = {
    200000, 80000, 120000, 400, 500, 5000
};

EMERGENCY_AUXILIARY_PROPULSION_CAN::RPMThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::currentRPMThresholds = {
    -2000, 2000, true
};

EMERGENCY_AUXILIARY_PROPULSION_CAN::VoltageThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::currentVoltageThresholds = {
    0, 800, 750  // 0-800V, 高电压阈值750V
};

EMERGENCY_AUXILIARY_PROPULSION_CAN::CurrentThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::currentCurrentThresholds = {
    255  // 0-255A
};

EMERGENCY_AUXILIARY_PROPULSION_CAN::TemperatureThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::currentTemperatureThresholds = {
    150, 150  // 0-150℃
};

// 阈值设置实现
void EMERGENCY_AUXILIARY_PROPULSION_CAN::setTimingThresholds(const TimingThresholds& thresholds) {
    currentTimingThresholds = thresholds;
}

void EMERGENCY_AUXILIARY_PROPULSION_CAN::setRPMThresholds(const RPMThresholds& thresholds) {
    currentRPMThresholds = thresholds;
}

void EMERGENCY_AUXILIARY_PROPULSION_CAN::setVoltageThresholds(const VoltageThresholds& thresholds) {
    currentVoltageThresholds = thresholds;
}

void EMERGENCY_AUXILIARY_PROPULSION_CAN::setCurrentThresholds(const CurrentThresholds& thresholds) {
    currentCurrentThresholds = thresholds;
}

void EMERGENCY_AUXILIARY_PROPULSION_CAN::setTemperatureThresholds(const TemperatureThresholds& thresholds) {
    currentTemperatureThresholds = thresholds;
}

// 阈值获取实现
EMERGENCY_AUXILIARY_PROPULSION_CAN::TimingThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::getCurrentThresholds() {
    return currentTimingThresholds;
}

EMERGENCY_AUXILIARY_PROPULSION_CAN::RPMThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::getCurrentRPMThresholds() {
    return currentRPMThresholds;
}

EMERGENCY_AUXILIARY_PROPULSION_CAN::VoltageThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::getCurrentVoltageThresholds() {
    return currentVoltageThresholds;
}

EMERGENCY_AUXILIARY_PROPULSION_CAN::CurrentThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::getCurrentCurrentThresholds() {
    return currentCurrentThresholds;
}

EMERGENCY_AUXILIARY_PROPULSION_CAN::TemperatureThresholds EMERGENCY_AUXILIARY_PROPULSION_CAN::getCurrentTemperatureThresholds() {
    return currentTemperatureThresholds;
}

// 状态检查实现
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkOutputTiming(int64_t timeInterval) {
    auto thresholds = getCurrentThresholds();
    if (timeInterval > thresholds.output_no_response) return 3;
    if (timeInterval < thresholds.output_too_fast) return 2;
    if (timeInterval > thresholds.output_too_slow) return 1;
    return 0;
}

int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkFeedbackTiming(int64_t timeBack) {
    auto thresholds = getCurrentThresholds();
    if (timeBack > thresholds.feedback_no_response) return 2;
    if (timeBack > thresholds.feedback_slow) return 1;
    return 0;
}

bool EMERGENCY_AUXILIARY_PROPULSION_CAN::checkRPM(int16_t rpm, std::string& message) {
    auto thresholds = getCurrentRPMThresholds();
    if (!thresholds.allow_negative && rpm < 0) {
        message = "Negative RPM not allowed";
        return false;
    }
    if (rpm < thresholds.min_rpm || rpm > thresholds.max_rpm) {
        message = "RPM out of range";
        return false;
    }
    message = "RPM normal";
    return true;
}

// 数据解析实现
int16_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseRPM(const uint8_t* data, size_t offset) {
    uint16_t raw = (data[offset+1] << 8) | data[offset];
    bool is_negative = (raw & 0x8000);
    return is_negative ? -static_cast<int16_t>(raw & 0x7FFF) : static_cast<int16_t>(raw);
}

uint16_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseVoltage(const uint8_t data[], size_t offset) {
    return (data[offset+1] << 8) | data[offset]; // 单位1V
}

uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseCurrent(const uint8_t data[], size_t offset) {
    return data[offset]; // 单位1A
}

uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseMotorTemperature(const uint8_t data[], size_t offset) {
    return data[offset]; // 单位1℃
}

uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseDriverTemperature(const uint8_t data[], size_t offset) {
    return data[offset]; // 单位1℃
}

//32023
// 补偿器液位解析 (0-100%)
uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseCompensatorLevel(const uint8_t* data, size_t offset) {
    return data[offset];
}

// 伸缩机构状态解析 (0-100%)
uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseExtensionStatus(const uint8_t* data, size_t offset) {
    return data[offset + 1];
}

// 接近开关状态解析
uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseProximitySwitches(const uint8_t* data, size_t offset) {
    return data[offset + 2];
}

// 故障位1解析
uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseFaultBits1(const uint8_t* data, size_t offset) {
    return data[offset + 3];
}

// 故障位2解析
uint8_t EMERGENCY_AUXILIARY_PROPULSION_CAN::parseFaultBits2(const uint8_t* data, size_t offset) {
    return data[offset + 4];
}


// 状态检查实现
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkVoltageStatus(const uint8_t data[], size_t offset) {
    uint16_t voltage = parseVoltage(data, offset);
    auto thresholds = getCurrentVoltageThresholds();
    
    if (voltage > thresholds.high_voltage_threshold) return 2;
    if (voltage > thresholds.max_voltage) return 3;
    if (voltage < thresholds.min_voltage) return 1;
    return 0;
}

int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkCurrentStatus(const uint8_t data[], size_t offset) {
    uint8_t current = parseCurrent(data, offset);
    return (current > getCurrentCurrentThresholds().max_current) ? 1 : 0;
}

int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkMotorTempStatus(const uint8_t data[], size_t motor_temp_offset) {
    uint8_t temp = parseMotorTemperature(data, motor_temp_offset);
    const uint8_t min_temp = 0;
    
    if (temp < min_temp) return 2;
    if (temp > getCurrentTemperatureThresholds().max_driver_temp) return 1;
    return 0;
}

int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkDriverTempStatus(const uint8_t data[], size_t driver_temp_offset) {
    uint8_t temp = parseDriverTemperature(data, driver_temp_offset);
    const uint8_t min_temp = 0;
    
    if (temp < min_temp) return 2;
    if (temp > getCurrentTemperatureThresholds().max_driver_temp) return 1;
    return 0;
}


// 补偿器液位状态检查 (液位低于90%报警)
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkCompensatorLevelStatus(const uint8_t* data, size_t offset) {
    return parseCompensatorLevel(data, offset) < 90 ? 1 : 0;
}

// 伸缩机构状态检查
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkExtensionStatus(const uint8_t* data, size_t offset) {
    uint8_t status = parseExtensionStatus(data, offset);
    return status > 100 ? 1 : 0; // 超过100%为异常
}

// 接近开关状态检查
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkProximitySwitchesStatus(const uint8_t* data, size_t offset) {
    uint8_t switches = parseProximitySwitches(data, offset);
    // 检查是否有冲突状态
    if ((switches & 0x03) == 0x03) return 1; // 位0和位1同时为1
    if ((switches & 0x0C) == 0x0C) return 1; // 位2和位3同时为1
    return 0;
}

// 故障位1状态检查
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkFaultBits1Status(const uint8_t* data, size_t offset) {
    return parseFaultBits1(data, offset) != 0 ? 1 : 0;
}

// 故障位2状态检查
int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkFaultBits2Status(const uint8_t* data, size_t offset) {
    return parseFaultBits2(data, offset) != 0 ? 1 : 0;
}

int EMERGENCY_AUXILIARY_PROPULSION_CAN::checkRPMStatus(const uint8_t* data, size_t offset) {
    int16_t rpm = parseRPM(data, offset);
    std::string message;
    bool valid = checkRPM(rpm, message);
    
    if (!valid) {
        if (!getCurrentRPMThresholds().allow_negative && rpm < 0) return 2;
        return 1;
    }
    return 0;
}