#pragma once

#include <cstdint>
#include <string>
#include <array>

class EMERGENCY_AUXILIARY_PROPULSION_CAN {
public:
    // 阈值配置结构体
    struct TimingThresholds {
        int64_t output_no_response;    // 无输出阈值(微秒) - No response threshold (μs)
        int64_t output_too_fast;       // 输出过快阈值(微秒) - Output too fast threshold (μs)
        int64_t output_too_slow;       // 输出过慢阈值(微秒) - Output too slow threshold (μs)
        int64_t feedback_normal;       // 反馈正常阈值(微秒) - Normal feedback threshold (μs)
        int64_t feedback_slow;         // 反馈较慢阈值(微秒) - Slow feedback threshold (μs)
        int64_t feedback_no_response;  // 无反馈阈值(微秒) - No feedback threshold (μs)
    };

    struct RPMThresholds {
        int16_t min_rpm;              // 最小转速 - Minimum RPM value
        int16_t max_rpm;              // 最大转速 - Maximum RPM value
        bool allow_negative;          // 是否允许负转速 - Whether negative RPM is allowed
    };

    struct VoltageThresholds {
        uint16_t min_voltage;        // 最小电压值 (0V) - Minimum voltage value (0V)
        uint16_t max_voltage;        // 最大电压值 - Maximum voltage value
        uint16_t high_voltage_threshold; // 高电压阈值 - High voltage warning threshold
    };
    
    struct CurrentThresholds {
        uint16_t max_current;        // 最大电流值 (255A) - Maximum current value (255A)
    };
    
    struct TemperatureThresholds {
        uint8_t max_motor_temp;      // 电机最高温度 (150℃) - Max motor temperature (150°C)
        uint8_t max_driver_temp;     // 驱动器最高温度 (150℃) - Max driver temperature (150°C)
    };

    // 阈值设置
    static void setTimingThresholds(const TimingThresholds& thresholds);  // 设置时序阈值 - Set timing thresholds
    static void setRPMThresholds(const RPMThresholds& thresholds);        // 设置转速阈值 - Set RPM thresholds
    static void setVoltageThresholds(const VoltageThresholds& thresholds);// 设置电压阈值 - Set voltage thresholds
    static void setCurrentThresholds(const CurrentThresholds& thresholds);// 设置电流阈值 - Set current thresholds
    static void setTemperatureThresholds(const TemperatureThresholds& thresholds); // 设置温度阈值 - Set temperature thresholds

    // 阈值获取
    static TimingThresholds getCurrentThresholds();         // 获取当前时序阈值 - Get current timing thresholds
    static RPMThresholds getCurrentRPMThresholds();         // 获取当前转速阈值 - Get current RPM thresholds
    static VoltageThresholds getCurrentVoltageThresholds(); // 获取当前电压阈值 - Get current voltage thresholds
    static CurrentThresholds getCurrentCurrentThresholds(); // 获取当前电流阈值 - Get current current thresholds
    static TemperatureThresholds getCurrentTemperatureThresholds(); // 获取当前温度阈值 - Get current temperature thresholds

    // 状态检查
    static int checkOutputTiming(int64_t timeInterval);     // 检查输出时序 - Check output timing
    static int checkFeedbackTiming(int64_t timeBack);       // 检查反馈时序 - Check feedback timing
    static bool checkRPM(int16_t rpm, std::string& message);// 检查转速状态 - Check RPM status
    static int checkRPMStatus(const uint8_t* data, size_t offset); // 检查转速状态(基于原始数据) - Check RPM status (raw data)
    
    // 数据解析
    static int16_t parseRPM(const uint8_t* data, size_t offset);           // 解析转速数据 - Parse RPM data
    static uint16_t parseVoltage(const uint8_t data[], size_t offset);     // 解析电压数据 - Parse voltage data
    static uint8_t parseCurrent(const uint8_t data[], size_t offset);      // 解析电流数据 - Parse current data
    static uint8_t parseMotorTemperature(const uint8_t data[], size_t offset); // 解析电机温度 - Parse motor temperature
    static uint8_t parseDriverTemperature(const uint8_t data[], size_t offset); // 解析驱动器温度 - Parse driver temperature
    
    // 状态检查
    static int checkVoltageStatus(const uint8_t data[], size_t offset);    // 检查电压状态 - Check voltage status
    static int checkCurrentStatus(const uint8_t data[], size_t offset);    // 检查电流状态 - Check current status
    static int checkMotorTempStatus(const uint8_t data[],size_t motor_temp_offset); // 检查电机温度状态 - Check motor temp status
    static int checkDriverTempStatus(const uint8_t data[],size_t driver_temp_offset); // 检查驱动器温度状态 - Check driver temp status

    //320223 状态反馈帧处理
    static uint8_t parseCompensatorLevel(const uint8_t* data, size_t offset); // 解析补偿器液位 - Parse compensator level
    static uint8_t parseExtensionStatus(const uint8_t* data, size_t offset);  // 解析伸缩机构状态 - Parse extension status
    static uint8_t parseProximitySwitches(const uint8_t* data, size_t offset); // 解析接近开关状态 - Parse proximity switches
    static uint8_t parseFaultBits1(const uint8_t* data, size_t offset);       // 解析故障位1 - Parse fault bits 1
    static uint8_t parseFaultBits2(const uint8_t* data, size_t offset);       // 解析故障位2 - Parse fault bits 2

    // 状态检查
    static int checkCompensatorLevelStatus(const uint8_t* data, size_t offset); // 检查补偿器液位状态 - Check compensator level
    static int checkExtensionStatus(const uint8_t* data, size_t offset);        // 检查伸缩机构状态 - Check extension status
    static int checkProximitySwitchesStatus(const uint8_t* data, size_t offset); // 检查接近开关状态 - Check proximity switches
    static int checkFaultBits1Status(const uint8_t* data, size_t offset);       // 检查故障位1状态 - Check fault bits 1
    static int checkFaultBits2Status(const uint8_t* data, size_t offset);       // 检查故障位2状态 - Check fault bits 2

private:
    static TimingThresholds currentTimingThresholds;      // 当前时序阈值 - Current timing thresholds
    static RPMThresholds currentRPMThresholds;            // 当前转速阈值 - Current RPM thresholds
    static VoltageThresholds currentVoltageThresholds;    // 当前电压阈值 - Current voltage thresholds
    static CurrentThresholds currentCurrentThresholds;    // 当前电流阈值 - Current current thresholds
    static TemperatureThresholds currentTemperatureThresholds; // 当前温度阈值 - Current temperature thresholds
};