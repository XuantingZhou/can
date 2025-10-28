/*
 * @Author: ZTX
 * @Date: 2025-06-29 13:47:32
 * @LastEditors: ZTX 2778425770@qq.com
 * @LastEditTime: 2025-06-29 14:02:54
 * @FilePath: /6.29/STEERING_SYSTEM_CAN.cpp
 * @Description: 操舵系统模块代码
 */
#pragma once
#include <cstdint>
#include <string>

class STEERING_SYSTEM_CAN {
public:
    // 阈值配置结构体
    struct TimingThresholds {
        int64_t output_no_response;    // 无输出阈值(微秒)
        int64_t output_too_fast;       // 输出过快阈值(微秒) 
        int64_t output_too_slow;       // 输出过慢阈值(微秒)
        int64_t feedback_normal;       // 反馈正常阈值(微秒)
        int64_t feedback_slow;         // 反馈较慢阈值(微秒)
        int64_t feedback_no_response;  // 无反馈阈值(微秒)
    };

    struct RPMThresholds {
        int16_t min_rpm;              // 最小转速(-100.0rpm)
        int16_t max_rpm;              // 最大转速(100.0rpm)
        bool allow_negative;          // 允许负转速
    };

    struct VoltageThresholds {
        uint16_t min_voltage;        // 最小电压(0V)
        uint16_t max_voltage;        // 最大电压(800V)
        uint16_t high_voltage_threshold; // 高压警告(750V)
    };
    
    struct CurrentThresholds {
        uint16_t max_input_current;  // 最大输入电流(600A)
        uint16_t max_motor_current;  // 最大电机电流(500A)
    };
    
    struct TemperatureThresholds {
        uint8_t max_temp;            // 最高温度(150℃)
        uint8_t warning_temp;        // 警告温度(90℃)
    };

    // 阈值设置
    static void setTimingThresholds(const TimingThresholds& thresholds);
    static void setRPMThresholds(const RPMThresholds& thresholds);
    static void setVoltageThresholds(const VoltageThresholds& thresholds);
    static void setCurrentThresholds(const CurrentThresholds& thresholds);
    static void setTemperatureThresholds(const TemperatureThresholds& thresholds);

    // 阈值获取
    static TimingThresholds getCurrentThresholds();
    static RPMThresholds getCurrentRPMThresholds();
    static VoltageThresholds getCurrentVoltageThresholds();
    static CurrentThresholds getCurrentCurrentThresholds();
    static TemperatureThresholds getCurrentTemperatureThresholds();

    // 状态检查
    static int checkOutputTiming(int64_t timeInterval);
    static int checkFeedbackTiming(int64_t timeBack);
    static int checkRPMStatus(const uint8_t* data, size_t offset);
    static int checkVoltageStatus(const uint8_t* data, size_t offset);
    static int checkCurrentStatus(const uint8_t* data, size_t offset, bool isMotorCurrent);
    static int checkTemperatureStatus(const uint8_t* data, size_t offset);

    // 数据解析
    static int16_t parseRPM(const uint8_t* data, size_t offset);
    static float parseVoltage(const uint8_t* data, size_t offset);
    static float parseCurrent(const uint8_t* data, size_t offset);
    static uint8_t parseTemperature(const uint8_t* data, size_t offset);
    static uint8_t parseCommandBits(const uint8_t* data, size_t offset);
    static uint8_t parseStatusBits(const uint8_t* data, size_t offset);
    static float parseBusVoltage(const uint8_t* data, size_t offset);

    // 温度相关状态检查
    static int checkBearingTempStatus(const uint8_t* data, size_t offset);
    static int checkWindingTempStatus(const uint8_t* data, size_t offset); 
    static int checkOilTempStatus(const uint8_t* data, size_t offset);
    static int checkInverterTempStatus(const uint8_t* data, size_t offset);

    // 压力相关状态检查  
    static int checkInletPressureStatus(const uint8_t* data, size_t offset);
    static int checkOutletPressureStatus(const uint8_t* data, size_t offset);

    // 液位相关状态检查
    static int checkCompensatorLevelStatus(const uint8_t* data, size_t offset);

    // 数据解析
    static uint8_t parseBearingTemp(const uint8_t* data, size_t offset);
    static uint8_t parseCompensatorLevel(const uint8_t* data, size_t offset);

    // 新增函数声明
    static int checkRudderAngleStatus(const uint8_t* data, size_t offset);
    static int checkRudderSpeedStatus(const uint8_t* data, size_t offset);
    static int checkTelescopicCommandStatus(const uint8_t* data, size_t offset);
    static int checkExceedAuthStatus(const uint8_t* data, size_t offset);
    static int16_t parseRudderAngle(const uint8_t* data, size_t offset);
    static float parseRudderSpeed(const uint8_t* data, size_t offset);
    static uint8_t parseTelescopicCommand(const uint8_t* data, size_t offset);
    static uint8_t parseExceedAuth(const uint8_t* data, size_t offset);
    static int checkBusVoltageStatus(const uint8_t* data, size_t offset);
    static int checkMotorCurrentStatus(const uint8_t* data, size_t offset);
    static int checkIdCurrentStatus(const uint8_t* data, size_t offset);
    static int checkIqCurrentStatus(const uint8_t* data, size_t offset);
    static float parseMotorCurrent(const uint8_t* data, size_t offset);
    static float parseIdCurrent(const uint8_t* data, size_t offset);
    static float parseIqCurrent(const uint8_t* data, size_t offset);
    static int checkPressureStatus(const uint8_t* data, size_t offset);
    static int checkIGBTTemperatureStatus(const uint8_t* data, size_t offset);
    static int checkMotorTemperatureStatus(const uint8_t* data, size_t offset);
    static float parseIGBTTemperature(const uint8_t* data, size_t offset);
    static float parseMotorTemperature(const uint8_t* data, size_t offset);
    static int checkValveStatus(const uint8_t* data, size_t offset);
    static int checkCylinderAngleStatus(const uint8_t* data, size_t offset);
    static int checkOilLevelStatus(const uint8_t* data, size_t offset);
    static uint8_t parseValveStatus(const uint8_t* data, size_t offset);
    static uint16_t parseCylinderAngle(const uint8_t* data, size_t offset);
    static uint16_t parseOilLevel(const uint8_t* data, size_t offset);
    static int checkValveCurrentStatus(const uint8_t* data, size_t offset);
    static uint16_t parseValveCurrent(const uint8_t* data, size_t offset);
    static int checkSelfcheckStatus(const uint8_t* data, size_t offset);
    static int checkModeStatus(const uint8_t* data, size_t offset);
    static uint8_t parseSelfcheckStatus(const uint8_t* data, size_t offset);
    static uint8_t parseModeStatus(const uint8_t* data, size_t offset);
    static uint8_t parsePressure(const uint8_t* data, size_t offset);

private:
    static TimingThresholds currentTimingThresholds;
    static RPMThresholds currentRPMThresholds;
    static VoltageThresholds currentVoltageThresholds;
    static CurrentThresholds currentCurrentThresholds;
    static TemperatureThresholds currentTemperatureThresholds;
};