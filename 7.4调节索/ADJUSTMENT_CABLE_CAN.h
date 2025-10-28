#pragma once
#include <cstdint>
#include <cstddef> // for size_t

class ADJUSTMENT_CABLE_CAN {
public:
    // 阈值结构体定义
    struct TimingThresholds {
        int64_t output_no_response;
        int64_t output_too_fast;
        int64_t output_too_slow;
        int64_t feedback_normal;
        int64_t feedback_slow;
        int64_t feedback_no_response;
    };

    struct ChainLengthThresholds {
        double min_length;
        double max_length;
    };

    struct SpeedThresholds {
        double max_speed;
    };

    struct PressureThresholds {
        double max_pressure;
    };

    struct ValveStatusThresholds {
        double max_valve_status;
        double min_valve_status;
    };

    // 设置阈值
    static void setTimingThresholds(const TimingThresholds& thresholds);
    static void setChainLengthThresholds(const ChainLengthThresholds& thresholds);
    static void setSpeedThresholds(const SpeedThresholds& thresholds);
    static void setPressureThresholds(const PressureThresholds& thresholds);
    static void setValveStatusThresholds(const ValveStatusThresholds& thresholds);

    // 获取阈值
    static TimingThresholds getCurrentTimingThresholds();
    static ChainLengthThresholds getCurrentChainLengthThresholds();
    static SpeedThresholds getCurrentSpeedThresholds();
    static PressureThresholds getCurrentPressureThresholds();
    static ValveStatusThresholds getCurrentValveStatusThresholds();

    // 状态检查函数
    static int checkOutputTiming(int64_t timeInterval);
    static int checkCommandStatus(const uint8_t* data, size_t offset);
    static int checkZLXStatus(const uint8_t* data, size_t offset);
    static int checkSystemStatus(const uint8_t* data, size_t offset);
    static int checkSystemCounterStatus(const uint8_t* data, size_t offset);
    static int checkChainLengthStatus(const uint8_t* data, size_t offset);
    static int checkSpeedStatus(const uint8_t* data, size_t offset);
    static int checkPressureStatus(const uint8_t* data, size_t offset);
    static int checkValveStatus(const uint8_t* data, size_t offset);
    static int checkAlarmGroup1Status(const uint8_t* data, size_t offset);
    static int checkAlarmGroup2Status(const uint8_t* data, size_t offset);
    static int checkEmergencyDropLinkStatus(const uint8_t* data, size_t offset);

    // 数据解析函数
    static int8_t parseOnoffCmd(const uint8_t* data, size_t offset);
    static int8_t parseRScable(const uint8_t* data, size_t offset);
    static double parseProportionValveStatus(const uint8_t* data, size_t offset);
    static int8_t parseOnoffCtrl1(const uint8_t* data, size_t offset);
    static int8_t parseOnoffCtrl2(const uint8_t* data, size_t offset);
    static double parseChainLength(const uint8_t* data, size_t offset);
    static double parseSpeed(const uint8_t* data, size_t offset);
    static double parsePressure(const uint8_t* data, size_t offset);
    static double parsePumpPressure(const uint8_t* data, size_t offset);
    static double parsePumpPressureA(const uint8_t* data, size_t offset);
    static double parsePumpPressureB(const uint8_t* data, size_t offset);
    static double parseValveStatus(const uint8_t* data, size_t offset);
    static int32_t parseAlarmGroup1(const uint8_t* data, size_t offset);
    static int32_t parseAlarmGroup2(const uint8_t* data, size_t offset);
    static int8_t parseIsolationValveStatus(const uint8_t* data, size_t offset);
    static double parseRpmMotor(const uint8_t* data, size_t offset);
    static int32_t parseSystemCounter(const uint8_t* data, size_t offset);
    static int32_t parseSystemStatus(const uint8_t* data, size_t offset);
    static int32_t parseZLXStatus(const uint8_t* data, size_t offset);

private:
    static TimingThresholds currentTimingThresholds;
    static ChainLengthThresholds currentChainLengthThresholds;
    static SpeedThresholds currentSpeedThresholds;
    static PressureThresholds currentPressureThresholds;
    static ValveStatusThresholds currentValveStatusThresholds;
};