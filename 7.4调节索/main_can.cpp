#include "main_can.h"

// 常量定义
const std::string INTERFACE_NAMES[CAN_SIZE] = {"can0", "can1", "can2"};

// 索引数组
int write_indices[CAN_SIZE] = {0};
int read_indices[CAN_SIZE] = {0};
int local_write_indices[CAN_SIZE] = {0};



// 缓冲区
std::array<std::array<std::array<unsigned char, FRAME_SIZE>, BUFFER_SIZE>, CAN_SIZE> g_packet_buffer;
std::array<std::array<std::array<unsigned char, RESULT_FRAME_SIZE>, RESULT_BUFFER_SIZE>, CAN_SIZE> Result_Buffer;

// 同步工具
std::mutex buffer_mutex;
std::condition_variable data_condition;
std::mutex result_mutex;
bool running = true;

// 时间戳记录
std::map<size_t, std::map<uint32_t, int64_t>> last_timestamp_map;

int64_t calcFrameInterval(size_t channel, uint32_t frame_id, int64_t current_timestamp) {
    int64_t interval = -1;
    if (last_timestamp_map[channel].count(frame_id)) {
        interval = (current_timestamp - last_timestamp_map[channel][frame_id]) / 1000;
    }
    last_timestamp_map[channel][frame_id] = current_timestamp;
    return interval;
}


// 时间戳数组 - 改为CAN_SIZE
std::array<int64_t, CAN_SIZE> current_Timestamp = {};
std::array<int64_t, CAN_SIZE> last_Timestamp = {};
std::array<int64_t, CAN_SIZE> last_Timestamp_1 = {};
std::array<int64_t, CAN_SIZE> last_Timestamp_2 = {};
std::array<int64_t, CAN_SIZE> last_Timestamp_3 = {};
std::array<int64_t, CAN_SIZE> remote_Timestamp1 = {};
std::array<int64_t, CAN_SIZE> back_Timestamp1 = {};

// ID数组
std::array<int32_t, CAN_SIZE> ID = {};

// 控制变量数组
int pack_control[24][4] = {};

// 时间戳数组 - 操舵系统专用
std::array<int64_t, 12> last_Timestamp_Steering = {};  // 假设最多12个操舵相关ID


void parseData() { 
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bool hasNewData = false;
        
        while (!hasNewData) {
            for (size_t i = 0; i < CAN_SIZE; ++i) {
                if (write_indices[i] != read_indices[i]) {
                    hasNewData = true;
                    break;
                }
            }
        }

        std::unique_lock<std::mutex> lock(buffer_mutex);
        
        if (hasNewData) {
            std::array<std::array<std::array<unsigned char, FRAME_SIZE>, BUFFER_SIZE>, CAN_SIZE> g1_packet_buffer;
            for (size_t i = 0; i < CAN_SIZE; ++i) {
                std::copy(std::begin(g_packet_buffer[i]), std::end(g_packet_buffer[i]), std::begin(g1_packet_buffer[i]));  
            }

            std::copy(std::begin(write_indices), std::end(write_indices), std::begin(local_write_indices));

            lock.unlock();

            for (size_t i = 0; i < CAN_SIZE; ++i) {
                while (local_write_indices[i] != read_indices[i]) {
                    int packetIndex = read_indices[i];

                    std::memcpy(&ID[i], &g1_packet_buffer[i][packetIndex][21], 4);
                    std::memcpy(&current_Timestamp[i], &g1_packet_buffer[i][packetIndex][2], sizeof(current_Timestamp[i]));

                    switch (i) {
                        case 1: { // 主推进器
                            static int64_t last_main_ts[CAN_SIZE][18] = {0}; 
                            static int main_pack_ctrl[CAN_SIZE][18] = {0};  
                            
                            switch (ID[i]) {
                                case 0x32111: { 
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][0]) / 1000; 
                                    last_main_ts[i][0] = current_Timestamp[i]; 
                                    main_pack_ctrl[i][0] = packetIndex;
                                    Result_Buffer[i][packetIndex][6] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][8] = MAIN_PROPULSION_CAN::checkRPMStatus(g1_packet_buffer[i][packetIndex].data(), 30);
                                    int16_t set_rpm = MAIN_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][packetIndex].data(), 30); 
                                    std::cout << "Main CMD RPM:" << (set_rpm/10.0) << "rpm-32111 || "; 
                                    break;
                                }
                                case 0x321121: { 
                                    int64_t fb_delay = (current_Timestamp[i] - last_main_ts[i][0]) / 1000;
                                    Result_Buffer[i][packetIndex][7] = MAIN_PROPULSION_CAN::checkFeedbackTiming(fb_delay);
                                    if (main_pack_ctrl[i][0] >= 0) { 
                                        int16_t cmd_rpm = MAIN_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][main_pack_ctrl[i][0]].data(), 30); 
                                        int16_t fb_rpm = MAIN_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][packetIndex].data(), 30); 
                                        Result_Buffer[i][packetIndex][9] = (cmd_rpm == fb_rpm) ? 0 : 1; 
                                    }
                                    std::cout << "Main FB Delay:" << fb_delay << "ms-321121 || "; 
                                    break;
                                }
                                case 0x321122: { 
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][1]) / 1000; 
                                    last_main_ts[i][1] = current_Timestamp[i]; 
                                    main_pack_ctrl[i][1] = packetIndex;
                                    Result_Buffer[i][packetIndex][10] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][11] = MAIN_PROPULSION_CAN::checkRPMStatus(g1_packet_buffer[i][packetIndex].data(), 28);
                                    Result_Buffer[i][packetIndex][12] = MAIN_PROPULSION_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 30);
                                    Result_Buffer[i][packetIndex][13] = MAIN_PROPULSION_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 34);
                                    int16_t rpm = MAIN_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][packetIndex].data(), 28); 
                                    std::cout << "Branch1 RPM:" << (rpm/10.0) << "rpm-321122 || "; 
                                    break;
                                }
                                case 0x321123: { 
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][2]) / 1000; 
                                    last_main_ts[i][2] = current_Timestamp[i];
                                    Result_Buffer[i][packetIndex][14] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][15] = MAIN_PROPULSION_CAN::checkCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 26, false);
                                    Result_Buffer[i][packetIndex][16] = MAIN_PROPULSION_CAN::checkCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 30, true);
                                    std::cout << "Branch1 Current-321123 || "; 
                                    break;
                                }
                                case 0x321124: { 
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][3]) / 1000; 
                                    last_main_ts[i][3] = current_Timestamp[i]; 
                                    main_pack_ctrl[i][3] = packetIndex;
                                    Result_Buffer[i][packetIndex][17] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][18] = MAIN_PROPULSION_CAN::checkRPMStatus(g1_packet_buffer[i][packetIndex].data(), 28);
                                    Result_Buffer[i][packetIndex][19] = MAIN_PROPULSION_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 30);
                                    std::cout << "Branch2 RPM:" << (MAIN_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][packetIndex].data(), 28)/10.0 ) << "rpm-321124 ||"; 
                                    break;
                                }
                                case 0x321126: { // 轴承温度帧1
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][5]) / 1000; 
                                    last_main_ts[i][5] = current_Timestamp[i];
                                    
                                    Result_Buffer[i][packetIndex][22] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][30] = MAIN_PROPULSION_CAN::checkBearingTempStatus(g1_packet_buffer[i][packetIndex].data(), 30);
                                    Result_Buffer[i][packetIndex][31] = MAIN_PROPULSION_CAN::checkBearingTempStatus(g1_packet_buffer[i][packetIndex].data(), 31);
                                    Result_Buffer[i][packetIndex][32] = MAIN_PROPULSION_CAN::checkBearingTempStatus(g1_packet_buffer[i][packetIndex].data(), 32);
                                    Result_Buffer[i][packetIndex][33] = MAIN_PROPULSION_CAN::checkBearingTempStatus(g1_packet_buffer[i][packetIndex].data(), 33);
                                    
                                    std::cout << "Bearing1 Temp:" 
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 30) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 31) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 32) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 33) 
                                              << "℃-321126 || ";
                                    break;
                                }
                                
                                case 0x321127: { // 轴承温度帧2 + 绕组温度
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][6]) / 1000;
                                    last_main_ts[i][6] = current_Timestamp[i];
                                    
                                    Result_Buffer[i][packetIndex][23] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][34] = MAIN_PROPULSION_CAN::checkBearingTempStatus(g1_packet_buffer[i][packetIndex].data(), 30);
                                    Result_Buffer[i][packetIndex][35] = MAIN_PROPULSION_CAN::checkBearingTempStatus(g1_packet_buffer[i][packetIndex].data(), 31);
                                    Result_Buffer[i][packetIndex][36] = MAIN_PROPULSION_CAN::checkWindingTempStatus(g1_packet_buffer[i][packetIndex].data(), 32);
                                    Result_Buffer[i][packetIndex][37] = MAIN_PROPULSION_CAN::checkWindingTempStatus(g1_packet_buffer[i][packetIndex].data(), 33);
                                    
                                    std::cout << "Bearing2/Winding Temp:" 
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 30) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 31) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 32) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 33)
                                              << "℃-321127 || ";
                                    break;
                                }
                                
                                case 0x321128: { // 变频器温度
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][7]) / 1000;
                                    last_main_ts[i][7] = current_Timestamp[i];
                                    
                                    Result_Buffer[i][packetIndex][24] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][38] = MAIN_PROPULSION_CAN::checkInverterTempStatus(g1_packet_buffer[i][packetIndex].data(), 34);
                                    Result_Buffer[i][packetIndex][39] = MAIN_PROPULSION_CAN::checkInverterTempStatus(g1_packet_buffer[i][packetIndex].data(), 35);
                                    Result_Buffer[i][packetIndex][40] = MAIN_PROPULSION_CAN::checkInverterTempStatus(g1_packet_buffer[i][packetIndex].data(), 36);
                                    Result_Buffer[i][packetIndex][41] = MAIN_PROPULSION_CAN::checkInverterTempStatus(g1_packet_buffer[i][packetIndex].data(), 37);
                                    
                                    std::cout << "Inverter Temp:" 
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 34) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 35) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 36) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseTemperature(g1_packet_buffer[i][packetIndex].data(), 37)
                                              << "℃-321128 || ";
                                    break;
                                }
                                
                                case 0x321129: { // 补偿器液位
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][8]) / 1000;
                                    last_main_ts[i][8] = current_Timestamp[i];
                                    
                                    Result_Buffer[i][packetIndex][25] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][42] = MAIN_PROPULSION_CAN::checkCompensatorLevelStatus(g1_packet_buffer[i][packetIndex].data(), 34);
                                    Result_Buffer[i][packetIndex][43] = MAIN_PROPULSION_CAN::checkCompensatorLevelStatus(g1_packet_buffer[i][packetIndex].data(), 35);
                                    Result_Buffer[i][packetIndex][44] = MAIN_PROPULSION_CAN::checkCompensatorLevelStatus(g1_packet_buffer[i][packetIndex].data(), 36);
                                    Result_Buffer[i][packetIndex][45] = MAIN_PROPULSION_CAN::checkCompensatorLevelStatus(g1_packet_buffer[i][packetIndex].data(), 37);
                                    
                                    std::cout << "Compensator Level:" 
                                              << (int)MAIN_PROPULSION_CAN::parseCompensatorLevel(g1_packet_buffer[i][packetIndex].data(), 34) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseCompensatorLevel(g1_packet_buffer[i][packetIndex].data(), 35) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseCompensatorLevel(g1_packet_buffer[i][packetIndex].data(), 36) << "/"
                                              << (int)MAIN_PROPULSION_CAN::parseCompensatorLevel(g1_packet_buffer[i][packetIndex].data(), 37)
                                              << "%-321129 || ";
                                    break;
                                }
                                
                                case 0x321130: { // 压力检测
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][9]) / 1000;
                                    last_main_ts[i][9] = current_Timestamp[i];
                                    
                                    Result_Buffer[i][packetIndex][26] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][46] = MAIN_PROPULSION_CAN::checkInletPressureStatus(g1_packet_buffer[i][packetIndex].data(), 30);
                                    Result_Buffer[i][packetIndex][47] = MAIN_PROPULSION_CAN::checkOutletPressureStatus(g1_packet_buffer[i][packetIndex].data(), 34);
                                    
                                    std::cout << "Pressure In/Out:" 
                                              << MAIN_PROPULSION_CAN::parsePressure(g1_packet_buffer[i][packetIndex].data(), 30) << "/"
                                              << MAIN_PROPULSION_CAN::parsePressure(g1_packet_buffer[i][packetIndex].data(), 34)
                                              << "kPa-321130 || ";
                                    break;
                                }
                                
                                case 0x321131: case 0x321132: case 0x321133: { 
                                    int frameType = ID[i] - 0x321130; 
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][frameType + 8]) / 1000; 
                                    last_main_ts[i][frameType + 8] = current_Timestamp[i];
                                    Result_Buffer[i][packetIndex][26 + frameType - 1] = MAIN_PROPULSION_CAN::checkOutputTiming(interval);
                                    std::cout << "Fault Frame" << frameType << "-" << std::hex << ID[i] << " || "; 
                                    break;
                                }
                                default: break;
                            }
                            break;
                        }
                        case 2: {       // 应急推进器
                            switch (ID[i]) {
                                case 0x32021: {
                                    std::memcpy(&last_Timestamp[i], &g1_packet_buffer[i][pack_control[i][0]][2], sizeof(last_Timestamp[i]));
                                    int64_t timeInterval = (current_Timestamp[i] - last_Timestamp[i]) / 1000;                                   
                                    std::memcpy(&remote_Timestamp1[i], &g1_packet_buffer[i][packetIndex][2], sizeof(current_Timestamp[i]));
                                    pack_control[i][0] = packetIndex;
                                    Result_Buffer[i][packetIndex][6] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkOutputTiming(timeInterval);
                                    Result_Buffer[i][packetIndex][8] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkRPMStatus(g1_packet_buffer[i][packetIndex].data(), 26);
                                    int16_t actual_rpm = EMERGENCY_AUXILIARY_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][packetIndex].data(), 26);
                                    std::cout << actual_rpm << "rpm-32021 || ";
                                    break;
                                }
                                case 0x320221: {
                                    std::memcpy(&back_Timestamp1[i], &g1_packet_buffer[i][packetIndex][2], sizeof(current_Timestamp[i]));
                                    int64_t timeBack1 = (back_Timestamp1[i] - remote_Timestamp1[i]) / 1000;
                                    std::cout << "320221     "; 
                                    std::cout << "反馈时间=" << std::dec << timeBack1 << "微秒" << "  || ";
                                    Result_Buffer[i][packetIndex][7] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkFeedbackTiming(timeBack1);
                                    break;
                                }
                                case 0x320222: {
                                    std::cout << "320222     ";
                                    std::memcpy(&last_Timestamp_1[i], &g1_packet_buffer[i][pack_control[i][1]][2], sizeof(last_Timestamp[i]));
                                    int64_t timeInterval_1 = (current_Timestamp[i] - last_Timestamp_1[i]) / 1000;
                                    std::cout << "时间间隔=" << std::dec << timeInterval_1 << "微秒" << "   ||";
                                    pack_control[i][1] = packetIndex;
                                    Result_Buffer[i][packetIndex][9] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkOutputTiming(timeInterval_1);
                                    Result_Buffer[i][packetIndex][8] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkRPMStatus(g1_packet_buffer[i][packetIndex].data(), 27);
                                    int16_t actual_rpm_1 = EMERGENCY_AUXILIARY_PROPULSION_CAN::parseRPM(g1_packet_buffer[i][packetIndex].data(), 27);
                                    std::cout << actual_rpm_1 << "rpm" << std::endl;
                                    
                                    Result_Buffer[i][packetIndex][14] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 30) != 0;
                                    Result_Buffer[i][packetIndex][12] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 32);
                                    Result_Buffer[i][packetIndex][15] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkMotorTempStatus(g1_packet_buffer[i][packetIndex].data(), 33);
                                    Result_Buffer[i][packetIndex][16] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkDriverTempStatus(g1_packet_buffer[i][packetIndex].data(), 34);
                                    break;
                                }
                                case 0x320223: {
                                    std::cout << "320223     ";
                                    std::memcpy(&last_Timestamp_2[i], &g1_packet_buffer[i][pack_control[i][2]][2], sizeof(last_Timestamp[i]));
                                    int64_t timeInterval_2 = (current_Timestamp[i] - last_Timestamp_2[i]) / 1000;
                                    std::cout << "时间间隔=" << std::dec << timeInterval_2 << "微秒" << "   ||";
                                    pack_control[i][2] = packetIndex;
                                    Result_Buffer[i][packetIndex][10] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkOutputTiming(timeInterval_2);
                                    Result_Buffer[i][packetIndex][17] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkCompensatorLevelStatus(g1_packet_buffer[i][packetIndex].data(), 26); 
                                    Result_Buffer[i][packetIndex][18] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkFaultBits1Status(g1_packet_buffer[i][packetIndex].data(), 26); 
                                    Result_Buffer[i][packetIndex][19] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkFaultBits2Status(g1_packet_buffer[i][packetIndex].data(), 26); 
                                    Result_Buffer[i][packetIndex][20] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkExtensionStatus(g1_packet_buffer[i][packetIndex].data(), 26); 
                                    Result_Buffer[i][packetIndex][21] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkProximitySwitchesStatus(g1_packet_buffer[i][packetIndex].data(), 26); 
                                    break;
                                }
                                case 0x320224: {
                                    std::cout << "320224     ";
                                    std::memcpy(&last_Timestamp_3[i], &g1_packet_buffer[i][pack_control[i][3]][2], sizeof(last_Timestamp[i]));
                                    int64_t timeInterval_3 = (current_Timestamp[i] - last_Timestamp_3[i]) / 1000;
                                    std::cout << "时间间隔=" << std::dec << timeInterval_3 << "微秒" << "   ||";
                                    pack_control[i][3] = packetIndex;
                                    Result_Buffer[i][packetIndex][11] = EMERGENCY_AUXILIARY_PROPULSION_CAN::checkOutputTiming(timeInterval_3); 
                                    break;
                                }
                                default:
                                    break;
                            }
                            break;
                        }
                        case 3:
                            break;

                        case 9:
                            switch (ID[i]) {

                                    static int64_t last_main_ts[CAN_SIZE][17] = {0};  
                                    static int main_pack_ctrl[CAN_SIZE][17] = {0};   

                                case 0x33411: { // 指令帧
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][9]) / 1000;
                                    last_main_ts[i][9] = current_Timestamp[i];
                                    main_pack_ctrl[i][9] = packetIndex;

                                    Result_Buffer[i][packetIndex][27] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][28] = STEERING_SYSTEM_CAN::checkRudderAngleStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][29] = STEERING_SYSTEM_CAN::checkRudderSpeedStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][30] = STEERING_SYSTEM_CAN::checkTelescopicCommandStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                    Result_Buffer[i][packetIndex][31] = STEERING_SYSTEM_CAN::checkExceedAuthStatus(g1_packet_buffer[i][packetIndex].data(), 5);

                                    float rudder_angle = STEERING_SYSTEM_CAN::parseRudderAngle(g1_packet_buffer[i][packetIndex].data(), 0);
                                    float rudder_speed = STEERING_SYSTEM_CAN::parseRudderSpeed(g1_packet_buffer[i][packetIndex].data(), 2);
                                    uint8_t telescopic_command = STEERING_SYSTEM_CAN::parseTelescopicCommand(g1_packet_buffer[i][packetIndex].data(), 4);
                                    uint8_t exceed_auth = STEERING_SYSTEM_CAN::parseExceedAuth(g1_packet_buffer[i][packetIndex].data(), 5);

                                    std::cout << "Rudder CMD Angle:" << (rudder_angle / 10.0) << "deg-33411 || ";
                                    std::cout << "Rudder CMD Speed:" << (rudder_speed / 10.0) << "deg/s-33411 || ";
                                    std::cout << "Telescopic CMD:" << static_cast<int>(telescopic_command) << "-33411 || ";
                                    std::cout << "Exceed Auth:" << static_cast<int>(exceed_auth) << "-33411 || ";
                                    break;
                                }

                                case 0x334121: { // 反馈帧
                                    int64_t fb_delay = (current_Timestamp[i] - last_main_ts[i][10]) / 1000;
                                    Result_Buffer[i][packetIndex][32] = STEERING_SYSTEM_CAN::checkFeedbackTiming(fb_delay);
                                    if (main_pack_ctrl[i][9] >= 0) {
                                        int16_t cmd_rudder_angle = STEERING_SYSTEM_CAN::parseRudderAngle(g1_packet_buffer[i][main_pack_ctrl[i][9]].data(), 0);
                                        int16_t fb_rudder_angle = STEERING_SYSTEM_CAN::parseRudderAngle(g1_packet_buffer[i][packetIndex].data(), 0);
                                        Result_Buffer[i][packetIndex][33] = (cmd_rudder_angle == fb_rudder_angle) ? 0 : 1;
                                    }
                                        int16_t cmd_rudder_speed = STEERING_SYSTEM_CAN::parseRudderSpeed(g1_packet_buffer[i][main_pack_ctrl[i][9]].data(), 2);
                                        uint8_t cmd_telescopic_command = STEERING_SYSTEM_CAN::parseTelescopicCommand(g1_packet_buffer[i][main_pack_ctrl[i][9]].data(), 4);
                                        uint8_t cmd_exceed_auth = STEERING_SYSTEM_CAN::parseExceedAuth(g1_packet_buffer[i][main_pack_ctrl[i][9]].data(), 5);

                                        Result_Buffer[i][packetIndex][34] = STEERING_SYSTEM_CAN::checkRudderSpeedStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                        Result_Buffer[i][packetIndex][35] = STEERING_SYSTEM_CAN::checkTelescopicCommandStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                        Result_Buffer[i][packetIndex][36] = STEERING_SYSTEM_CAN::checkExceedAuthStatus(g1_packet_buffer[i][packetIndex].data(), 5);

                                        std::cout << "Rudder FB Delay:" << fb_delay << "ms-334121 || ";
                                        std::cout << "Rudder CMD Speed:" << (cmd_rudder_speed / 10.0) << "deg/s-334121 || ";
                                        std::cout << "Telescopic CMD:" << static_cast<int>(cmd_telescopic_command) << "-334121 || ";
                                        std::cout << "Exceed Auth:" << static_cast<int>(cmd_exceed_auth) << "-334121 || ";
                                        break;
                                    }

                                case 0x334122: { // 状态反馈帧1
                                        int64_t interval = (current_Timestamp[i] - last_main_ts[i][11]) / 1000;
                                        last_main_ts[i][11] = current_Timestamp[i];
                                        main_pack_ctrl[i][10] = packetIndex;

                                        Result_Buffer[i][packetIndex][34] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                        Result_Buffer[i][packetIndex][35] = STEERING_SYSTEM_CAN::checkRudderAngleStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                        Result_Buffer[i][packetIndex][36] = STEERING_SYSTEM_CAN::checkRudderSpeedStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                        Result_Buffer[i][packetIndex][37] = STEERING_SYSTEM_CAN::checkBusVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 6);

                                        uint8_t selfcheck_status = STEERING_SYSTEM_CAN::parseSelfcheckStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                        uint8_t mode_status = STEERING_SYSTEM_CAN::parseModeStatus(g1_packet_buffer[i][packetIndex].data(), 1);

                                        Result_Buffer[i][packetIndex][38] = STEERING_SYSTEM_CAN::checkSelfcheckStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                        Result_Buffer[i][packetIndex][39] = STEERING_SYSTEM_CAN::checkModeStatus(g1_packet_buffer[i][packetIndex].data(), 1);

                                        float rudder_angle = STEERING_SYSTEM_CAN::parseRudderAngle(g1_packet_buffer[i][packetIndex].data(), 2);
                                        float rudder_speed = STEERING_SYSTEM_CAN::parseRudderSpeed(g1_packet_buffer[i][packetIndex].data(), 4);
                                        float bus_voltage = STEERING_SYSTEM_CAN::parseBusVoltage(g1_packet_buffer[i][packetIndex].data(), 6);

                                        std::cout << "Selfcheck Status:" << static_cast<int>(selfcheck_status) << "-334122 || ";
                                        std::cout << "Mode Status:" << static_cast<int>(mode_status) << "-334122 || ";
                                        std::cout << "Rudder Angle:" << (rudder_angle / 10.0) << "deg-334122 || ";
                                        std::cout << "Rudder Speed:" << (rudder_speed / 10.0) << "deg/s-334122 || ";
                                        std::cout << "Bus Voltage:" << (bus_voltage / 10.0) << "V-334122 || ";
                                        break;
                                    }

                                case 0x334123: { // 状态反馈帧2
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][12]) / 1000;
                                    last_main_ts[i][12] = current_Timestamp[i];
                                    main_pack_ctrl[i][11] = packetIndex;

                                    Result_Buffer[i][packetIndex][38] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][39] = STEERING_SYSTEM_CAN::checkMotorCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][40] = STEERING_SYSTEM_CAN::checkMotorCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][41] = STEERING_SYSTEM_CAN::checkIdCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                    Result_Buffer[i][packetIndex][42] = STEERING_SYSTEM_CAN::checkIqCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 6);

                                    float motor_u_current = STEERING_SYSTEM_CAN::parseMotorCurrent(g1_packet_buffer[i][packetIndex].data(), 0);
                                    float motor_v_current = STEERING_SYSTEM_CAN::parseMotorCurrent(g1_packet_buffer[i][packetIndex].data(), 2);
                                    float id_current = STEERING_SYSTEM_CAN::parseIdCurrent(g1_packet_buffer[i][packetIndex].data(), 4);
                                    float iq_current = STEERING_SYSTEM_CAN::parseIqCurrent(g1_packet_buffer[i][packetIndex].data(), 6);

                                    std::cout << "Motor U Current:" << (motor_u_current / 100.0) << "A-334123 || ";
                                    std::cout << "Motor V Current:" << (motor_v_current / 100.0) << "A-334123 || ";
                                    std::cout << "Id Current:" << (id_current / 100.0) << "A-334123 || ";
                                    std::cout << "Iq Current:" << (iq_current / 100.0) << "A-334123 || ";
                                    break;
                                }

                                case 0x334124: { // 状态反馈帧3
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][13]) / 1000;
                                    last_main_ts[i][13] = current_Timestamp[i];
                                    main_pack_ctrl[i][12] = packetIndex;

                                    Result_Buffer[i][packetIndex][43] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][44] = STEERING_SYSTEM_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][45] = STEERING_SYSTEM_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 1);
                                    Result_Buffer[i][packetIndex][46] = STEERING_SYSTEM_CAN::checkVoltageStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][47] = STEERING_SYSTEM_CAN::checkPressureStatus(g1_packet_buffer[i][packetIndex].data(), 3);

                                    float voltage_24v = STEERING_SYSTEM_CAN::parseVoltage(g1_packet_buffer[i][packetIndex].data(), 0);
                                    float voltage_12v = STEERING_SYSTEM_CAN::parseVoltage(g1_packet_buffer[i][packetIndex].data(), 1);
                                    float voltage_5v = STEERING_SYSTEM_CAN::parseVoltage(g1_packet_buffer[i][packetIndex].data(), 2);
                                    uint8_t pressure_value = STEERING_SYSTEM_CAN::parsePressure(g1_packet_buffer[i][packetIndex].data(), 3);

                                    std::cout << "Voltage 24V:" << static_cast<int>(voltage_24v) << "V-334124 || ";
                                    std::cout << "Voltage 12V:" << static_cast<int>(voltage_12v) << "V-334124 || ";
                                    std::cout << "Voltage 5V:" << static_cast<int>(voltage_5v) << "V-334124 || ";
                                    std::cout << "Pressure Value:" << static_cast<int>(pressure_value) << "MPa-334124 || ";
                                    break;
                                }

                                case 0x334125: { // 状态反馈帧4
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][14]) / 1000;
                                    last_main_ts[i][14] = current_Timestamp[i];
                                    main_pack_ctrl[i][13] = packetIndex;

                                    Result_Buffer[i][packetIndex][48] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][49] = STEERING_SYSTEM_CAN::checkIGBTTemperatureStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                    Result_Buffer[i][packetIndex][50] = STEERING_SYSTEM_CAN::checkMotorTemperatureStatus(g1_packet_buffer[i][packetIndex].data(), 5);

                                    float igbt_temperature = STEERING_SYSTEM_CAN::parseIGBTTemperature(g1_packet_buffer[i][packetIndex].data(), 4);
                                    float motor_temperature = STEERING_SYSTEM_CAN::parseMotorTemperature(g1_packet_buffer[i][packetIndex].data(), 5);

                                    std::cout << "IGBT Temperature:" << static_cast<int>(igbt_temperature) << "℃-334125 || ";
                                    std::cout << "Motor Temperature:" << static_cast<int>(motor_temperature) << "℃-334125 || ";
                                    break;
                                }

                                case 0x334126: { // 状态反馈帧5
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][15]) / 1000;
                                    last_main_ts[i][15] = current_Timestamp[i];
                                    main_pack_ctrl[i][14] = packetIndex;

                                    Result_Buffer[i][packetIndex][51] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][52] = STEERING_SYSTEM_CAN::checkValveStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][53] = STEERING_SYSTEM_CAN::checkCylinderAngleStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][54] = STEERING_SYSTEM_CAN::checkOilLevelStatus(g1_packet_buffer[i][packetIndex].data(), 4);

                                    uint8_t valve_status = STEERING_SYSTEM_CAN::parseValveStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    uint16_t cylinder_angle = STEERING_SYSTEM_CAN::parseCylinderAngle(g1_packet_buffer[i][packetIndex].data(), 2);
                                    uint16_t oil_level = STEERING_SYSTEM_CAN::parseOilLevel(g1_packet_buffer[i][packetIndex].data(), 4);

                                    std::cout << "Valve Status:" << static_cast<int>(valve_status) << "-334126 || ";
                                    std::cout << "Cylinder Angle:" << (cylinder_angle / 10.0) << "°-334126 || ";
                                    std::cout << "Oil Level:" << (oil_level / 10.0) << "mm-334126 || ";
                                    break;
                                }

                                case 0x334127: { // 状态反馈帧6
                                    int64_t interval = (current_Timestamp[i] - last_main_ts[i][16]) / 1000;
                                    last_main_ts[i][16] = current_Timestamp[i];
                                    main_pack_ctrl[i][15] = packetIndex;

                                    Result_Buffer[i][packetIndex][55] = STEERING_SYSTEM_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][56] = STEERING_SYSTEM_CAN::checkValveCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][57] = STEERING_SYSTEM_CAN::checkValveCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][58] = STEERING_SYSTEM_CAN::checkValveCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                    Result_Buffer[i][packetIndex][59] = STEERING_SYSTEM_CAN::checkValveCurrentStatus(g1_packet_buffer[i][packetIndex].data(), 6);

                                    uint16_t valve1_current = STEERING_SYSTEM_CAN::parseValveCurrent(g1_packet_buffer[i][packetIndex].data(), 0);
                                    uint16_t valve2_current = STEERING_SYSTEM_CAN::parseValveCurrent(g1_packet_buffer[i][packetIndex].data(), 2);
                                    uint16_t valve3_current = STEERING_SYSTEM_CAN::parseValveCurrent(g1_packet_buffer[i][packetIndex].data(), 4);
                                    uint16_t valve4_current = STEERING_SYSTEM_CAN::parseValveCurrent(g1_packet_buffer[i][packetIndex].data(), 6);

                                    std::cout << "Valve1 Current:" << (valve1_current / 100.0) << "A-334127 || ";
                                    std::cout << "Valve2 Current:" << (valve2_current / 100.0) << "A-334127 || ";
                                    std::cout << "Valve3 Current:" << (valve3_current / 100.0) << "A-334127 || ";
                                    std::cout << "Valve4 Current:" << (valve4_current / 100.0) << "A-334127 || ";
                                    break;
                                }

                        case 15: { // 调节索装置
                                static int64_t last_adj_ts[CAN_SIZE][8] = {0};
                                static int adj_pack_ctrl[CAN_SIZE][8] = {0};

                            switch (ID[i]) {
                                case 0x345101: { // 上位机 → 驱动器，100ms周期指令帧
                                    int64_t interval = (current_Timestamp[i] - last_adj_ts[i][0]) / 1000;
                                    last_adj_ts[i][0] = current_Timestamp[i];
                                    adj_pack_ctrl[i][0] = packetIndex;
                                    Result_Buffer[i][packetIndex][0] = ADJUSTMENT_CABLE_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][1] = ADJUSTMENT_CABLE_CAN::checkCommandStatus(g1_packet_buffer[i][packetIndex].data(), 0); // 启停
                                    Result_Buffer[i][packetIndex][2] = ADJUSTMENT_CABLE_CAN::parseOnoffCmd(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][3] = ADJUSTMENT_CABLE_CAN::parseRScable(g1_packet_buffer[i][packetIndex].data(), 1);
                                    Result_Buffer[i][packetIndex][4] = ADJUSTMENT_CABLE_CAN::parseProportionValveStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][5] = ADJUSTMENT_CABLE_CAN::parseOnoffCtrl1(g1_packet_buffer[i][packetIndex].data(), 3);
                                    Result_Buffer[i][packetIndex][6] = ADJUSTMENT_CABLE_CAN::parseOnoffCtrl2(g1_packet_buffer[i][packetIndex].data(), 4);
                                    std::cout << "345101 指令帧：启停=" << +Result_Buffer[i][packetIndex][2]
                                            << ", 收放链=" << +Result_Buffer[i][packetIndex][3]
                                            << ", 比例阀设定=" << +Result_Buffer[i][packetIndex][4]
                                            << ", 弃链1=" << +Result_Buffer[i][packetIndex][5]
                                            << ", 弃链2=" << +Result_Buffer[i][packetIndex][6] << " || ";
                                    break;
                                }
                                case 0x345201: { // 驱动器 → 上位机，立即反馈帧
                                    int64_t interval = (current_Timestamp[i] - last_adj_ts[i][1]) / 1000;
                                    last_adj_ts[i][1] = current_Timestamp[i];
                                    adj_pack_ctrl[i][1] = packetIndex;
                                    Result_Buffer[i][packetIndex][7] = ADJUSTMENT_CABLE_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][8] = ADJUSTMENT_CABLE_CAN::checkCommandStatus(g1_packet_buffer[i][packetIndex].data(), 0); // 启停反馈
                                    Result_Buffer[i][packetIndex][9] = ADJUSTMENT_CABLE_CAN::parseOnoffCmd(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][10] = ADJUSTMENT_CABLE_CAN::parseRScable(g1_packet_buffer[i][packetIndex].data(), 1);
                                    Result_Buffer[i][packetIndex][11] = ADJUSTMENT_CABLE_CAN::parseProportionValveStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][12] = ADJUSTMENT_CABLE_CAN::parseOnoffCtrl1(g1_packet_buffer[i][packetIndex].data(), 3);
                                    Result_Buffer[i][packetIndex][13] = ADJUSTMENT_CABLE_CAN::parseOnoffCtrl2(g1_packet_buffer[i][packetIndex].data(), 4);
                                    std::cout << "345201 反馈帧：启停反馈=" << +Result_Buffer[i][packetIndex][9]
                                            << ", 收放链反馈=" << +Result_Buffer[i][packetIndex][10]
                                            << ", 比例阀反馈=" << +Result_Buffer[i][packetIndex][11]
                                            << ", 弃链1反馈=" << +Result_Buffer[i][packetIndex][12]
                                            << ", 弃链2反馈=" << +Result_Buffer[i][packetIndex][13] << " || ";
                                    break;
                                }
                                case 0x345202: { // 周期性反馈帧，状态 + 放链信息
                                    int64_t interval = (current_Timestamp[i] - last_adj_ts[i][2]) / 1000;
                                    last_adj_ts[i][2] = current_Timestamp[i];
                                    adj_pack_ctrl[i][2] = packetIndex;
                                    Result_Buffer[i][packetIndex][14] = ADJUSTMENT_CABLE_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][15] = ADJUSTMENT_CABLE_CAN::checkZLXStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][16] = ADJUSTMENT_CABLE_CAN::checkSystemStatus(g1_packet_buffer[i][packetIndex].data(), 1);
                                    Result_Buffer[i][packetIndex][17] = ADJUSTMENT_CABLE_CAN::checkChainLengthStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][18] = ADJUSTMENT_CABLE_CAN::checkChainLengthStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                    Result_Buffer[i][packetIndex][19] = ADJUSTMENT_CABLE_CAN::checkSpeedStatus(g1_packet_buffer[i][packetIndex].data(), 6);
                                    Result_Buffer[i][packetIndex][20] = ADJUSTMENT_CABLE_CAN::checkSpeedStatus(g1_packet_buffer[i][packetIndex].data(), 7);
                                    std::cout << "345202 状态帧 || ";
                                    break;
                                }
                                case 0x345203: { // 压力、阀门状态、报警
                                    int64_t interval = (current_Timestamp[i] - last_adj_ts[i][3]) / 1000;
                                    last_adj_ts[i][3] = current_Timestamp[i];
                                    adj_pack_ctrl[i][3] = packetIndex;
                                    Result_Buffer[i][packetIndex][21] = ADJUSTMENT_CABLE_CAN::checkOutputTiming(interval);
                                    Result_Buffer[i][packetIndex][22] = ADJUSTMENT_CABLE_CAN::checkPressureStatus(g1_packet_buffer[i][packetIndex].data(), 0);
                                    Result_Buffer[i][packetIndex][23] = ADJUSTMENT_CABLE_CAN::checkPressureStatus(g1_packet_buffer[i][packetIndex].data(), 2);
                                    Result_Buffer[i][packetIndex][24] = ADJUSTMENT_CABLE_CAN::checkPressureStatus(g1_packet_buffer[i][packetIndex].data(), 4);
                                    Result_Buffer[i][packetIndex][25] = ADJUSTMENT_CABLE_CAN::checkValveStatus(g1_packet_buffer[i][packetIndex].data(), 6);
                                    Result_Buffer[i][packetIndex][26] = ADJUSTMENT_CABLE_CAN::checkValveStatus(g1_packet_buffer[i][packetIndex].data(), 7);
                                    Result_Buffer[i][packetIndex][27] = ADJUSTMENT_CABLE_CAN::checkAlarmGroup1Status(g1_packet_buffer[i][packetIndex].data(), 8);
                                    Result_Buffer[i][packetIndex][28] = ADJUSTMENT_CABLE_CAN::checkAlarmGroup2Status(g1_packet_buffer[i][packetIndex].data(), 9);
                                    std::cout << "345203 压力/阀/报警帧 || ";
                                    break;
                                }
                                default: break;
                            }
                            break;
                        }


                                default:
                                    // 处理默认情况
                                    std::cerr << "Unknown ID: 0x" << std::hex << ID[i] << std::endl;
                                    break;
                            }
                            

                        default:
                            break;
                    }
                    read_indices[i] = (read_indices[i] + 1) % BUFFER_SIZE;
                }
            }
        }
    }
}
void summarizeResults() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            for (size_t i = 0; i < CAN_SIZE; i++) {
                std::cout << "第 " << i+1 << " 路结果汇总是：" << std::endl;
                int count0 = 0, count1 = 0, count2 = 0, count3 = 0;
                int count0_1 = 0, count1_1 = 0, count2_1 = 0, count3_1 = 0;
                int noFeedbackCount = 0, slowFeedbackCount = 0;
                int errorCount = 0;

                for (size_t j = 0; j < BUFFER_SIZE; j++) {
                    switch (Result_Buffer[i][j][6]) {
                        case 0: count0++; break;
                        case 1: count1++; break;
                        case 2: count2++; break;
                        case 3: count3++; break;
                    }
                    switch (Result_Buffer[i][j][7]) {
                        case 1: slowFeedbackCount++; break;
                        case 2: noFeedbackCount++; break;
                    }
                    if (Result_Buffer[i][j][8] == 1) {
                        errorCount++;
                    }
                    switch (Result_Buffer[i][j][9]) {
                        case 0: count0_1++; break;
                        case 1: count1_1++; break;
                        case 2: count2_1++; break;
                        case 3: count3_1++; break;
                    }
                }

                std::cout << "输出指令发送过快次数: " << count2 << std::endl;
                std::cout << "输出指令发送过慢次数: " << count1 << std::endl;
                std::cout << "无指令发送次数: " << count3 << std::endl;
                std::cout << "设备无反馈次数: " << noFeedbackCount << std::endl;
                std::cout << "反馈超过500微秒次数: " << slowFeedbackCount << std::endl;
                std::cout << "指令解析错误次数: " << errorCount << std::endl;
                std::cout << "指令1发送过快次数: " << count2_1 << std::endl;
                std::cout << "指令1发送过慢次数: " << count1_1 << std::endl;
                std::cout << "指令1无指令发送次数: " << count3_1 << std::endl;
            }
        }
    }
}
void process_can_data(const struct can_frame &frame, int interface_index) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    int packetIndex = write_indices[interface_index] % BUFFER_SIZE;

    if (static_cast<size_t>(write_indices[interface_index]) < BUFFER_SIZE) {
        g_packet_buffer[interface_index][packetIndex][25] = (frame.can_id & CAN_RTR_FLAG) ? 1 : 0;
        std::memset(g_packet_buffer[interface_index][packetIndex].data() + 1, 0, 2);

        auto currentTimestamp = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTimestamp.time_since_epoch()).count();
        std::memcpy(&g_packet_buffer[interface_index][packetIndex][2], &timestamp, sizeof(timestamp));

        uint32_t frameID = frame.can_id & CAN_EFF_MASK;
        std::memcpy(&g_packet_buffer[interface_index][packetIndex][21], &frameID, sizeof(frameID));

        if (!(frame.can_id & CAN_RTR_FLAG)) {
            if (frame.can_dlc <= (FRAME_SIZE - 26)) {
                std::memcpy(&g_packet_buffer[interface_index][packetIndex][26], frame.data, frame.can_dlc);
                g_packet_buffer[interface_index][packetIndex][26 + frame.can_dlc] = '\0';
            } else {
                std::cerr << "错误，存储字节数不够" << interface_index << std::endl;
            }
        }
        
        write_indices[interface_index] = (write_indices[interface_index] + 1) % BUFFER_SIZE;
    } else {
        std::cerr << "Buffer full for interface " << interface_index << std::endl;
    }
}

