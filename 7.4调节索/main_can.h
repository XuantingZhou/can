/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-07-03 22:14:43
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-03 22:14:44
 * @FilePath: /6.29/main_can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MAIN_PUBLIC_H
#define MAIN_PUBLIC_H

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <chrono>
#include <array>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
#include "EMERGENCY_AUXILIARY_PROPULSION_CAN.h"
#include "MAIN_PROPULSION_CAN.h"
#include "STEERING_SYSTEM_CAN.h"
#include "ADJUSTMENT_CABLE_CAN.h"

// 常量定义
constexpr size_t CAN_SIZE = 3;      // 目前最多3路CAN诊断
constexpr size_t BUFFER_SIZE = 6000;
constexpr size_t FRAME_SIZE = 100;
constexpr size_t RESULT_BUFFER_SIZE = 6000;
constexpr size_t RESULT_FRAME_SIZE = 10;

// CAN接口名称
extern const std::string INTERFACE_NAMES[CAN_SIZE];

// 索引数组
extern int write_indices[CAN_SIZE];
extern int read_indices[CAN_SIZE];
extern int local_write_indices[CAN_SIZE];
extern int pack_control[24][4];

// 时间戳数组
extern std::array<int64_t, CAN_SIZE> current_Timestamp;
extern std::array<int64_t, CAN_SIZE> last_Timestamp;
extern std::array<int32_t, CAN_SIZE> ID;
extern std::array<int64_t, CAN_SIZE> remote_Timestamp1;
extern std::array<int64_t, CAN_SIZE> back_Timestamp1;
extern std::array<int64_t, CAN_SIZE> current_Timestamp_1;
extern std::array<int64_t, CAN_SIZE> last_Timestamp_1;
extern std::array<int64_t, CAN_SIZE> last_Timestamp_2;
extern std::array<int64_t, CAN_SIZE> last_Timestamp_3;


// 缓冲区
extern std::array<std::array<std::array<unsigned char, FRAME_SIZE>, BUFFER_SIZE>, CAN_SIZE> g_packet_buffer;
extern std::array<std::array<std::array<unsigned char, RESULT_FRAME_SIZE>, RESULT_BUFFER_SIZE>, CAN_SIZE> Result_Buffer;

// 同步工具
extern std::mutex buffer_mutex;
extern std::condition_variable data_condition;
extern std::mutex result_mutex;
extern bool running;

// 时间戳记录case 0x32021:
extern std::map<size_t, std::map<uint32_t, int64_t>> last_timestamp_map;

// 函数声明
int64_t calcFrameInterval(size_t channel, uint32_t frame_id, int64_t current_timestamp);
void parseData();
void summarizeResults();

void process_can_data(const struct can_frame &frame, int interface_index);


#endif