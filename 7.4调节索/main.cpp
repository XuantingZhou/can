#include "main_can.h"

int main() {
    struct ifreq ifr = {0};
    struct sockaddr_can can_addr = {0};
    int can_sockets[CAN_SIZE];
    struct can_frame frame;

    // 创建所有 CAN 套接字
    for (size_t i = 0; i < CAN_SIZE; i++) {
        can_sockets[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_sockets[i] < 0) {
            perror("socket error");
            exit(EXIT_FAILURE);
        }

        std::strcpy(ifr.ifr_name, INTERFACE_NAMES[i].c_str());
        ioctl(can_sockets[i], SIOCGIFINDEX, &ifr);
        can_addr.can_family = AF_CAN;
        can_addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_sockets[i], reinterpret_cast<struct sockaddr*>(&can_addr), sizeof(can_addr)) < 0) {  
            perror("bind error");
            close(can_sockets[i]);
            exit(EXIT_FAILURE);
        }
    }

    // 启动线程
    std::thread parserThread(parseData);
    std::thread summaryThread(summarizeResults);

    while (true) {
        fd_set read_fds;
        FD_ZERO(&read_fds);  
        int max_fd = -1;

        for (size_t i = 0; i < CAN_SIZE; i++) {
            FD_SET(can_sockets[i], &read_fds);
            if (can_sockets[i] > max_fd) {
                max_fd = can_sockets[i];
            }
        }

        int ret = select(max_fd + 1, &read_fds, nullptr, nullptr, nullptr);
        if (ret < 0) {
            perror("Select");
            break;
        }

        for (size_t i = 0; i < CAN_SIZE; i++) {
            if (FD_ISSET(can_sockets[i], &read_fds)) {
                read(can_sockets[i], &frame, sizeof(frame));
                process_can_data(frame, i);
            }
        }
    }

    // 清理资源
    for (size_t i = 0; i < CAN_SIZE; i++) {
        close(can_sockets[i]);
    }

    running = false;
    data_condition.notify_all();
    parserThread.join();
    summaryThread.join();

    return EXIT_SUCCESS;
}