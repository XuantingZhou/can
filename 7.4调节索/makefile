# source /home/wjj/AM62xx/SDK/linux-devkit/environment-setup

# 交叉编译工具链定义
CXX      := aarch64-none-linux-gnu-g++  # C++ 交叉编译器
CC       := aarch64-none-linux-gnu-gcc  # C 交叉编译器（保留备用）

# 编译选项
CFLAGS   := -Wall -g             # 通用编译警告和调试选项
CXXFLAGS := $(CFLAGS) -std=c++11 # C++特有选项（C++11标准）
INCLUDES := -I.                  # 头文件搜索路径
LDFLAGS  := -pthread -lstdc++    # 链接选项（线程和C++标准库）

# 源文件定义
SRCS := main.cpp \
		EMERGENCY_AUXILIARY_PROPULSION_CAN.cpp \
		main_can.cpp \
		MAIN_PROPULSION_CAN.cpp \
        STEERING_SYSTEM_CAN.cpp\
		ADJUSTMENT_CABLE_CAN.cpp
        # AUXILIARY_PROPULSION_CAN.cpp \
        # STEERING_SYSTEM_CAN.cpp

# 自动生成目标文件列表
OBJS := $(SRCS:.cpp=.o)

# 最终目标
TARGETS := main

# 默认构建所有目标
all: $(TARGETS)

# 主程序构建规则
main: $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# 通用编译规则（.cpp -> .o）
%.o: %.cpp
	$(CXX) -c $< -o $@ $(INCLUDES) $(CXXFLAGS)

# 伪目标声明
.PHONY: all clean

# 清理规则
clean:
	rm -f $(OBJS) $(TARGETS)