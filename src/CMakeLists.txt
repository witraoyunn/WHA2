#CMake最低版本号要求 
cmake_minimum_required(VERSION 2.8) 

#指定项目名称 
project(lbfs) 

#若是需要指定编译器路径 
#set(CROSS_TOOLCHAIN_PREFIX "/path/arm-linux-") 

#指定可执行文件的输出目录，输出到bin下面  
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)

#指定编译器 
#set(CMAKE_C_COMPILER   "${CROSS_TOOLCHAIN_PREFIX}gcc") 
#set(CMAKE_CXX_COMPILER "${CROSS_TOOLCHAIN_PREFIX}g++") 

#使用默认路径的g++指定编译器   
set(CMAKE_CXX_COMPILER "g++-4.8")

set(CMAKE_CXX_FLAGS_RELEASE "-Wall -std=c++11 -pthread -fpermissive -fno-rtti -O2") 
set(CMAKE_CXX_FLAGS_DEBUG  "-Wall -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0 -pthread -fpermissive -fno-rtti -O0 -g") 
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -pthread -fpermissive -fno-rtti -O2 -g")

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_SKIP_BUILD_RPATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "../lib")
set(CMAKE_LIBRARY_PATH "../lib")

# 指定宏定义 --------------------------------------------------------

# 数据存储
add_definitions(-DSTOREDATA)
# 脱机保护
add_definitions(-DOFFLINE_PROTECTION)
# 自动连接下位机
#add_definitions(-DAUTO_CONNECT)

# 中原项目 
#add_definitions(-DZHONGYUAN_PRJ) 
# 中原项目数据库
#add_definitions(-DZHONGYUAN_DB)
# 中航项目
#add_definitions(-DZHONGHANG_PRJ)
#add_definitions(-DZHONGHANG_PRJ_NEW)
# 化成工艺自动化
#add_definitions(-DFORMATION_TECH_AUTO)

# 5V160A下位机协议 (模拟方案)
add_definitions(-DPROTOCOL_5V160A)
# 5V80A下位机协议 (数字方案)
#add_definitions(-DPROTOCOL_5V80A)
# 中原BTS
#add_definitions(-DZHONGYUAN_BTS)
# 校准工装 (64通道)
#add_definitions(-DCABTOOL_64CH)

# PLC使能 (只要使用到PLC，就必须打开此项)
add_definitions(-DENABLE_PLC)
# 精实负压PLC
#add_definitions(-DSIEMENS_PLC)
# 精实负压化成PLC (中原项目)
#add_definitions(-DPLC_JS_VF)
# 精实恒温PLC
#add_definitions(-DPLC_JS_CT)
# 精毅通针床PLC
#add_definitions(-DPLC_JYT)
# 软包样线PLC
#add_definitions(-DMITSUBISHI_PLC)
# 中航武汉预充
#add_definitions(-DAWH_PRECHARGE_SYS)
# 中航武汉化成
#add_definitions(-DAWH_FORMATION_SYS)
# 中航武汉分容
add_definitions(-DAWH_GRADING_SYS)

# 中航系统保护
#add_definitions(-DZHONGHANG_SYSTEM_PROTECTION)
# 辅助电压采集盒
#add_definitions(-DAUX_VOLTAGE_ACQ)
# 辅助温度采集盒
add_definitions(-DAUX_TEMPERATURE_ACQ)
# 通道启动延时
#add_definitions(-DCHANNEL_START_DELAY)
# 通道存储库位温度
#add_definitions(-DCHANNEL_STORE_CELL_TEMPER)
# 指定宏定义 --------------------------------------------------------

#指定编译选项 
set(CMAKE_BUILD_TYPE Debug) 

#指定编译目录 
set(PROJECT_BINARY_DIR ${PROJECT_SOURCE_DIR}/../build)

#指定头文件目录,PROJECT_SOURCE_DIR为工程的根目录   
include_directories(${PROJECT_SOURCE_DIR}/../3rd/include
					${PROJECT_SOURCE_DIR}/../3rd/include/zmq
					./com ./utility ./cab ./cell ./channel ./pins ./dcir ./sqlite
					)
					
add_subdirectory(com)  
add_subdirectory(cab) 
add_subdirectory(cell)
add_subdirectory(channel)  
add_subdirectory(pins) 
add_subdirectory(dcir) 
add_subdirectory(utility) 
add_subdirectory(sqlite) 

link_directories(${PROJECT_SOURCE_DIR}/../lib)
link_libraries(com)
link_libraries(cab)
link_libraries(cell)
link_libraries(channel)
link_libraries(pins)
link_libraries(dcir)
link_libraries(utility)
link_libraries(sqlite)

#指定生成目标 
add_executable(lbfs main.cpp) 

#链接共享库 
target_link_libraries(lbfs com cab cell channel pins dcir utility sqlite pthread json zmq snap7 xml2 modbus -ldl)

