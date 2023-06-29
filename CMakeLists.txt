cmake_minimum_required(VERSION 3.5)

project(ModbusClient LANGUAGES CXX)
set(PROJECT_NAME "ModbusClient")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)

include_directories("includes")
link_directories("lib")

add_executable(${PROJECT_NAME}
    main.cpp
    src/enovasensemodbusclient.h
    src/enovasensemodbusclient.cpp
)
target_link_libraries(${PROJECT_NAME} modbus-5)
