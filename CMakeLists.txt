cmake_minimum_required(VERSION 2.8.12)

project(offboard)

if(MINGW)
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
endif()

if(MSVC)
    add_definitions("-std=c++11 -WX -W2")
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
else()
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
endif()

find_package(MAVSDK REQUIRED)

find_package( OpenCV REQUIRED )

add_executable(
    offboard_vpe offboard_vpe.cpp
)


target_link_libraries(offboard_vpe
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mavlink_passthrough
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
)
