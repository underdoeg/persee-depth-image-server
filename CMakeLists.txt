cmake_minimum_required(VERSION 3.5)
project(openni2-net-stream-server)

# add modules path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gstreamer)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3 REQUIRED )

find_package(Boost 1.4 REQUIRED system)


## check if system is ARM, if so app is compiled on persee and needs the included openni2 libs
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    message("--Using OpenNI2 libraries from the libs folder")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/openNI2/include)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/openNI2/lib)

    set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libs/openNI2/lib)
    message(${CMAKE_PREFIX_PATH})

    find_library(OPENNI2_LIB OpenNI2 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/libs/openNI2/lib)
    message("-- OpenNI2 lib found: " ${OPENNI2_LIB})
    #target_link_libraries (openni2-net-stream-server OpenNI2)

ELSE(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    message("-- Using System OpenNI2 libraries")
    find_package(OpenNI2)

    include_directories(${OPENNI2_INCLUDE_DIRS})
    link_libraries(${OPENNI2_LIBRARIES})

ENDIF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")


######################################################################################################################
## COMMON

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(openni2-net-stream-common common/openni2-net-common.cpp common/openni2-net-common.h common/loguru.hpp)
target_include_directories(openni2-net-stream-common PUBLIC ${Boost_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/common)
target_link_libraries(openni2-net-stream-common PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} Threads::Threads dl)


######################################################################################################################
## SERVER
add_executable(openni2-net-stream-server server/server.cpp server/grabber.cpp server/grabber.h server/sender.cpp server/sender.h)

target_link_libraries(openni2-net-stream-server openni2-net-stream-common OpenNI2)


######################################################################################################################
## CLIENT
add_library(openni2-net-stream-client client/openni2-net-client.cpp client/openni2-net-client.h)
target_include_directories(openni2-net-stream-client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/client)
target_link_libraries(openni2-net-stream-client openni2-net-stream-common)


######################################################################################################################
## CLIENT SAMPLE
add_executable(openni2-net-stream-client-sample client-sample/client-sample.cpp)
target_link_libraries(openni2-net-stream-client-sample openni2-net-stream-client)
