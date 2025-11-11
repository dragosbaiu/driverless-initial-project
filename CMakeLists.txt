cmake_minimum_required(VERSION 3.16)
project(driverless-initial-project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/local")

find_package(raylib CONFIG REQUIRED)

set(SOURCES
    src/main.cpp
    src/vehicle.cpp
    src/graphics.cpp
    src/io.cpp
    src/sim.cpp
    src/control.cpp
    src/noise.cpp
    src/path.cpp
)

add_executable(driverless-initial-project ${SOURCES})

target_include_directories(driverless-initial-project PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(driverless-initial-project PRIVATE raylib)
