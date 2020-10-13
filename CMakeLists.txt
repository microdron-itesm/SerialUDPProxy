cmake_minimum_required(VERSION 3.17)
project(SerialUDPProxy)

set(CMAKE_CXX_STANDARD 14)
add_subdirectory(libs/UDPLib)
include_directories(libs/UDPLib/include)

find_package( Boost REQUIRED COMPONENTS program_options )
include_directories( ${Boost_INCLUDE_DIRS} )

find_package(fmt)

file(GLOB_RECURSE SerialUDPProxy_SRC
        "src/*.h"
        "src/*.cpp"
        "src/**/*.cpp"
        "src/**/*.hpp"
        "src/**/*.h"
        "src/*.hpp"
        "src/*.c"
        "src/**/*.c"
        "libs/*.c"
        "libs/*.h"
        "libs/UDPLib/src/UDP.c"
        )

add_executable(SerialUDPProxy ${SerialUDPProxy_SRC})
target_link_libraries(SerialUDPProxy ${Boost_LIBRARIES} fmt::fmt UDPLib)