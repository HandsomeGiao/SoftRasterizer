cmake_minimum_required(VERSION 4.0)
project(SoftwareRasterizerExample)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/*.cpp")
add_executable(SoftwareRasterizerExample ${SOURCES})

target_link_libraries(SoftwareRasterizerExample ${OpenCV_LIBS})
target_link_directories(SoftwareRasterizerExample PRIVATE ${CMAKE_SOURCE_DIR}/include)
