cmake_minimum_required(VERSION 3.10)
project(me553_final LANGUAGES CXX)
set(app_name me553_final)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories (${EIGEN3_INCLUDE_DIRS})

add_executable(${app_name} main.cpp)
set_target_properties(${app_name} PROPERTIES MACOSX_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/mac/lib")
if(WIN32)
    target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32 Winmm)
else()
    target_link_libraries(${app_name} PUBLIC raisim::raisim pthread)
endif()

target_compile_definitions(${app_name} PUBLIC "-DRSC_DIR=${CMAKE_CURRENT_SOURCE_DIR}/rsc")