cmake_minimum_required(VERSION 3.5)
project(jetson_local_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

find_package(ZED REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} camera_capture.cpp)

if (LINK_SHARED_ZED)
    set(ZED_LIBS
        ${ZED_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_CUDART_LIBRARY}
    )
else()
    set(ZED_LIBS
        ${ZED_STATIC_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_LIBRARY}
    )
endif()

target_link_libraries(${PROJECT_NAME} ${ZED_LIBS})