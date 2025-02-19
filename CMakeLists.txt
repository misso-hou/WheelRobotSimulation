cmake_minimum_required(VERSION 3.12)

project(simulator)

#外部库加载
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# option(ENABLE_DISPLAY_TOOL "enable display option" ON)

#指定路径信息
set(PROJECT_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/bin")
file(MAKE_DIRECTORY ${PROJECT_OUTPUT_DIRECTORY})
set(CSV_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/bin/csvPlt")
file(MAKE_DIRECTORY ${CSV_OUTPUT_DIRECTORY})
# 在 CMakeLists.txt 中定义预处理宏，传递工程根目录路径
add_definitions(-DCSV_SOURCE_DIR="${CSV_OUTPUT_DIRECTORY}")
add_definitions(-DPROJECT_DIR="${CMAKE_SOURCE_DIR}")
# 仿真倍速
add_definitions(-DSIMULATION_RATE=6)

#添加子模块
add_subdirectory(modules)

#在指定位置生成执行程序
# include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_OUTPUT_DIRECTORY}"
)

#链接子模块库
target_link_libraries(${PROJECT_NAME} planning)
target_link_libraries(${PROJECT_NAME} datacenter)
target_link_libraries(${PROJECT_NAME} logger)
target_link_libraries(${PROJECT_NAME} animation)
target_link_libraries(${PROJECT_NAME} environment)
target_link_libraries(${PROJECT_NAME} sensor)
target_link_libraries(${PROJECT_NAME} control)
target_link_libraries(${PROJECT_NAME} vehicle)
target_link_libraries(${PROJECT_NAME} gridmap)

#外部库链接
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES} pybind11::embed
                      matplotlibcpp17::matplotlibcpp17)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} Threads::Threads)

#链接其他头文件
target_include_directories(${PROJECT_NAME} PUBLIC  ${Python3_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/utilities/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/vehicle/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/logger/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/animation/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/datacenter/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/environment/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/sensor/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/control/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/planning/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/gridmap/include)