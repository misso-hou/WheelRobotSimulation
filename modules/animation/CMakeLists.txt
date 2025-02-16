project(animation)

#外部库加载
find_package(OpenCV REQUIRED)
#新增加的可视化工具库
set(pybind11_DIR "/usr/lib/cmake/pybind11")
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

#建立子库
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp) 
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

#链接自身头文件
include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#链接其他头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/utilities/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/datacenter/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/vehicle/include)

#外部库链接
target_link_libraries(${PROJECT_NAME} datacenter)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# 包含pythont头文件
target_include_directories(${PROJECT_NAME} PUBLIC ${Python3_INCLUDE_DIRS})
# 链接Python库
target_link_libraries(${PROJECT_NAME} PUBLIC ${Python3_LIBRATIES})