project(animation)

#外部库加载
find_package(OpenCV REQUIRED)
#新增加的可视化工具库
set(pybind11_DIR "/usr/lib/cmake/pybind11")
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

#建立子库
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp) 
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

#链接自身头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

#链接其他头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/utilities/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/datacenter/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/vehicle/include)

#外部库链接	
target_link_libraries(${PROJECT_NAME} datacenter)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

#新增库链接(python库手动指定路径，find_package行不通)
target_link_libraries(${PROJECT_NAME} pybind11::embed matplotlibcpp17::matplotlibcpp17)
# Manually specify the path to the Python3 executable (replace with your actual path)
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")
# Manually specify the include directories for Python3
set(PYTHON_INCLUDE_DIRS "/usr/include/python3.8")
# Manually specify the link directories for Python3 (replace with your actual path)
set(PYTHON_LIBRARY_DIRS "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu;/usr/lib")
# Manually specify the Python3 libraries to link against
set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")
# Include Python3 headers
include_directories(${PYTHON_INCLUDE_DIRS})
# Link against Python3 libraries
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
    