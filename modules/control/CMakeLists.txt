project(control)

#外部库链接
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

#建立子库
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp) 
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

#链接自身头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

#链接其他头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/utilities/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/vehicle/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/datacenter/include)

#外部库链接	
target_link_libraries(${PROJECT_NAME} datacenter)
target_link_libraries(${PROJECT_NAME} vehicle)
#第三方库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} OsqpEigen::OsqpEigen)
target_link_libraries(${PROJECT_NAME} Threads::Threads)