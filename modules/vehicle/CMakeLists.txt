project(vehicle)	

#第三方库链接
find_package(Eigen3 REQUIRED)

#建立子库
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIRj/src/* .cpp) 
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

#链接自身头文件
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#链接其他头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DlR}/utilities/include)

#外部库链接
target_link_libraries(${PROJECT_NAME} Eigen3: :Eigen)
