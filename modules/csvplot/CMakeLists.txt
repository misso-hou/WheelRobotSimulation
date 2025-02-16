project(csvplot)

#生成可执行文件
add_executable(${PROJECT_NAME})
#搜索当前目录下的所有源文件并将其添加到可执行文件中
target_sources($P{PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/data_play.cpp)
#设置输出路径
set_target_properties(${PROJECT_NAME} EXPORT_PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CSV_OUTPUT_DIRECTORY}"
)   

#链接自身头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

#外部库链接	
target_link_libraries(${PROJECT_NAME} datacenter)
target_link_libraries(${PROJECT_NAME} animation)

#链接其他头文件
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/modules/datacenter/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/utilities/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/animation/include)