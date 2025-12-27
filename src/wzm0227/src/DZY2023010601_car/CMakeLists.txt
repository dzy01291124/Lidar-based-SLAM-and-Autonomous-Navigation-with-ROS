cmake_minimum_required(VERSION 2.8.3)

project(wzm0227_car)

find_package(catkin REQUIRED COMPONENTS
		message_generation
		message_runtime
		roscpp
		roslaunch
		)

add_message_files(FILES
		xjupose.msg
		fusion_analysis.msg
		)

generate_messages()

catkin_package(CATKIN_DEPENDS message_generation message_runtime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xju_bridge_node node/xju_bridge_node.cpp src/xju_bridge.cpp)
add_dependencies(xju_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_bridge_node ${catkin_LIBRARIES})


add_executable(node_a src/a.cpp)  # 假设a.cpp在src目录下
add_dependencies(node_a ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_a ${catkin_LIBRARIES})

# 新增节点b
add_executable(node_b src/b.cpp)  # 假设b.cpp在src目录下
add_dependencies(node_b ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_b ${catkin_LIBRARIES})