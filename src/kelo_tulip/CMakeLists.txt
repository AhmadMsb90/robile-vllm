cmake_minimum_required(VERSION 3.8)
project(kelo_tulip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS thread)

if(WIN32)
	set(INCLUDE_DIRS
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/win32
		include/kelo_tulip/soem/oshw/win32
		${WINPCAP_INCLUDE_DIRS}
	)
else(WIN32)
	set(INCLUDE_DIRS
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/linux
		include/kelo_tulip/soem/oshw/linux
	)
endif(WIN32)


include_directories(
	${INCLUDE_DIRS}
	${rclcpp_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${geometry_msgs_INCLUDE_DIRS}
	${nav_msgs_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${tf2_INCLUDE_DIRS}
)

# Optionally apply cap_net_raw+ep permissions to binary. Requires root previleges.
# Use sudo as default. Still, allow to be configured as environment veriable.
OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)
IF("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND sudo)				# default = sudo
ELSE("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND $ENV{SUDO_COMMAND})
ENDIF("$ENV{SUDO_COMMAND}" STREQUAL "")

add_subdirectory(src/soem)

add_library(tulip_velocity_controller
	src/Utils.cpp
	src/VelocityPlatformController.cpp
)

target_link_libraries(tulip_velocity_controller
	${rclcpp_LIBRARIES}
	${std_msgs_LIBRARIES}
	${geometry_msgs_LIBRARIES}
	${ament_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(platform_driver
	src/EtherCATModule.cpp
	src/PlatformDriver.cpp
	src/PlatformDriverROS.cpp
	src/modules/RobileMasterBattery.cpp
)

ament_target_dependencies(platform_driver rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
#rosidl_target_interfaces(platform_driver ${PROJECT_NAME})

target_link_libraries(platform_driver
	tulip_velocity_controller
	${ament_LIBRARIES}
	${Boost_LIBRARIES}
	soem
	pthread
)

IF(USE_SETCAP)
  add_custom_command(TARGET platform_driver POST_BUILD
  	COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:platform_driver>
  )
endif(USE_SETCAP)

install(TARGETS platform_driver tulip_velocity_controller
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
	DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
