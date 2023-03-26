cmake_minimum_required(VERSION 3.0.2)
project(hrc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    roslaunch
    )


find_package(ur_rtde REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
# add Eigen3
find_package(Eigen3 REQUIRED)

if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

# catkin_python_setup()

add_message_files(
    FILES
    RobotPose.msg
    RobotControl.msg
    RobotData.msg
    ProcessedFTData.msg
    KeyValue.msg
    )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hrc
    CATKIN_DEPENDS roscpp rospy message_runtime
    DEPENDS system_lib EIGEN3
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ~/catkin_ws/devel/include
    )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hrc.cpp
# )

add_library(   ##用来手动链接.hpp和.cpp文件
    process_ft_data          ##自定义的链接库名，后面用来识别
    include/process_ft_data.h    ##.hpp的路径
    src/process_ft_data.cpp     ##.cpp的路径
    )

add_library(
    process_robot_data
    include/process_robot_data.h
    src/process_robot_data.cpp
    )

add_library(
    controller
    include/controller.h
    src/controller.cpp
    )

add_library(
    virtual_fixture
    include/virtual_fixture.h
    src/virtual_fixture.cpp
    )
add_library(
    generic_api
    include/generic_api.h
    src/generic_api.cpp
    )
add_library(
    process_keyboard_event
    include/process_keyboard_event.h
    src/process_keyboard_event.cpp
    )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ur_move src/ur_move.cpp)
target_link_libraries(ur_move ${catkin_LIBRARIES} ur_rtde::rtde)

#add_executable(jog_udrl src/jog_udrl.cpp)
#target_link_libraries(jog_udrl ${catkin_LIBRARIES} ur_rtde::rtde)
#target_include_directories(jog_udrl PUBLIC ${CURSES_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
#target_link_libraries(jog_udrl PRIVATE rtde ${CURSES_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
#				set_target_properties(jog_udrl
#					PROPERTIES
#					RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
#					)
#				add_executable(ur_rtde::jog_udrl ALIAS jog_udrl)

add_executable(test_SpeedJ src/test_SpeedJ.cpp)
target_link_libraries(test_SpeedJ ${catkin_LIBRARIES} ur_rtde::rtde)

add_executable(get_ur_data src/get_ur_data.cpp)
target_link_libraries(get_ur_data ${catkin_LIBRARIES} ur_rtde::rtde)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} )

add_executable(get_ft_data src/get_ft_data.cpp)
target_link_libraries(get_ft_data ${catkin_LIBRARIES})

add_executable(record_node src/record_node.cpp)
target_link_libraries(record_node ${catkin_LIBRARIES}
    generic_api)

add_executable(keyboard_detection src/keyboard_detection.cpp)
target_link_libraries(keyboard_detection ${catkin_LIBRARIES})

add_executable(hrc_assembly_main src/hrc_assembly_main.cpp)
target_link_libraries(hrc_assembly_main
    process_ft_data
    process_robot_data
    process_keyboard_event
    controller
    generic_api
    ${catkin_LIBRARIES})

add_executable(hrc_vf_main src/hrc_vf_main.cpp)
target_link_libraries(hrc_vf_main
    virtual_fixture
    process_ft_data
    process_robot_data
    controller
    ${catkin_LIBRARIES})

add_executable(myadmittance_cart src/myadmittance_cart.cpp)
target_link_libraries(myadmittance_cart
    virtual_fixture
    process_ft_data
    process_robot_data
    controller
    ${catkin_LIBRARIES})

add_executable(hrc_gtc_main src/hrc_gtc_main.cpp)
target_link_libraries(hrc_gtc_main
    process_ft_data
    process_robot_data
    controller
    generic_api
    ${catkin_LIBRARIES})


add_executable(casic_assembly src/casic_assembly.cpp)
target_link_libraries(casic_assembly
    process_ft_data
    process_robot_data
    process_keyboard_event
    controller
    generic_api
    ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_man_machine_coordination.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
