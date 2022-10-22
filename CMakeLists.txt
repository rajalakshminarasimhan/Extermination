cmake_minimum_required(VERSION 2.8.3)
project(extermination_control)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs tf2 tf2_ros)
find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    rospy
    tf2
    tf2_ros
    tf_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/navigation_sim.py scripts/image_rec_sim.py  scripts/weed_sorting.py scripts/sprayers.py scripts/sprayers_leds.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
