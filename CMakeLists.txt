cmake_minimum_required(VERSION 2.8.3)
project(imagine_asc)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  vision_msgs
  imagine_common
  std_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  imagine_common
 )

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
