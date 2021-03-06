cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## C++11 Compiler usage added
add_compile_options(-std=c++11)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation tf)


add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  AddTwoInts.srv
  text.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

## Test build
if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)

   add_rostest_gtest(
      talkerTest
      test/main.cpp
      test/talkerTest.cpp
      launch/talkerTest.launch
   )

   target_link_libraries(
      talkerTest
      ${catkin_LIBRARIES}
   )

   add_dependencies(
      talkerTest
      talker
      ${PROJECT_NAME}_generated_messages_cpp
      ${catkin_EXPORTED_TARGETS}
   )
endif()

