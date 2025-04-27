cmake_minimum_required(VERSION 3.14.0)
project(ur_driver_examples)
# find_package(ur_client_library REQUIRED)


add_executable(rotate_J6
  test_J6_rotation.cpp)
target_link_libraries(rotate_J6 urcl)


add_executable(howdy_client 
  howdy_client.cpp)
target_link_libraries(howdy_client urcl zmqpp zmq)


add_executable(meen612driver
  meen612driver.cpp)
target_link_libraries(meen612driver urcl zmqpp zmq)
