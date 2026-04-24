cmake_minimum_required(VERSION 3.14.0)
project(ur_driver_examples)

# list(APPEND CMAKE_PREFIX_PATH "/home/testuser/wk/Universal_Robots_Client_Library")
find_package(ur_client_library REQUIRED PATHS "/home/testuser/install/lib/cmake/ur_client_library/" NO_DEFAULT_PATH)
set(CMAKE_VERBOSE_MAKEFILE ON)


add_executable(rotate_J6
  test_J6_rotation.cpp)
target_link_libraries(rotate_J6 ur_client_library::urcl)


add_executable(howdy_client 
  howdy_client.cpp)
target_link_libraries(howdy_client ur_client_library::urcl zmqpp zmq)


add_executable(meen612driver
  meen612driver.cpp)
target_link_libraries(meen612driver ur_client_library::urcl zmqpp zmq)
