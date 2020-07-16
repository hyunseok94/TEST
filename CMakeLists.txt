cmake_minimum_required(VERSION 3.0.2)
project(elmo_pkgs2)

add_compile_options(-std=c++11)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES elmo_pkgs2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/ethercat/osal
  include/ethercat/oshw
  include/ethercat/soem
  include/ethercat/etc
  /usr/xenomai/include/cobalt
  /usr/xenomai/include/
  /usr/xenomai/include/alchemy
  /usr/xenomai/lib
  /usr/xenomai/
)

SET(SRC_FILES
 src/main.cpp
 include/ethercat/etc/servo.c
 include/ethercat/etc/ecat_dc.c
 include/ethercat/etc/slave_info.c
 include/ethercat/osal/osal.c
 include/ethercat/oshw/nicdrv.c
 include/ethercat/oshw/oshw.c
 include/ethercat/soem/ethercatbase.c
 include/ethercat/soem/ethercatcoe.c
 include/ethercat/soem/ethercatconfig.c
 include/ethercat/soem/ethercatdc.c
 include/ethercat/soem/ethercatfoe.c
 include/ethercat/soem/ethercatmain.c
 include/ethercat/soem/ethercatprint.c
 include/ethercat/soem/ethercatsoe.c
)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=alchemy --rtdm --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=alchemy --rtdm --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ${XENO_CFLAGS} ${XENO_LDFLAGS} -lalchemy -lcopperplate -D_GNU_SOURCE -D_REENTRANT -D__COBALT__")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(elmo_pkgs2_exe ${SRC_FILES})

target_link_libraries(elmo_pkgs2_exe ${catkin_LIBRARIES})
