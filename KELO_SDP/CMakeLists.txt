project(KELO_SDP)
cmake_minimum_required(VERSION 3.5)

find_package(soem REQUIRED)

add_executable(kelo_control
  EthercatCommunication.c
  PlatformToWheelInverseKinematicsSolver.c
  SmartWheelKinematics.c
  KELORobotKinematics.c
)

target_link_libraries(kelo_control
  soem
  pthread
  gsl
  gslcblas
  m
)