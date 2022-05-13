project(KELO_SDP)
cmake_minimum_required(VERSION 3.5)

find_package(soem REQUIRED)

add_executable(robile_control
  EthercatCommunication.c
  PlatformToWheelInverseKinematicsSolver.c
  SmartWheelKinematics.c
  KELORobotKinematics.c
)

target_link_libraries(robile_control
  soem
  gsl
  gslcblas
  m
)