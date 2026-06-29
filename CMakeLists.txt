cmake_minimum_required(VERSION 3.0.2)
project(localizacao_husky)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  gazebo_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_conversor.py
  scripts/metricas.py
  scripts/resultados.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_replay.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY bags config launch results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)