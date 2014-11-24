PROJECT(exoter_odometry)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(exoter_odometryBase)

install(PROGRAMS bin/exoter-odometer
    DESTINATION bin)


