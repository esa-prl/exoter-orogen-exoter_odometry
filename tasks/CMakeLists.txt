# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(exoter_odometryTaskLib)
ADD_LIBRARY(${EXOTER_ODOMETRY_TASKLIB_NAME} SHARED 
    ${EXOTER_ODOMETRY_TASKLIB_SOURCES})
add_dependencies(${EXOTER_ODOMETRY_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${EXOTER_ODOMETRY_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${EXOTER_ODOMETRY_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${EXOTER_ODOMETRY_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${EXOTER_ODOMETRY_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${EXOTER_ODOMETRY_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${EXOTER_ODOMETRY_TASKLIB_HEADERS}
    DESTINATION include/orocos/exoter_odometry)

