install(
    TARGETS ${PROJECT_NAME}_brain
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
)
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/
)
install(
    PROGRAMS
        scripts/debug_goal_bridge.py
        scripts/follow_waypoints_runner.py
        scripts/static_grid_publisher.py
        scripts/goal_topic_bridge.py
    DESTINATION
        lib/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
ament_package()
