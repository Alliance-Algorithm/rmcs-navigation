install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}/
)
install(
    PROGRAMS
        src/script/goal_topic_bridge.py
        src/script/follow_waypoints_runner.py
    DESTINATION lib/${PROJECT_NAME}/
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
    DIRECTORY maps/
    DESTINATION share/${PROJECT_NAME}/maps/
)
install(
    DIRECTORY src/lua/
    DESTINATION share/${PROJECT_NAME}/lua/
)

find_package(ament_cmake REQUIRED)
ament_package()
