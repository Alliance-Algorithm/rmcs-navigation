install(
    TARGETS app
    DESTINATION lib/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
ament_package()
