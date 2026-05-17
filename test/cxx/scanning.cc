#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <memory>
#include <tuple>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sol/sol.hpp>
#include <visualization_msgs/msg/marker.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("scanning_visualizer");
    auto publisher = node->create_publisher<visualization_msgs::msg::Marker>(
        "/rmcs_navigation/test/scanning_marker", 10);

    auto lua = sol::state{};
    lua.open_libraries(sol::lib::base, sol::lib::math, sol::lib::package);

    auto root = std::filesystem::path{__FILE__}.parent_path().parent_path().parent_path();
    auto package = lua["package"].get<sol::table>();
    auto package_path = package["path"].get_or(std::string{});
    package["path"] = std::format(
        "{};{}/src/lua/?.lua;{}/src/lua/?/init.lua;{}/src/lua/?/?.lua", package_path, root.string(),
        root.string(), root.string());

    auto module_result = lua.safe_script("return require('util.math')", sol::script_pass_on_error);
    if (!module_result.valid()) {
        auto error = module_result.get<sol::error>();
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    auto module = module_result.get<sol::table>();
    auto scanning_signal = module["scanning_signal"].get<sol::protected_function>();

    auto config_result = lua.safe_script_file(
        (root / "test/cxx/scanning.lua").string(), sol::script_pass_on_error);
    if (!config_result.valid()) {
        auto error = config_result.get<sol::error>();
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    auto config = config_result.get<sol::table>();

    const auto start = std::chrono::steady_clock::now();

    using namespace std::chrono_literals;
    auto timer = node->create_wall_timer(10ms, [&, node, publisher] {
        const auto timestamp =
            std::chrono::duration<double>{std::chrono::steady_clock::now() - start}.count();

        config["timestamp"] = timestamp;

        sol::protected_function_result result = scanning_signal(config);
        if (!result.valid()) {
            auto error = result.get<sol::error>();
            RCLCPP_ERROR(node->get_logger(), "%s", error.what());
            return;
        }

        auto values = result.get<std::tuple<double, double>>();
        const auto yaw = std::get<0>(values);
        const auto pitch = std::get<1>(values);

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = node->now();
        marker.header.frame_id = "world";
        marker.ns = "scanning";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        marker.color.a = 1.0F;
        marker.color.g = 1.0F;

        marker.points.resize(2);
        marker.points[0] = geometry_msgs::msg::Point{};
        marker.points[1].x = std::cos(pitch) * std::cos(yaw);
        marker.points[1].y = std::cos(pitch) * std::sin(yaw);
        marker.points[1].z = std::sin(pitch);

        publisher->publish(marker);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
