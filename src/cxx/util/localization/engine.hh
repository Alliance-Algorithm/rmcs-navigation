#pragma once
#include "cxx/util/pimpl.hh"

#include <Eigen/Geometry>
#include <expected>
#include <future>
#include <rclcpp/node.hpp>

namespace rmcs::navigation {

class Localization {
    RMCS_PIMPL_DEFINITION(Localization)

public:
    struct Config {
        rclcpp::Node& rclcpp;

        std::string topic_registered;
        std::string map_filename;

        float ndt_resolution = 1.0;
        double ndt_step_size = 0.1;
        double ndt_result_epsilon = 0.01;
        int ndt_max_iterations = 50;
    };

    explicit Localization(Config config);

    // 开始收集配准好的点云
    auto start_collecting(std::chrono::seconds seconds) -> std::expected<void, std::string>;

    // 开始重定位
    auto start_localizing(const Eigen::Isometry3d& initial_solution)
        -> std::future<std::expected<Eigen::Isometry3d, std::string>>;
};

} // namespace rmcs::navigation
