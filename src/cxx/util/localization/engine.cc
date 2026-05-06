#include "cxx/util/localization/engine.hh"
#include "cxx/util/node_mixin.hh"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs::navigation {

struct Localization::Impl {
    Config config;
    LoggerWrap<rclcpp::Node> logging;

    struct Context {
        using Point = pcl::PointXYZ;
        using Cloud = pcl::PointCloud<Point>;

        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> subscription;
        std::atomic<bool> stop_collecting = false;

        std::shared_ptr<Cloud> map;
        std::shared_ptr<Cloud> collected = std::make_shared<Cloud>();

        explicit Context(Config& config) {
            const auto& filename = config.map_filename;
            if (pcl::io::loadPCDFile<Point>(filename, *map) != 0)
                throw std::runtime_error{"Couldn't read " + filename};

            auto& rclcpp = config.rclcpp;
            subscription = rclcpp.create_subscription<sensor_msgs::msg::PointCloud2>(
                config.topic_registered, 10,
                [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                    if (!stop_collecting) {
                        auto received = Cloud{};
                        pcl::fromROSMsg(*msg, received);

                        *collected += received;
                    }
                });
        }
    } context;

    pcl::NormalDistributionsTransform<Context::Point, Context::Point> engine;

    explicit Impl(Localization::Config config)
        : config{std::move(config)}
        , logging{config.rclcpp}
        , context{config} {

        engine.setTransformationEpsilon(config.ndt_result_epsilon); // 收敛判定阈值
        engine.setStepSize(config.ndt_step_size);                   // More-Thuente 线搜索最大步长
        engine.setResolution(config.ndt_resolution);                // NDT 网格分辨率
        engine.setMaximumIterations(config.ndt_max_iterations);     // 最大迭代次数
    }

    auto start_collecting(std::chrono::seconds seconds) -> std::expected<void, std::string> {
        std::ignore = this;
        std::ignore = seconds;
        return {};
    }

    auto start_localizing(const Eigen::Isometry3d& initial_solution)
        -> std::future<std::expected<Eigen::Isometry3d, std::string>> {
        std::ignore = this;
        std::ignore = initial_solution;
        return {};
    }
};

Localization::Localization(Config config)
    : pimpl{std::make_unique<Impl>(std::move(config))} {}

Localization::~Localization() noexcept = default;

auto Localization::start_collecting(std::chrono::seconds seconds)
    -> std::expected<void, std::string> {
    return pimpl->start_collecting(seconds);
}

auto Localization::start_localizing(const Eigen::Isometry3d& initial_solution)
    -> std::future<std::expected<Eigen::Isometry3d, std::string>> {
    return pimpl->start_localizing(initial_solution);
}

} // namespace rmcs::navigation
