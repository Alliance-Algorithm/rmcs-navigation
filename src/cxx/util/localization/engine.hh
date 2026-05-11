#pragma once
#include "cxx/util/pimpl.hh"

#include <cstdint>
#include <rclcpp/node.hpp>
#include <string>

namespace rmcs::navigation {

enum class RelocalizeState : std::uint8_t {
    IDLE = 0,
    IN_FLIGHT = 1,
    SUCCEEDED = 2,
    FAILED = 3,
};

enum class RelocalizeMode : std::uint8_t {
    Initial = 0,
    Local = 1,
    Wide = 2,
};

struct RelocalizeStatus {
    RelocalizeState state = RelocalizeState::IDLE;
    bool success = false;
    std::string message;
    double fitness_score = 0.0;
    double confidence = 0.0;
    double estimated_x = 0.0;
    double estimated_y = 0.0;
    double estimated_z = 0.0;
    double estimated_qx = 0.0;
    double estimated_qy = 0.0;
    double estimated_qz = 0.0;
    double estimated_qw = 1.0;
};

class Localization {
    RMCS_PIMPL_DEFINITION(Localization)

public:
    struct Config {
        rclcpp::Node& rclcpp;
        std::string service_name = "/rmcs_relocation/relocalize";
        double request_timeout_sec = 30.0;
    };

    explicit Localization(Config config);

    auto relocalize(RelocalizeMode mode, double x, double y, double yaw) -> bool;
    auto relocalize_status() const -> RelocalizeStatus;
};

} // namespace rmcs::navigation
