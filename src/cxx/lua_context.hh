#pragma once
#include "cxx/util/localization/engine.hh"
#include "cxx/util/pimpl.hh"

#include <functional>
#include <string>

#include <rclcpp/node.hpp>
#include <sol/sol.hpp>

namespace rmcs::navigation::details {

class LuaContext {
    RMCS_PIMPL_DEFINITION(LuaContext)

public:
    struct Api {
        std::function<void(bool)> update_enable_control;
        std::function<void(double, double)> send_target;
        std::function<void(bool)> switch_topic_forward;
        std::function<void(double)> update_gimbal_direction;
        std::function<void(const std::string&)> switch_controller;
        std::function<bool(double, double, double)> relocalize_initial;
        std::function<bool(double, double, double)> relocalize_local;
        std::function<bool(double, double, double)> relocalize_wide;
        std::function<RelocalizeStatus()> relocalize_status;
    };

    explicit LuaContext(rclcpp::Node& node) noexcept;

    auto init(Api api) -> void;
    auto tick() -> void;
    auto blackboard() -> sol::table&;
};

} // namespace rmcs::navigation::details
