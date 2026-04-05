#pragma once

#if defined(__has_include)
# if __has_include(<format>)
#  include <format>
#  if defined(__cpp_lib_format) && (__cpp_lib_format >= 201907L)
#   define RMCS_NAVIGATION_HAS_STD_FORMAT 1
#  endif
# endif
#endif

#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

#if defined(RMCS_NAVIGATION_HAS_STD_FORMAT)
namespace rmcs::navigation::detail {

constexpr auto to_string(rmcs_msgs::GameStage stage) noexcept {
    switch (stage) {
    case rmcs_msgs::GameStage::NOT_START: return "NOT_START";
    case rmcs_msgs::GameStage::PREPARATION: return "PREPARATION";
    case rmcs_msgs::GameStage::REFEREE_CHECK: return "REFEREE_CHECK";
    case rmcs_msgs::GameStage::COUNTDOWN: return "COUNTDOWN";
    case rmcs_msgs::GameStage::STARTED: return "STARTED";
    case rmcs_msgs::GameStage::SETTLING: return "SETTLING";
    case rmcs_msgs::GameStage::UNKNOWN: return "UNKNOWN";
    }
    return "UNREACHABLE";
}
constexpr auto to_string(rmcs_msgs::Switch value) noexcept -> const char* {
    switch (value) {
    case rmcs_msgs::Switch::UNKNOWN: return "UNKNOWN";
    case rmcs_msgs::Switch::UP: return "UP";
    case rmcs_msgs::Switch::DOWN: return "DOWN";
    case rmcs_msgs::Switch::MIDDLE: return "MIDDLE";
    }
    return "UNREACHABLE";
}
constexpr auto to_string(rmcs_msgs::ArmorID id) noexcept -> const char* {
    switch (id) {
    case rmcs_msgs::ArmorID::Unknown: return "Unknown";
    case rmcs_msgs::ArmorID::Hero: return "Hero";
    case rmcs_msgs::ArmorID::Engineer: return "Engineer";
    case rmcs_msgs::ArmorID::InfantryIII: return "InfantryIII";
    case rmcs_msgs::ArmorID::InfantryIV: return "InfantryIV";
    case rmcs_msgs::ArmorID::InfantryV: return "InfantryV";
    case rmcs_msgs::ArmorID::Aerial: return "Aerial";
    case rmcs_msgs::ArmorID::Sentry: return "Sentry";
    case rmcs_msgs::ArmorID::Dart: return "Dart";
    case rmcs_msgs::ArmorID::Radar: return "Radar";
    case rmcs_msgs::ArmorID::Outpost: return "Outpost";
    case rmcs_msgs::ArmorID::Base: return "Base";
    }
    return "UNREACHABLE";
}
constexpr auto to_string(rmcs_msgs::RobotId id) noexcept -> const char* {
    switch (static_cast<rmcs_msgs::RobotId::Value>(id)) {
    case rmcs_msgs::RobotId::UNKNOWN: return "UNKNOWN";
    case rmcs_msgs::RobotId::RED_HERO: return "RED_HERO";
    case rmcs_msgs::RobotId::RED_ENGINEER: return "RED_ENGINEER";
    case rmcs_msgs::RobotId::RED_INFANTRY_III: return "RED_INFANTRY_III";
    case rmcs_msgs::RobotId::RED_INFANTRY_IV: return "RED_INFANTRY_IV";
    case rmcs_msgs::RobotId::RED_INFANTRY_V: return "RED_INFANTRY_V";
    case rmcs_msgs::RobotId::RED_AERIAL: return "RED_AERIAL";
    case rmcs_msgs::RobotId::RED_SENTRY: return "RED_SENTRY";
    case rmcs_msgs::RobotId::RED_DART: return "RED_DART";
    case rmcs_msgs::RobotId::RED_RADAR: return "RED_RADAR";
    case rmcs_msgs::RobotId::RED_OUTPOST: return "RED_OUTPOST";
    case rmcs_msgs::RobotId::RED_BASE: return "RED_BASE";
    case rmcs_msgs::RobotId::BLUE_HERO: return "BLUE_HERO";
    case rmcs_msgs::RobotId::BLUE_ENGINEER: return "BLUE_ENGINEER";
    case rmcs_msgs::RobotId::BLUE_INFANTRY_III: return "BLUE_INFANTRY_III";
    case rmcs_msgs::RobotId::BLUE_INFANTRY_IV: return "BLUE_INFANTRY_IV";
    case rmcs_msgs::RobotId::BLUE_INFANTRY_V: return "BLUE_INFANTRY_V";
    case rmcs_msgs::RobotId::BLUE_AERIAL: return "BLUE_AERIAL";
    case rmcs_msgs::RobotId::BLUE_SENTRY: return "BLUE_SENTRY";
    case rmcs_msgs::RobotId::BLUE_DART: return "BLUE_DART";
    case rmcs_msgs::RobotId::BLUE_RADAR: return "BLUE_RADAR";
    case rmcs_msgs::RobotId::BLUE_OUTPOST: return "BLUE_OUTPOST";
    case rmcs_msgs::RobotId::BLUE_BASE: return "BLUE_BASE";
    }
    return "UNREACHABLE";
}

} // namespace rmcs::navigation::detail

template <>
struct std::formatter<rmcs_msgs::GameStage> : std::formatter<const char*> {
    auto format(rmcs_msgs::GameStage stage, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(stage), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::ArmorID> : std::formatter<const char*> {
    auto format(rmcs_msgs::ArmorID id, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs::navigation::detail::to_string(id), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::RobotId> : std::formatter<const char*> {
    auto format(rmcs_msgs::RobotId id, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs::navigation::detail::to_string(id), ctx);
    }
};
#endif
