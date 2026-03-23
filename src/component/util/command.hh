#pragma once
#include <string_view>
#include <yaml-cpp/yaml.h>

namespace rmcs_navigation {

enum class GimbalMode {
    Unknown,
    Forward,
    Scanning,
};
auto from_string(std::string_view string, GimbalMode& mode) {
    if (string == "Forward")
        mode = GimbalMode::Forward;
    else if (string == "Scanning")
        mode = GimbalMode::Scanning;
    else
        mode = GimbalMode::Unknown;
}

struct Command {
    GimbalMode gimbal_mode;

    explicit Command(const YAML::Node& yaml) {}
};

}; // namespace rmcs_navigation
