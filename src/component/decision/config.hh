#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace rmcs::navigation {

struct Config {
    using Point = std::pair<double, double>;

    std::uint16_t health_limit = 150;
    std::uint16_t health_ready = 350;
    std::uint16_t bullet_limit = 0;
    std::uint16_t bullet_ready = 0;

    Point home{0.0, 0.0};
    double cruise_interval = 0.0;
    std::unordered_map<std::string, std::vector<Point>> cruise_methods;

    Config() = default;

    explicit Config(const YAML::Node& config)
        : Config{config.as<Config>()} {}
};

} // namespace rmcs::navigation

namespace YAML {

template <>
struct convert<rmcs::navigation::Config> {
    static auto encode(const rmcs::navigation::Config& rhs) -> Node {
        auto node = Node{NodeType::Map};
        node["health_limit"] = rhs.health_limit;
        node["health_ready"] = rhs.health_ready;
        node["bullet_limit"] = rhs.bullet_limit;
        node["bullet_ready"] = rhs.bullet_ready;
        node["home"] = rhs.home;
        node["cruise_interval"] = rhs.cruise_interval;

        auto cruise_methods = Node{NodeType::Map};
        for (const auto& [name, route] : rhs.cruise_methods) {
            cruise_methods[name] = route;
        }
        node["cruise_methods"] = cruise_methods;
        return node;
    }

    static auto decode(const Node& node, rmcs::navigation::Config& rhs) -> bool {
        const auto decision = node["decision"] ? node["decision"] : node;
        if (decision.IsMap() == false) {
            return false;
        }

        if (const auto value = decision["health_limit"]; value) {
            rhs.health_limit = value.as<std::uint16_t>();
        }
        if (const auto value = decision["health_ready"]; value) {
            rhs.health_ready = value.as<std::uint16_t>();
        }
        if (const auto value = decision["bullet_limit"]; value) {
            rhs.bullet_limit = value.as<std::uint16_t>();
        }
        if (const auto value = decision["bullet_ready"]; value) {
            rhs.bullet_ready = value.as<std::uint16_t>();
        }
        if (const auto value = decision["home"]; value) {
            rhs.home = value.as<std::array<double, 2>>();
        }
        if (const auto value = decision["cruise_interval"]; value) {
            rhs.cruise_interval = value.as<double>();
        }

        rhs.cruise_methods.clear();
        if (const auto methods = decision["cruise_methods"]; methods && methods.IsMap()) {
            for (const auto& it : methods) {
                using namespace rmcs::navigation;
                rhs.cruise_methods.emplace(
                    it.first.as<std::string>(), it.second.as<std::vector<Config::Point>>());
            }
        }

        return true;
    }
};

} // namespace YAML
