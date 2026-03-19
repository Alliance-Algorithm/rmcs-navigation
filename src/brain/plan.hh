#pragma once

#include "util/pimpl.hh"
#include <yaml-cpp/yaml.h>

namespace rmcs {

struct PlanBox final {
    RMCS_PIMPL_DEFINITION(PlanBox)

public:
    struct Information {
        double current_x = 0;
        double current_y = 0;

        std::uint16_t health = 0;
        std::uint16_t bullet = 0;
    };

    template <std::invocable<Information&> F>
    auto update_information(F&& function) noexcept -> void {
        std::forward<F>(function)(information_());
        do_plan_();
    }

    auto goal_position() noexcept -> std::tuple<double, double>;

    auto set_rule(const YAML::Node& rule) -> void;
    auto get_rule() const -> std::string;

private:
    auto do_plan_() noexcept -> void;
    auto information_() noexcept -> Information&;
};

} // namespace rmcs
