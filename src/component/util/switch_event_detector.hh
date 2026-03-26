#pragma once

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include <chrono>

namespace rmcs::navigation {

class SwitchEventDetector {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration = Clock::duration;
    using SwitchInput = rmcs_executor::Component::InputInterface<rmcs_msgs::Switch>;

    explicit SwitchEventDetector(
        SwitchInput& input, Duration window = std::chrono::milliseconds(500))
        : input_{input}
        , trigger_window_{window} {}

    auto reset() noexcept -> void {
        stage_ = Stage::Idle;
        has_last_ = false;
    }

    auto spin(TimePoint now = Clock::now()) -> bool {
        if (!input_.ready()) {
            reset();
            return false;
        }

        const auto current = *input_;

        if (!has_last_) {
            last_ = current;
            has_last_ = true;
            return false;
        }

        if (stage_ == Stage::SeenMiddleToUp && now - seen_middle_to_up_at_ > trigger_window_) {
            stage_ = Stage::Idle;
        }

        auto triggered = false;
        if (last_ == rmcs_msgs::Switch::MIDDLE && current == rmcs_msgs::Switch::UP) {
            stage_ = Stage::SeenMiddleToUp;
            seen_middle_to_up_at_ = now;
        } else if (last_ == rmcs_msgs::Switch::UP && current == rmcs_msgs::Switch::MIDDLE) {
            if (stage_ == Stage::SeenMiddleToUp && now - seen_middle_to_up_at_ <= trigger_window_) {
                triggered = true;
            }
            stage_ = Stage::Idle;
        }

        last_ = current;
        return triggered;
    }

private:
    enum class Stage {
        Idle,
        SeenMiddleToUp,
    };

    SwitchInput& input_;
    Duration trigger_window_;

    Stage stage_ = Stage::Idle;
    bool has_last_ = false;
    rmcs_msgs::Switch last_ = rmcs_msgs::Switch::UNKNOWN;
    TimePoint seen_middle_to_up_at_{};
};

} // namespace rmcs::navigation
