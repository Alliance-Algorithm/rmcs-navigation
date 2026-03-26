#pragma once

#include "component/util/timed_task.hh"

#include <chrono>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace rmcs::navigation {

class NodTaskQueue {
public:
    using Clock = TimedTask::Clock;
    using TimePoint = TimedTask::TimePoint;
    using Duration = TimedTask::Duration;
    using SetPitchVelocity = std::function<void(double)>;

    explicit NodTaskQueue(SetPitchVelocity set_pitch_velocity)
        : set_pitch_velocity_{std::move(set_pitch_velocity)} {}

    auto push_task(
        Duration duration, TimedTask::SpinFunction on_spin = {},
        TimedTask::FinishFunction on_finish = {}) -> void {
        if (duration <= Duration::zero()) {
            return;
        }

        tasks_.emplace_back(duration, std::move(on_spin), std::move(on_finish));
    }

    auto push_delay(Duration duration) -> void {
        push_task(
            duration,
            []([[maybe_unused]] Duration, [[maybe_unused]] Duration, [[maybe_unused]] double) {});
    }

    auto nod_once(double speed_abs, Duration half_period) -> void {
        if (!set_pitch_velocity_) {
            return;
        }
        if (speed_abs <= 0.0 || half_period <= Duration::zero()) {
            return;
        }

        push_task(
            half_period,
            [this, speed_abs](
                [[maybe_unused]] Duration, [[maybe_unused]] Duration, [[maybe_unused]] double) {
                set_pitch_velocity_(+speed_abs);
            },
            [this] { set_pitch_velocity_(0.0); });

        push_task(
            half_period,
            [this, speed_abs](
                [[maybe_unused]] Duration, [[maybe_unused]] Duration, [[maybe_unused]] double) {
                set_pitch_velocity_(-speed_abs);
            },
            [this] { set_pitch_velocity_(0.0); });
    }

    auto spin(TimePoint now = Clock::now()) -> void {
        if (!set_pitch_velocity_) {
            return;
        }

        if (empty()) {
            set_pitch_velocity_(0.0);
            return;
        }

        auto& task = tasks_.at(current_index_);
        if (!current_started_) {
            task.start(now);
            current_started_ = true;
        }

        auto running = task.spin(now);
        if (running) {
            return;
        }

        current_index_++;
        current_started_ = false;

        if (current_index_ >= tasks_.size()) {
            clear();
        }
    }

    auto clear() -> void {
        tasks_.clear();
        current_index_ = 0;
        current_started_ = false;
        if (set_pitch_velocity_) {
            set_pitch_velocity_(0.0);
        }
    }

    [[nodiscard]] auto empty() const noexcept -> bool { return current_index_ >= tasks_.size(); }

private:
    SetPitchVelocity set_pitch_velocity_;
    std::vector<TimedTask> tasks_;
    std::size_t current_index_ = 0;
    bool current_started_ = false;
};

} // namespace rmcs::navigation
