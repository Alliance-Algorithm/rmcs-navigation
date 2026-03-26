#pragma once

#include <chrono>
#include <functional>
#include <utility>

namespace rmcs::navigation {

class TimedTask {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration = Clock::duration;
    using SpinFunction = std::function<void(Duration elapsed, Duration remaining, double progress)>;
    using FinishFunction = std::function<void()>;

    explicit TimedTask(Duration duration, SpinFunction on_spin, FinishFunction on_finish = {})
        : duration_{duration > Duration::zero() ? duration : Duration::zero()}
        , on_spin_{std::move(on_spin)}
        , on_finish_{std::move(on_finish)} {}

    auto start(TimePoint now = Clock::now()) noexcept -> void {
        start_time_ = now;
        end_time_ = now + duration_;
        active_ = duration_ > Duration::zero();
        finished_ = false;
    }

    auto stop() noexcept -> void {
        active_ = false;
        finished_ = true;
    }

    [[nodiscard]] auto running() const noexcept -> bool { return active_; }

    auto spin(TimePoint now = Clock::now()) -> bool {
        if (!active_) {
            return false;
        }

        if (now >= end_time_) {
            active_ = false;
            if (!finished_ && on_finish_) {
                on_finish_();
            }
            finished_ = true;
            return false;
        }

        auto elapsed = now - start_time_;
        if (elapsed < Duration::zero()) {
            elapsed = Duration::zero();
        }
        auto remaining = end_time_ - now;
        if (remaining < Duration::zero()) {
            remaining = Duration::zero();
        }

        auto progress = 0.0;
        auto total_seconds = std::chrono::duration<double>(duration_).count();
        if (total_seconds > 0.0) {
            progress = std::chrono::duration<double>(elapsed).count() / total_seconds;
            if (progress < 0.0) {
                progress = 0.0;
            } else if (progress > 1.0) {
                progress = 1.0;
            }
        }

        if (on_spin_) {
            on_spin_(elapsed, remaining, progress);
        }
        return true;
    }

private:
    Duration duration_{};
    SpinFunction on_spin_;
    FinishFunction on_finish_;

    bool active_ = false;
    bool finished_ = false;
    TimePoint start_time_;
    TimePoint end_time_;
};

} // namespace rmcs::navigation
