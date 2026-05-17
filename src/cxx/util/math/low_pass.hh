#include <chrono>
#include <cmath>

namespace rmcs::navigation {

template <double kVar>
struct Tau {
    static constexpr auto var = kVar;
    static_assert(var > 0, "Tau should be larger then zero");
};

template <typename T>
struct LowPassFilter {
    using clock = std::chrono::steady_clock;

    const double tau;

    clock::time_point last_timestamp = clock::now();
    T current;

    template <double var>
    explicit LowPassFilter(Tau<var>, T init)
        : current{init}
        , tau{var} {}

    auto clean(T var) noexcept {
        last_timestamp = clock::now();
        current = var;
    }

    auto update(T var) noexcept -> T {
        const auto now = clock::now();
        const auto dt = std::chrono::duration<double>{now - last_timestamp}.count();

        if (dt > 1.0) {
            clean(var);
            return var;
        }

        const auto alpha = 1.0 - std::exp(-dt / tau);
        current = T{current * (1.0 - alpha)} + T{var * alpha};

        last_timestamp = now;
        return current;
    }
};

} // namespace rmcs::navigation
