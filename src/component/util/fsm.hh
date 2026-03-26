#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace rmcs::navigation {

template <typename state_type>
class Fsm {
    static_assert(std::is_enum_v<state_type>, "state_type must be an enum");
    static_assert(requires { state_type::END; }, "state_type::END is required");

private:
    struct IState {
        virtual auto on_begin() -> void = 0;
        virtual auto on_event() -> state_type = 0;
        virtual ~IState() = default;
    };
    using StateUnique = std::unique_ptr<IState>;
    using StateKey = state_type;
    using StateRaw = IState*;

    std::array<StateUnique, std::to_underlying(StateKey::END)> states_map;
    StateKey current_state = state_type::END;
    StateRaw current_event = nullptr;

public:
    explicit Fsm(state_type start_state) noexcept
        : current_state{start_state}
        , current_event{nullptr} {
        assert(start_state != StateKey::END && "start_state cannot be END");
    }

    template <typename on_begin_type, typename on_event_type>
    auto use(state_type state, on_begin_type&& on_begin, on_event_type&& on_event) {
        static_assert(
            std::is_invocable_r_v<void, on_begin_type>, "on_begin must be callable returning void");
        static_assert(
            std::is_invocable_r_v<state_type, on_event_type>,
            "on_event must be callable returning state_type");

        struct State final : IState {
            [[no_unique_address]] std::decay_t<on_begin_type> impl_on_begin;
            [[no_unique_address]] std::decay_t<on_event_type> impl_on_event;

            explicit State(on_begin_type&& on_begin, on_event_type&& on_event)
                : impl_on_begin{std::forward<on_begin_type>(on_begin)}
                , impl_on_event{std::forward<on_event_type>(on_event)} {}

            ~State() override = default;

            auto on_begin() -> void override { impl_on_begin(); }
            auto on_event() -> state_type override { return impl_on_event(); }
        };
        states_map.at(std::to_underlying(state)) = std::make_unique<State>(
            std::forward<on_begin_type>(on_begin), std::forward<on_event_type>(on_event));
    }

    template <state_type state>
    auto use(auto&& on_begin, auto&& on_event) {
        using _1 = decltype(on_begin);
        using _2 = decltype(on_event);
        use(state, std::forward<_1>(on_begin), std::forward<_2>(on_event));
    }

    template <state_type state>
    auto use(auto&& on_event) {
        using _1 = decltype(on_event);
        use(state, [] {}, std::forward<_1>(on_event));
    }

    [[nodiscard]] auto registered_count() const noexcept -> std::size_t {
        return std::ranges::count_if(states_map, [](const auto& event) { return bool{event}; });
    }
    [[nodiscard]] auto fully_registered() const noexcept -> bool {
        return registered_count() == states_map.size();
    }

    auto start_on(state_type state) {
        current_state = state;
        current_event = nullptr;
    }

    /// @return: Is Stop
    auto spin_once() -> bool {
        if (current_event == nullptr) {
            current_event = states_map.at(std::to_underlying(current_state)).get();
            current_event->on_begin();
        }

        auto next = current_event->on_event();
        if (next != current_state) {
            current_state = next;
            current_event = nullptr;
        }
        return next == StateKey::END;
    }
};

} // namespace rmcs::navigation
