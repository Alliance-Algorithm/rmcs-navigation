#include "util/fsm.hh"

#include <print>
#include <thread>

auto main() -> int {
    std::println("Hello World");

    enum class Status { A, B, C, END };

    auto fsm = rmcs::Fsm{Status::A};
    auto b_event_count = 0;
    auto b_event_limit = 5;

    fsm.use<Status::A>(
        [] { std::println("[A] begin"); },
        [] {
            std::println("[A] event");
            return Status::B;
        });
    fsm.use<Status::B>(
        [] { std::println("[B] begin"); },
        [&] {
            std::println("[B] event");
            if (b_event_count++ == b_event_limit) {
                return Status::C;
            }
            return Status::B;
        });
    fsm.use<Status::C>(
        [] { std::println("[C] begin"); },
        [] {
            std::println("[C] event");
            return Status::END;
        });

    if (!fsm.fully_registered()) {
        std::println("Size: {}", fsm.registered_count());
        throw std::runtime_error{"Fsm must be fully registered"};
    }

    for (;;) {
        if (fsm.spin_once())
            break;

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(200ms);
    }
}
