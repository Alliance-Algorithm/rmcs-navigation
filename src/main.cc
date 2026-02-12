#include <string_view>

import app.test;
import util.rclcpp_node;

auto main() -> int {
    using namespace rmcs;

    auto app_name = std::string_view{"test"};
    auto app_node = util::RclcppNode{"Navigation"};

    auto always_run = [&]<class App> {
        try {
            auto app = App{};
            app.run();
        } catch (const std::exception& e) {
            app_node.error("Uncathed exception while app running, shutdown now");
            app_node.error("  {}", e.what());
            app_node.shutdown();
        }
    };

    if (app_name == "test") {
        app_node.info("Navigation is running with plan test");
        always_run.operator()<app::Test>();
    }

    // if (app_name == "rmuc")
    //     run<app::Rmuc>();

    // if (app_name == "rmul")
    //     run<app::Rmul>();
}
