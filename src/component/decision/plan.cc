#include "component/decision/plan.hh"
#include "component/decision/config.hh"
#include "component/util/fsm.hh"
#include "component/util/rmcs_msgs_format.hh" // IWYU pragma: keep

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <expected>
#include <experimental/scope>
#include <format>
#include <functional>
#include <future>
#include <utility>

namespace rmcs_navigation {

constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

struct PlanBox::Impl {
    enum class Mode : std::uint8_t {
        Waiting,
        ToTheHome,
        Cruise,
        END,
    };

    std::function<void(const std::string&)> printer = [](const std::string&) {};
    std::string config_name;

    Information information;

    std::unique_ptr<Config> config;

    rmcs::Fsm<Mode> fsm{Mode::Waiting};

    double goal_x = kNan;
    double goal_y = kNan;

    bool rotate_chassis = false;
    bool gimbal_scanning = false;

    // CONTEXT
    rmcs_msgs::GameStage last_game_stage = rmcs_msgs::GameStage::UNKNOWN;

    std::size_t cruise_index = 0;
    std::string occupation_label = "occupation";
    std::string aggressive_label = "aggressive";

    bool cruise_point_reached = false;
    bool has_cruise_point_reached = false;
    std::chrono::steady_clock::time_point cruise_reached_timestamp;

    auto configure(const YAML::Node& _config) -> std::expected<void, std::string> {
        config = std::make_unique<Config>(_config);
        fsm.start_on(Mode::Waiting);

        auto& methods = config->cruise_methods;
        if (methods.empty()) {
            return std::unexpected{"Cruise is empty"};
        }
        if (!methods.contains(occupation_label)) {
            return std::unexpected{"Occupation method is not found"};
        }
        for (auto& [key, method] : methods) {
            if (method.empty()) {
                auto error = std::format("Empty method found: {}", key);
                return std::unexpected{error};
            }
        }
        return {};
    }

    // 首先确认 screen_label 对应的 screen 是否存在
    // 如果存在，则关闭该 screen，再重新拉起，运行的指令为下述的导航启动命令
    // 启动后，检查对应的 screen 容器是否存在，输出相应的 logging
    auto restart_navigation() const -> void {
        using namespace std::chrono_literals;

        auto source_command = std::string{"source /root/env_setup.bash"};
        auto launch_command = std::format(
            "ros2 launch rmcs-navigation online.launch.py config_name:={}", config_name);

        auto screen_label = std::string{"rmcs-navigation"};

        // 预定义指令
        auto check_cmd = std::format("screen -S {} -Q select . >/dev/null 2>&1", screen_label);
        auto stop_cmd = std::format("screen -S {} -X quit", screen_label);
        auto start_cmd = std::format(
            "screen -dmS {} bash -lc \"{} && {}\"", screen_label, source_command, launch_command);

        printer(std::format("[Navigation] Target: {}", screen_label));

        // 检查并清理旧进程
        if (std::system(check_cmd.c_str()) == 0) {
            printer("  -> Found existing session, terminating...");
            std::ignore = std::system(stop_cmd.c_str());
            std::this_thread::sleep_for(500ms);
        }

        // 执行启动
        printer("  -> Starting new navigation instance...");
        if (std::system(start_cmd.c_str()) != 0) {
            printer("  [!] Error: Failed to execute screen start command.");
            return;
        }

        // 验证启动结果
        std::this_thread::sleep_for(800ms);
        if (std::system(check_cmd.c_str()) == 0) {
            printer("  [✓] Navigation is now running in background.");
        } else {
            printer("  [✗] Critical: Screen died immediately after start.");
            printer(std::format("      Command tried: {}", launch_command));
        }
    }

    auto select_mode() const noexcept -> Mode {
        using namespace rmcs_msgs;
        auto game_stage = information.game_stage;

        if (game_stage == GameStage::UNKNOWN)
            return Mode::Waiting;

        // 进入倒计时时重启导航
        if (last_game_stage != GameStage::COUNTDOWN && game_stage == GameStage::COUNTDOWN) {
            std::ignore = std::async(std::launch::async, [this] {
                printer("Countdown, navigation restarting");
                restart_navigation();
            });
            return Mode::Waiting;
        }
        if (game_stage != GameStage::STARTED) {
            return Mode::Waiting;
        }

        // 优势不在我，回家补给
        {
            auto situations = std::array{
                information.health < config->health_limit,
                information.bullet < config->bullet_limit,
            };
            if (std::ranges::any_of(situations, std::identity{})) {
                return Mode::ToTheHome;
            }
        }

        // 优势在我，进行巡航进攻
        {
            auto situations = std::array{
                information.health >= config->health_ready,
                information.bullet >= config->bullet_ready,
            };
            if (std::ranges::all_of(situations, std::identity{})) {
                return Mode::Cruise;
            }
        }

        return Mode::Waiting;
    }

    Impl() noexcept {
        // 等待模式，啥也不做
        fsm.use<Mode::Waiting>(
            [this] {
                goal_x = kNan;
                goal_y = kNan;

                rotate_chassis = false;
                gimbal_scanning = false;

                printer("Start Waiting Mode");
            },
            [this] { return select_mode(); });

        // 回家
        fsm.use<Mode::ToTheHome>(
            [this] {
                auto [x, y] = config->home;
                goal_x = x;
                goal_y = y;

                rotate_chassis = true;
                gimbal_scanning = false;

                printer("Start ToTheHome Mode");
            },
            [this] { return select_mode(); });

        // 巡航模式，小陀螺旋转，云台扫描
        fsm.use<Mode::Cruise>(
            [this] {
                cruise_index = 0;
                cruise_reached_timestamp = std::chrono::steady_clock::now();
                rotate_chassis = false;
                gimbal_scanning = false;
                cruise_point_reached = false;

                printer("Start Cruise Mode");
            },
            [this] {
                auto& positions = config->cruise_methods[occupation_label];

                if (cruise_index >= positions.size()) {
                    cruise_index = 0;
                }

                auto update_goal = [this, &positions] {
                    auto [x, y] = positions.at(cruise_index);
                    goal_x = x;
                    goal_y = y;
                };
                update_goal();

                constexpr auto kTolerance = 0.1;
                auto reached = std::abs(information.current_x - goal_x) < kTolerance
                            && std::abs(information.current_y - goal_y) < kTolerance;

                if (reached && cruise_point_reached == false) {
                    cruise_point_reached = true;
                    cruise_reached_timestamp = std::chrono::steady_clock::now();
                }
                if (reached == false) {
                    cruise_point_reached = false;
                }

                // 自第一个巡航点开始，小陀螺不止
                if (has_cruise_point_reached) {
                    rotate_chassis = true;
                }

                if (cruise_point_reached) {
                    has_cruise_point_reached = true;
                    gimbal_scanning = true;

                    auto interval = config->cruise_interval;
                    auto elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - cruise_reached_timestamp);
                    if (elapsed.count() >= interval) {
                        cruise_index = (cruise_index + 1) % positions.size();
                        cruise_point_reached = false;
                        gimbal_scanning = false;
                        update_goal();
                        printer(std::format("Cruise point changed: ({}, {})", goal_x, goal_y));
                    }
                }

                return select_mode();
            });
    }

    auto do_plan() noexcept {
        auto game_stage = information.game_stage;
        if (game_stage != last_game_stage) {
            printer(std::format("Stage: {} -> {}", last_game_stage, game_stage));
        }

        fsm.spin_once();
        last_game_stage = information.game_stage;
    }

    auto goal_position() noexcept { return std::tuple{goal_x, goal_y}; }
};

PlanBox::PlanBox() noexcept
    : pimpl{std::make_unique<Impl>()} {}

PlanBox::~PlanBox() noexcept = default;

auto PlanBox::configure(const YAML::Node& config) -> std::expected<void, std::string> {
    return pimpl->configure(config);
}

auto PlanBox::set_printer(std::function<void(const std::string&)> printer) -> void {
    pimpl->printer = std::move(printer);
}

auto PlanBox::set_config_name(const std::string& name) -> void { pimpl->config_name = name; }

auto PlanBox::goal_position() noexcept -> std::tuple<double, double> {
    return pimpl->goal_position();
}

auto PlanBox::rotate_chassis() const noexcept -> bool { return pimpl->rotate_chassis; }

auto PlanBox::gimbal_scanning() const noexcept -> bool { return pimpl->gimbal_scanning; }

auto PlanBox::do_plan_() noexcept -> void { pimpl->do_plan(); }

auto PlanBox::information_() noexcept -> Information& { return pimpl->information; }

} // namespace rmcs_navigation
