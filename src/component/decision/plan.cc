#include "component/decision/plan.hh"
#include "component/decision/config.hh"
#include "component/util/bool_edge_trigger.hh"
#include "component/util/delayed_task_queue.hh"
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
#include <utility>

#include <rclcpp/utilities.hpp>

namespace rmcs::navigation {

struct PlanBox::Impl {
    enum class Mode : std::uint8_t {
        WAITING,
        TO_HOME,
        CRUISE,
        ATTACK,
        RECOVERY,
        END,
    };

    Fsm<Mode> fsm{Mode::WAITING};

    Information new_info;
    Information old_info;
    Command command;

    std::function<void(const std::string&)> logging = [](const std::string&) {};
    std::unique_ptr<Config> config;

    // CONTEXT
    std::size_t cruise_index = 0;
    std::string occupation_label = "occupation";
    std::string aggressive_label = "aggressive";

    DelayedTaskQueue cruise_task_queue;
    BoolEdgeTrigger cruise_reached_edge{std::chrono::milliseconds{500}};

    bool need_recovery = false;

    auto configure(const YAML::Node& _config) -> std::expected<void, std::string> {
        config = std::make_unique<Config>(_config);
        fsm.start_on(Mode::WAITING);

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

    auto update_next_cruise_goal() {
        const auto& method = occupation_label;
        const auto& positions = config->cruise_methods[method];
        if (positions.empty()) {
            command.goal_x = kNan;
            command.goal_y = kNan;
            return;
        }

        if (cruise_index >= positions.size()) {
            cruise_index = 0;
        }

        auto [x, y] = positions.at(cruise_index);
        command.goal_x = x;
        command.goal_y = y;
    }

    auto select_mode() const noexcept -> Mode {
        using rmcs_msgs::GameStage;
        const auto& info = new_info;

        // 比赛未开始，等待
        if (info.game_stage != GameStage::STARTED) {
            return Mode::WAITING;
        }
        // 优势不在我，回家补给
        {
            auto situations = std::array{
                info.health <= config->health_limit,
                info.bullet <= config->bullet_limit,
            };
            if (std::ranges::any_of(situations, std::identity{})) {
                return need_recovery ? Mode::RECOVERY : Mode::TO_HOME;
            }
        }
        // 已确认打击目标，进入攻击姿态
        {
            if (std::isfinite(info.enemy_x) && std::isfinite(info.enemy_y)) {
                return Mode::ATTACK;
            }
        }
        // 优势在我，进行巡航进攻
        {
            auto situations = std::array{
                info.health >= config->health_limit,
                info.bullet >= config->bullet_limit,
            };
            if (std::ranges::all_of(situations, std::identity{})) {
                return Mode::CRUISE;
            }
        }

        return Mode::CRUISE;
    }

    Impl() noexcept {

        // 等待模式，啥也不做
        fsm.use<Mode::WAITING>(
            [this] {
                cruise_reached_edge.reset_edge_only();

                command.goal_x = kNan;
                command.goal_y = kNan;
                command.rotate_chassis = false;
                command.enable_autoaim = false;
                command.detect_targets = false;

                logging("Start Waiting Mode");
            },
            [this] { return select_mode(); });

        // 回家，你该补给了
        fsm.use<Mode::TO_HOME>(
            [this] {
                auto [x, y] = config->home;
                command.goal_x = x;
                command.goal_y = y;

                command.rotate_chassis = true;
                command.enable_autoaim = true;
                command.detect_targets = true;

                logging("Start ToTheHome Mode");
            },
            [this] { return select_mode(); });

        // 巡航模式，小陀螺旋转，云台扫描
        fsm.use<Mode::CRUISE>(
            [this] {
                cruise_index = 0;
                cruise_task_queue.clear();
                cruise_reached_edge.reset_edge_only();

                command.rotate_chassis = true;
                command.enable_autoaim = false;
                command.detect_targets = false;

                update_next_cruise_goal();
                logging("Start Cruise Mode");
            },
            [this] {
                constexpr auto kTolerance = 0.5;
                auto reached = std::abs(new_info.current_x - command.goal_x) < kTolerance
                            && std::abs(new_info.current_y - command.goal_y) < kTolerance;

                auto now = std::chrono::steady_clock::now();
                cruise_reached_edge.spin(reached, now);

                // 抵达巡航点时发布下一个延迟切换巡航点的任务
                if (cruise_reached_edge.consume_trigger()) {
                    const auto interval = std::chrono::duration_cast<DelayedTaskQueue::Duration>(
                        std::chrono::duration<double>{config->cruise_interval});

                    cruise_task_queue.clear();
                    cruise_task_queue.push(interval, [this] {
                        auto& positions = config->cruise_methods[occupation_label];
                        cruise_index = (cruise_index + 1) % positions.size();

                        update_next_cruise_goal();

                        const auto [x, y] = std::tie(command.goal_x, command.goal_y);
                        logging(std::format("Cruise point changed: ({}, {})", x, y));
                    });
                }
                cruise_task_queue.spin(now);

                // 只有在点内才开始扫描，切换巡航点不扫描
                const auto is_detecting = !cruise_task_queue.empty();
                command.detect_targets = is_detecting;

                // 自第一个巡航点开始到程序结束，小陀螺和扫描不止
                command.rotate_chassis = cruise_reached_edge.ever_triggered();

                return select_mode();
            });

        // 小陀螺站桩输出，拥有自瞄则拥有一千个人的力量
        fsm.use<Mode::ATTACK>(
            [this] {
                command.goal_x = kNan;
                command.goal_y = kNan;
                command.enable_autoaim = true;
                command.detect_targets = false;
                command.rotate_chassis = true;
            },
            [this] { return select_mode(); });

        // 回家，但是只回家，这个时候你什么也做不到
        fsm.use<Mode::RECOVERY>(
            [this] {
                const auto [x, y] = config->home;
                command.goal_x = x;
                command.goal_y = y;
                command.enable_autoaim = false;
                command.detect_targets = false;
                command.rotate_chassis = false;
            },
            [this] { return select_mode(); });

        if (!fsm.fully_registered()) {
            logging("Fsm is not fully registerd");
            rclcpp::shutdown();
        }
    }

    auto do_plan() noexcept {
        // 一些状态的切换沿检测
        auto new_stage = new_info.game_stage;
        auto old_stage = old_info.game_stage;
        if (new_stage != old_stage) {
            logging(std::format("Stage: {} -> {}", old_stage, new_stage));
        }

        // 血量变为 0 时，触发恢复模式
        if (old_info.health != 0 && new_info.health == 0)
            need_recovery = true;
        if (new_info.health >= config->health_ready)
            need_recovery = false;

        fsm.spin_once();

        old_info = new_info;
    }
};

PlanBox::PlanBox() noexcept
    : pimpl{std::make_unique<Impl>()} {}

PlanBox::~PlanBox() noexcept = default;

auto PlanBox::configure(const YAML::Node& config) -> std::expected<void, std::string> {
    return pimpl->configure(config);
}

auto PlanBox::set_logging(std::function<void(const std::string&)> logging) -> void {
    pimpl->logging = std::move(logging);
}

auto PlanBox::do_plan_() noexcept -> void { pimpl->do_plan(); }

auto PlanBox::information_() noexcept -> Information& { return pimpl->new_info; }

auto PlanBox::command_() noexcept -> const Command& { return pimpl->command; }

} // namespace rmcs::navigation
