#include "cxx/lua_context.hh"
#include "cxx/util/node_mixin.hh"

#include <filesystem>
#include <map>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter.hpp>

namespace rmcs::navigation::details {

struct LuaContext::Impl {
    rclcpp::Node& node;
    NodeWrap<rclcpp::Node> logging{node};

    std::unique_ptr<sol::state> lua;
    sol::table lua_blackboard;
    sol::protected_function lua_on_init;
    sol::protected_function lua_on_tick;
    sol::protected_function lua_on_exit;

    explicit Impl(rclcpp::Node& node)
        : node{node} {}

    template <typename T>
    auto unwrap_sol(T result, std::string_view message) -> T {
        if (!result.valid()) {
            auto error = result.template get<sol::error>();
            logging.fuck("\n{}", error.what());
            throw std::runtime_error(std::string{message});
        }
        return result;
    }

    auto make_api_injection(LuaContext::Api impl) -> void {
        auto api_result = unwrap_sol(
            lua->safe_script("return require('api')", sol::script_pass_on_error),
            "failed to get lua api");

        auto api = api_result.get<sol::table>();
        api.set_function(
            "info", [this](const std::string& text) { logging.info("Lua: {}", text); });
        api.set_function(
            "warn", [this](const std::string& text) { logging.warn("Lua: {}", text); });
        api.set_function(
            "fuck", [this](const std::string& text) { logging.fuck("Lua: {}", text); });
        api.set_function("update_enable_control", std::move(impl.update_enable_control));
        api.set_function("send_target", std::move(impl.send_target));
        api.set_function("switch_topic_forward", std::move(impl.switch_topic_forward));
        api.set_function("update_gimbal_direction", std::move(impl.update_gimbal_direction));
        api.set_function("update_gimbal_dominator", std::move(impl.update_gimbal_dominator));
        api.set_function("switch_motion_mode", std::move(impl.switch_motion_mode));
        api.set_function("update_under_attack", std::move(impl.update_under_attack));
    }

    auto make_option_injection() -> void {
        auto option_result = unwrap_sol(
            lua->safe_script("return require('option')", sol::script_pass_on_error),
            "failed to get lua option");

        auto option = option_result.get<sol::table>();

        auto parameters = std::map<std::string, rclcpp::Parameter>{};
        node.get_parameters("", parameters);

        auto to_lua_array = [this]<typename T>(const std::vector<T>& values) {
            auto array = lua->create_table(static_cast<int>(values.size()), 0);
            for (auto index = std::size_t{0}; index < values.size(); ++index)
                array[index + 1] = values[index];

            return array;
        };

        for (const auto& [name, parameter] : parameters) {
            switch (parameter.get_type()) {
            case rclcpp::PARAMETER_BOOL: option[name] = parameter.as_bool(); break;
            case rclcpp::PARAMETER_INTEGER: option[name] = parameter.as_int(); break;
            case rclcpp::PARAMETER_DOUBLE: option[name] = parameter.as_double(); break;
            case rclcpp::PARAMETER_STRING: option[name] = parameter.as_string(); break;
            case rclcpp::PARAMETER_BOOL_ARRAY:
                option[name] = to_lua_array(parameter.as_bool_array());
                break;
            case rclcpp::PARAMETER_INTEGER_ARRAY:
                option[name] = to_lua_array(parameter.as_integer_array());
                break;
            case rclcpp::PARAMETER_DOUBLE_ARRAY:
                option[name] = to_lua_array(parameter.as_double_array());
                break;
            case rclcpp::PARAMETER_STRING_ARRAY:
                option[name] = to_lua_array(parameter.as_string_array());
                break;
            default: option[name] = sol::lua_nil; break;
            }
        }

        logging.info("injected {} ros parameters into lua option", parameters.size());
    }
};

LuaContext::LuaContext(rclcpp::Node& node) noexcept
    : pimpl{std::make_unique<Impl>(node)} {}

LuaContext::~LuaContext() noexcept = default;

auto LuaContext::init(Api api) -> void {
    pimpl->lua = std::make_unique<sol::state>();
    pimpl->lua->open_libraries(
        sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::os, sol::lib::package,
        sol::lib::string, sol::lib::table, sol::lib::debug, sol::lib::io);

    auto package_root =
        std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs-navigation")};
    auto lua_root = package_root / "lua";
    auto package = (*pimpl->lua)["package"].get<sol::table>();
    auto package_path = package["path"].get_or(std::string{});
    package["path"] = std::format(
        "{};{}/?.lua;{}/?/init.lua", package_path, lua_root.string(), lua_root.string());

    pimpl->make_api_injection(std::move(api));
    pimpl->make_option_injection();

    if (!pimpl->node.has_parameter("endpoint")) {
        pimpl->logging.fuck("param [ {} ] for {} is needed", "endpoint", pimpl->node.get_name());
        throw std::runtime_error{"lack of param"};
    }
    auto endpoint = pimpl->node.get_parameter_or<std::string>("endpoint", std::string{});
    auto required = std::format("require('endpoint.{}')", endpoint);
    pimpl->unwrap_sol(
        pimpl->lua->safe_script(required, sol::script_pass_on_error), "failed to load lua main");

    pimpl->lua_blackboard = (*pimpl->lua)["blackboard"];
    pimpl->lua_on_init = (*pimpl->lua)["on_init"];
    pimpl->lua_on_tick = (*pimpl->lua)["on_tick"];

    const auto situation = std::array{
        pimpl->lua_on_init.valid(),
        pimpl->lua_on_tick.valid(),
    };
    if (!std::ranges::all_of(situation, std::identity{}))
        throw std::runtime_error("lua main must define on_init() and on_tick()");

    pimpl->lua_on_exit = (*pimpl->lua)["on_exit"];
    if (pimpl->lua_on_exit == sol::lua_nil) {
        pimpl->lua_on_exit =
            pimpl->lua->safe_script("return function() end", sol::script_pass_on_error);
        pimpl->logging.warn("lua endpoint does not define optional on_exit(), fallback to no-op");
    }

    pimpl->unwrap_sol(pimpl->lua_on_init(), "lua on_init failed");
    pimpl->logging.info("Lua resource is loaded successfully with endpoint {}", endpoint);
}

auto LuaContext::tick() -> void { pimpl->unwrap_sol(pimpl->lua_on_tick(), "lua on_tick failed"); }

auto LuaContext::blackboard() -> sol::table& { return pimpl->lua_blackboard; }

} // namespace rmcs::navigation::details
