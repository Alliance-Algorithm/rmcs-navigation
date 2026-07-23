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
    sol::table lua_api;
    sol::table lua_blackboard;
    sol::protected_function lua_on_init;
    sol::protected_function lua_on_tick;
    sol::protected_function lua_on_exit;
    bool endpoint_started = false;

    explicit Impl(rclcpp::Node& node)
        : node{node} {
        bootstrap();
    }

    template <typename T>
    auto unwrap_sol(T result, std::string_view message) -> T {
        if (!result.valid()) {
            auto error = result.template get<sol::error>();
            logging.fuck("\n{}", error.what());
            throw std::runtime_error(std::string{message});
        }
        return result;
    }

    auto bootstrap() -> void {
        lua = std::make_unique<sol::state>();
        lua->open_libraries(
            sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::os, sol::lib::package,
            sol::lib::string, sol::lib::table, sol::lib::debug, sol::lib::io);

        auto package_root =
            std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs-navigation")};
        auto lua_root = package_root / "lua";
        auto package = (*lua)["package"].get<sol::table>();
        auto package_path = package["path"].get_or(std::string{});
        package["path"] = std::format(
            "{};{}/?.lua;{}/?/init.lua", package_path, lua_root.string(), lua_root.string());

        auto api_result = unwrap_sol(
            lua->safe_script("return require('api')", sol::script_pass_on_error),
            "failed to get lua api");

        lua_api = api_result.get<sol::table>();
        lua_api.set_function(
            "info", [this](const std::string& text) { logging.info("Lua: {}", text); });
        lua_api.set_function(
            "warn", [this](const std::string& text) { logging.warn("Lua: {}", text); });
        lua_api.set_function(
            "fuck", [this](const std::string& text) { logging.fuck("Lua: {}", text); });
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

    auto ensure_started() -> void {
        if (endpoint_started)
            return;
        endpoint_started = true;

        make_option_injection();

        if (!node.has_parameter("endpoint")) {
            logging.fuck("param [ {} ] for {} is needed", "endpoint", node.get_name());
            throw std::runtime_error{"lack of param"};
        }
        auto endpoint = node.get_parameter_or<std::string>("endpoint", std::string{});
        auto required = std::format("require('endpoint.{}')", endpoint);
        unwrap_sol(
            lua->safe_script(required, sol::script_pass_on_error), "failed to load lua main");

        lua_blackboard = (*lua)["blackboard"];
        lua_on_init = (*lua)["on_init"];
        lua_on_tick = (*lua)["on_tick"];

        const auto situation = std::array{
            lua_on_init.valid(),
            lua_on_tick.valid(),
        };
        if (!std::ranges::all_of(situation, std::identity{}))
            throw std::runtime_error("lua main must define on_init() and on_tick()");

        lua_on_exit = (*lua)["on_exit"];
        if (lua_on_exit == sol::lua_nil) {
            lua_on_exit = lua->safe_script("return function() end", sol::script_pass_on_error);
            logging.warn("lua endpoint does not define optional on_exit(), fallback to no-op");
        }

        unwrap_sol(lua_on_init(), "lua on_init failed");
        logging.info("Lua resource is loaded successfully with endpoint {}", endpoint);
    }

    auto tick() -> void {
        ensure_started();
        unwrap_sol(lua_on_tick(), "lua on_tick failed");
    }

    auto blackboard() -> sol::table& {
        ensure_started();
        return lua_blackboard;
    }
};

LuaContext::LuaContext(rclcpp::Node& node)
    : pimpl{std::make_unique<Impl>(node)} {}

LuaContext::~LuaContext() noexcept = default;

auto LuaContext::api() -> sol::table& { return pimpl->lua_api; }

auto LuaContext::tick() -> void { pimpl->tick(); }

auto LuaContext::blackboard() -> sol::table& { return pimpl->blackboard(); }

} // namespace rmcs::navigation::details
