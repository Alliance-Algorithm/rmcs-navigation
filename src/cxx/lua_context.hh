#pragma once
#include "cxx/util/pimpl.hh"

#include <string>

#include <rclcpp/node.hpp>
#include <sol/sol.hpp>

namespace rmcs::navigation::details {

class LuaContext {
    RMCS_PIMPL_DEFINITION(LuaContext)

public:
    explicit LuaContext(rclcpp::Node& node);

    template <typename T>
    auto inject(std::string name, T&& func) -> void {
        api().set_function(std::move(name), std::forward<T>(func));
    }

    auto tick() -> void;
    auto blackboard() -> sol::table&;

private:
    auto api() -> sol::table&;
};

} // namespace rmcs::navigation::details
