#pragma once
#include <tuple>

namespace rmcs::navigation::util {

template <typename T>
auto tie(T& obj) {
    if constexpr (requires { obj.x, obj.y, obj.z, obj.w; }) {
        return std::tie(obj.x, obj.y, obj.z, obj.w);
    } else if constexpr (requires { obj.x, obj.y, obj.z; }) {
        return std::tie(obj.x, obj.y, obj.z);
    } else if constexpr (requires { obj.x, obj.y; }) {
        return std::tie(obj.x, obj.y);
    }
}

} // namespace rmcs::navigation::util
