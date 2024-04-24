#pragma once

#include <iostream>

// syntactic sugar
namespace utility {

template <typename... Args>
concept info_requires = requires(Args... args) {
    ((std::cout << args), ...);
};

template <typename... Args>
    requires info_requires<Args...>
inline void info(Args... args)
{
    ((std::cout << args), ...) << '\n';
}

}