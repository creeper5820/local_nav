#pragma once

#include <iostream>

// syntactic sugar
namespace utility {

template <typename... Args>
inline void info(Args... args)
{
    ((std::cout << args), ...) << '\n';
}

}