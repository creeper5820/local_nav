#pragma once

#include <cstdint>

namespace type {
enum class NodeType {
    NONE,
    BLOCK,
    USED,
    AVAILABLE,
};

struct Node {
    int x;
    int y;
    int8_t value;
    NodeType type;
};
} // namespace type