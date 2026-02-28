#pragma once
#include "util.h"
#include "glm/glm.hpp"

namespace gpu {
    struct Buffer {
        virtual ~Buffer() = default;
        virtual void push(const glm::vec3 &v) = 0;
        virtual void push(const glm::ivec3 &iv) = 0;
        virtual void push(const glm::vec2 &v) = 0;
        virtual void push(f32 val) = 0;
        virtual void push(i32 val) = 0;
        virtual void push(u32 val) = 0;
        virtual void push(bool val) = 0;
        virtual void push(const char *data, size_t size) = 0;
        virtual const void* data() const = 0;
        virtual size_t size() const = 0;
        virtual void clear() = 0;
    };
}

