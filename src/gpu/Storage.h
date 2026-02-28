#pragma once
#include "Buffer.h"

namespace gpu {
    struct Storage {
        virtual ~Storage() = default;
        virtual void upload(const Buffer&) = 0;
        virtual void bindToUnit(i32 index) const = 0;
    };
}
