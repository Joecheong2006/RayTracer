#pragma once
#include "gpu/Buffer.h"

namespace gpu {
    struct Serializable {
        virtual ~Serializable() = default;
        virtual void serialize(Buffer&) const = 0;
    };
}

