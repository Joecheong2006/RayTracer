#include "gpu/TBOStorage.h"
#include <glad/glad.h>

namespace gpu {
    TBOStorage::TBOStorage()
        : tboBuffer(nullptr, 0, GL_STATIC_DRAW, GL_R32F)
    {}

    void TBOStorage::upload(const gpu::Buffer &buffer) {
        tboBuffer.setBuffer(buffer.data(), buffer.size());
    }

    void TBOStorage::bindToUnit(int index) const {;
        tboBuffer.bindToUnit(index);
    }
}
