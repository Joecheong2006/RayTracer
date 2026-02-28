#pragma once
#include "gpu/Storage.h"
#include "glUtilities/TextureBuffer.h"

namespace gpu {
    struct TBOStorage : public gpu::Storage {
        gl::TextureBuffer tboBuffer;

        TBOStorage();
        virtual void upload(const gpu::Buffer &buffer) override;
        virtual void bindToUnit(int index) const override;
    };
}

