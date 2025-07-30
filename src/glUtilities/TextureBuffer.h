#pragma once
#include "../util.h"

namespace gl {
    class TextureBuffer {
    private:
        u32 m_tbo, m_tboTexture;
        i32 m_size, m_usage, m_internalFormat;

    public:
        TextureBuffer(const void *data, i32 size, i32 usage, i32 internalFormat);
        ~TextureBuffer();

        void setBuffer(const void *data, i32 size, i32 usage = 0, i32 internalFormat = 0);
        void updateBuffer(const void *data, i32 offset, i32 size);

        void bind(u32 slot) const;
        void unbind() const;

        i32 getSize() const { return m_size; }
        i32 getInternalFormat() const { return m_internalFormat; }

    };

}
