#pragma once
#include "../util.h"

namespace gl {
    class VertexBuffer {
    private:
        u32 m_id;
        i32 m_size, m_usage;

    public:
        VertexBuffer(const void *data, i32 size, i32 usage);
        ~VertexBuffer();

        void setBuffer(const void *data, i32 size);

        void bind() const;
        void unbind() const;

    };

}
