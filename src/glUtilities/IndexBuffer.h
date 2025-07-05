#pragma once
#include "../util.h"

namespace gl {
    class IndexBuffer {
    private:
        u32 m_id;
        i32 m_count, m_usage;

    public:
        IndexBuffer(const u32 *data, i32 count, i32 usage);
        ~IndexBuffer();

        void setBuffer(const void *data, i32 count);

        void bind() const;
        void unbind() const;

        inline u32 count() const { return m_count; }
    };

}
