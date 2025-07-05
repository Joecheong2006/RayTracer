#pragma once

#include "../util.h"

namespace gl {
    class VertexBufferLayout;
    class VertexArray {
    private:
        u32 m_id;

    public:
        VertexArray();
        ~VertexArray();

        void applyBufferLayout(const VertexBufferLayout &layout);
        void bind() const;
        void unbind() const;
    };

}
