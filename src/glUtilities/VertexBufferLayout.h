#pragma once

#include <vector>
#include "util.h"
#include "../util.h"
#include "glad/glad.h"

namespace gl {
    struct VertexBufferElement {
        u32 type;
        u32 count;
        u8 normalized;

        static u32 type_size(const u32 type)
        {
            switch(type)
            {
                case GL_FLOAT:          return sizeof(f32);
                case GL_UNSIGNED_INT:   return sizeof(u32);
                case GL_UNSIGNED_BYTE:  return sizeof(u8);
            }
            ASSERT(false);
            return 0;
        }
    };

    class VertexBufferLayout {
    private:
        std::vector<VertexBufferElement> m_elements;
        u32 m_stride;

    public:
        VertexBufferLayout()
            : m_stride(0) {}

        template<typename T>
        void add(u32 count)
        {
            (void)count;
            ASSERT(false);
        }

        inline const std::vector<VertexBufferElement> &elements() const { return m_elements; }
        inline u32 stride() const { return m_stride; }

    };

    template<>
    inline void VertexBufferLayout::add<f32>(u32 count)
    {
        m_elements.push_back({GL_FLOAT, count, GL_FALSE});
        m_stride += sizeof(f32) * count;
    }

    template<>
    inline void VertexBufferLayout::add<u32>(u32 count)
    {
        m_elements.push_back({GL_UNSIGNED_INT, count, GL_FALSE});
        m_stride += sizeof(u32) * count;
    }

    template<>
    inline void VertexBufferLayout::add<u8>(u32 count)
    {
        m_elements.push_back({GL_UNSIGNED_BYTE, count, GL_TRUE});
        m_stride += sizeof(u8) * count;
    }

}

