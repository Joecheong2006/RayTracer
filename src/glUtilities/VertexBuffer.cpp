#include "VertexBuffer.h"
#include "util.h"
#include "glad/glad.h"

namespace gl {
    VertexBuffer::VertexBuffer(const void *data, i32 size, i32 usage)
        : m_size(size), m_usage(usage)
    {
        GLCALL(glGenBuffers(1, &m_id));
        setBuffer(data, size);
    }

    VertexBuffer::~VertexBuffer() {
        GLCALL(glDeleteBuffers(1, &m_id));
    }

    void VertexBuffer::setBuffer(const void *data, i32 size) {
        bind();
        GLCALL(glBufferData(GL_ARRAY_BUFFER, size, data, m_usage));
        m_size = size;
    }

    void VertexBuffer::bind() const {
        GLCALL(glBindBuffer(GL_ARRAY_BUFFER, m_id));
    }

    void VertexBuffer::unbind() const {
        GLCALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    }

}
