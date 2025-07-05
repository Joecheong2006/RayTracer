#include "IndexBuffer.h"
#include "util.h"
#include "glad/glad.h"

namespace gl {
    IndexBuffer::IndexBuffer(const u32 *data, i32 count, i32 usage)
        : m_count(count), m_usage(usage)
    {
        GLCALL(glGenBuffers(1, &m_id));
        setBuffer(data, count);
    }

    IndexBuffer::~IndexBuffer() {
        GLCALL(glDeleteBuffers(1, &m_id));
    }

    void IndexBuffer::setBuffer(const void *data, i32 count) {
        bind();
        GLCALL(glBufferData(GL_ELEMENT_ARRAY_BUFFER, 4 * count, data, m_usage));
        m_count = count;
    }

    void IndexBuffer::bind() const {
        GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_id));
    }

    void IndexBuffer::unbind() const {
        GLCALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    }

}
