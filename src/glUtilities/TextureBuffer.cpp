#include "glUtilities/TextureBuffer.h"
#include "glad/glad.h"
#include "util.h"

namespace gl {
    TextureBuffer::TextureBuffer(const void *data, i64 size, i32 usage, i32 internalFormat)
        : m_size(size), m_usage(usage), m_internalFormat(internalFormat)
    {
        GLCALL(glGenBuffers(1, &m_tbo));
        GLCALL(glBindBuffer(GL_TEXTURE_BUFFER, m_tbo));
        GLCALL(glBufferData(GL_TEXTURE_BUFFER, size, data, usage));

        GLCALL(glGenTextures(1, &m_tboTexture));
        GLCALL(glBindTexture(GL_TEXTURE_BUFFER, m_tboTexture));
        GLCALL(glTexBuffer(GL_TEXTURE_BUFFER, internalFormat, m_tbo););
    }

    TextureBuffer::~TextureBuffer() {
        GLCALL(glDeleteBuffers(1, &m_tbo));
        GLCALL(glDeleteTextures(1, &m_tboTexture));
    }

    void TextureBuffer::setBuffer(const void *data, i64 size, i32 usage, i32 internalFormat) {
        GLCALL(glBindBuffer(GL_TEXTURE_BUFFER, m_tbo));

        if (usage) {
            m_usage = usage;
        }

        GLCALL(glBufferData(GL_TEXTURE_BUFFER, size, data, m_usage));

        if (internalFormat) {
            m_internalFormat = internalFormat;
        }

        GLCALL(glTexBuffer(GL_TEXTURE_BUFFER, m_internalFormat, m_tbo););
        m_size = size;
    }

    void TextureBuffer::updateBuffer(const void *data, i32 offset, i64 size) {
        GLCALL(glBindBuffer(GL_TEXTURE_BUFFER, m_tbo));
        GLCALL(glBufferSubData(GL_TEXTURE_BUFFER, offset, size, data));
        m_size = size;
    }

    void TextureBuffer::unbind() const {
        GLCALL(glBindBuffer(GL_TEXTURE_BUFFER, 0));
        GLCALL(glBindTexture(GL_TEXTURE_BUFFER, 0));
    }

    void TextureBuffer::bindToUnit(u32 unit) const {
        GLCALL(glActiveTexture(GL_TEXTURE0 + unit));
        GLCALL(glBindTexture(GL_TEXTURE_BUFFER, m_tboTexture));
    }

}
