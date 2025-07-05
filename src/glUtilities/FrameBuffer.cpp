#include "FrameBuffer.h"
#include "Texture2D.h"
#include "glad/glad.h"
#include "util.h"

namespace gl {
    FrameBuffer::FrameBuffer(int width, int height)
        : m_width(width), m_height(height)
    {
        glGenFramebuffers(1, &m_id);
        bind();
    }

    FrameBuffer::~FrameBuffer() {
        glDeleteFramebuffers(1, &m_id);
    }

    bool FrameBuffer::isCompleted() const {
        return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    }

    void FrameBuffer::genRenderBuffer(i32 internalFormat) {
        glGenRenderbuffers(1, &m_rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, m_rbo); 
        glRenderbufferStorage(GL_RENDERBUFFER, internalFormat, m_width, m_height);
    }

    void FrameBuffer::attachRenderBuffer(i32 attachment) const {
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, m_rbo);
    }

    void FrameBuffer::attachTexture(Texture2D &texture, i32 attachment) const {
        glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, texture.m_id, 0);
    }

    void FrameBuffer::bind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);
    }

    void FrameBuffer::unbind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

}
