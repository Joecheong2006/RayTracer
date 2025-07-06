#include "Framebuffer.h"
#include "Texture2D.h"
#include "glad/glad.h"
#include "util.h"

namespace gl {
    Framebuffer::Framebuffer(int width, int height)
        : m_width(width), m_height(height)
    {
        glGenFramebuffers(1, &m_id);
        bind();
    }

    Framebuffer::~Framebuffer() {
        glDeleteFramebuffers(1, &m_id);
    }

    bool Framebuffer::isCompleted() const {
        return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    }

    void Framebuffer::genRenderBuffer(i32 internalFormat) {
        glGenRenderbuffers(1, &m_rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, m_rbo); 
        glRenderbufferStorage(GL_RENDERBUFFER, internalFormat, m_width, m_height);
    }

    void Framebuffer::attachRenderBuffer(i32 attachment) const {
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, m_rbo);
    }

    void Framebuffer::attachTexture(Texture2D &texture, i32 attachment) const {
        glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, texture.m_id, 0);
    }

    void Framebuffer::bind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);
    }

    void Framebuffer::unbind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

}
