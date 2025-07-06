#pragma once

#include "glad/glad.h"
#include "../util.h"

namespace gl {
    class Texture2D;
    class Framebuffer {
    private:
        u32 m_id, m_rbo;

    public:
        Framebuffer();
        ~Framebuffer();

        bool isCompleted() const;

        void genRenderBuffer(i32 width, i32 height, i32 internalFormat);
        void attachRenderBuffer(i32 attachment) const;
        void attachTexture(Texture2D &texture, i32 attachment = GL_COLOR_ATTACHMENT0) const;

        void bind() const;
        void unbind() const;

    };

}
