#pragma once

#include "glad/glad.h"
#include "ShaderProgram.h"

namespace gl {
    class Texture2D;
    class FrameBuffer {
    private:
        u32 m_id, m_rbo;
        i32 m_width, m_height;

    public:
        FrameBuffer(int width, int height);
        ~FrameBuffer();

        bool isCompleted() const;

        void genRenderBuffer(i32 internalFormat);
        void attachRenderBuffer(i32 attachment) const;
        void attachTexture(Texture2D &texture, i32 attachment = GL_COLOR_ATTACHMENT0) const;

        void bind() const;
        void unbind() const;

    };

}
