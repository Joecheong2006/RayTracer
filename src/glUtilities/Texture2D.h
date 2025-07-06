#pragma once

#include "../util.h"
#include <string>
#include "glad/glad.h"

namespace gl {
    class Framebuffer;
    class Texture2D {
        friend Framebuffer;
    public:
        struct Construct {
            i32 width = 0, height = 0, style = 0, format = GL_RGBA, wrapStyle = GL_CLAMP_TO_EDGE, internal = 0, type = GL_UNSIGNED_BYTE, bpp = 0;
            bool mipmap = true;
            void *data = nullptr;
        };

    private:
        u32 m_id;
        Construct con;

    public:
        static Construct load(const std::string &path);
        Texture2D(const Construct &con);
        ~Texture2D();

        void bind(u32 slot) const;
        void unbind() const;

        inline i32 getWidth() { return con.width; }
        inline i32 getHeight() { return con.height; }
        inline i32 getBpp() { return con.bpp; }

    };

}
