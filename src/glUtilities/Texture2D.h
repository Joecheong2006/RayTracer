#pragma once

#include "../util.h"
#include <string>
#include "glad/glad.h"

namespace gl {
    class FrameBuffer;
    class Texture2D {
        friend FrameBuffer;
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

        inline i32 width() { return con.width; }
        inline i32 height() { return con.height; }
        inline i32 bpp() { return con.bpp; }

    };

}
