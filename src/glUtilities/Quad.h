#pragma once
#include "../util.h"

#include "glUtilities/VertexArray.h"
#include "glUtilities/VertexBuffer.h"
#include "glUtilities/IndexBuffer.h"

namespace gl {
    class Quad {
        static const f32 vertices[];
        static const u32 indices[];

        gl::VertexArray vao;
        gl::VertexBuffer vbo;
        gl::IndexBuffer ibo;

    public:
        Quad();
        void bind() const;
        u32 getCount() const { return ibo.count(); }

    };
}
