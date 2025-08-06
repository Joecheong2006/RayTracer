#include "Quad.h"
#include <glad/glad.h>
#include "glUtilities/VertexBufferLayout.h"

namespace gl {
    const f32 Quad::vertices[] = {
         1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f,
         1.0f, -1.0f, 0.0f
    };

    const u32 Quad::indices[] = {
        0, 1, 2,
        0, 3, 2,
    };

    Quad::Quad()
        : vbo(vertices, sizeof(vertices), GL_STATIC_DRAW)
        , ibo(indices, sizeof(indices) / sizeof(u32), GL_STATIC_DRAW)
    {
        gl::VertexBufferLayout layout;
        layout.add<f32>(3);
        vao.applyBufferLayout(layout);
        vao.unbind();
    }

    void Quad::bind() const {
        vao.bind();
    }

}

