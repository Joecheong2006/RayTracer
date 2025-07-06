#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <iostream>

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/VertexArray.h"
#include "glUtilities/VertexBufferLayout.h"
#include "glUtilities/VertexBuffer.h"
#include "glUtilities/IndexBuffer.h"
#include "glUtilities/Texture2D.h"
#include "glUtilities/Framebuffer.h"

static f32 quadVertices[] = {
     1.0f,  1.0f,
    -1.0f,  1.0f,
    -1.0f, -1.0f,
     1.0f, -1.0f,
};

const char *quadVertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec2 aPos;

out vec2 uv;

void main() {
    gl_Position = vec4(aPos, 0, 1.0);
    uv = aPos * 0.5 + 0.5;
}
)";

const char *quadFragmentShaderSource = R"(
#version 330 core
out vec4 fragColor;

in vec2 uv;

uniform sampler2D screenTexture;

void main() {
    vec3 color = texture(screenTexture, uv).rgb;
    color = pow(color, vec3(1.0 / 2.6));
    fragColor = vec4(color, 1.0);
}
)";

static f32 vertices[] = {
     0.5f,  0.5f,
    -0.5f,  0.5f,
    -0.5f, -0.5f,
     0.5f, -0.5f,
};

static u32 indices[] = {
    0, 1, 2,
    0, 3, 2,
};

const char *vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec2 aPos;

void main() {
    gl_Position = vec4(aPos, 0, 1.0);
}
)";

const char *fragmentShaderSource = R"(
#version 330 core
out vec4 fragColor;

void main() {
    fragColor = vec4(1.0, 0.5, 0.2, 1.0);
}
)";

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);
}

int main() {
    if(!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    const int width = 800, height = 600;

    GLFWwindow* window = glfwCreateWindow(width, height, "FrameBuffer Demo", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    {
        // Initialize Ractangle
        gl::VertexArray vao;

        gl::ShaderProgram shader;
        shader.attachShaderCode(GL_VERTEX_SHADER, vertexShaderSource);
        shader.attachShaderCode(GL_FRAGMENT_SHADER, fragmentShaderSource);
        shader.link();
        shader.bind();

        gl::VertexBuffer vbo(vertices, sizeof(vertices), GL_STATIC_DRAW);
        gl::IndexBuffer ibo(indices, sizeof(indices) / sizeof(u32), GL_STATIC_DRAW);

        {
            gl::VertexBufferLayout layout;
            layout.add<f32>(2);
            vao.applyBufferLayout(layout);
        }

        // Initialize Framebuffer
        gl::Framebuffer screenFB = gl::Framebuffer(width, height);
        screenFB.bind();
        screenFB.genRenderBuffer(GL_DEPTH24_STENCIL8);
        screenFB.attachRenderBuffer(GL_DEPTH_STENCIL_ATTACHMENT);

        gl::Texture2D::Construct con;
        con.width = width;
        con.height = height;
        con.style = GL_NEAREST;
        con.format = GL_RGBA;
        con.internal = GL_RGBA16F;
        gl::Texture2D screenTexture = gl::Texture2D(con);
        screenFB.attachTexture(screenTexture);

        if (!screenFB.isCompleted()) {
            return -1;
        }

        screenFB.unbind();

        // Initialize screen quad
        gl::VertexArray quadVao;

        gl::ShaderProgram quadShader;
        quadShader.attachShaderCode(GL_VERTEX_SHADER, quadVertexShaderSource);
        quadShader.attachShaderCode(GL_FRAGMENT_SHADER, quadFragmentShaderSource);
        quadShader.link();
        quadShader.bind();

        gl::VertexBuffer quadVbo(quadVertices, sizeof(quadVertices), GL_STATIC_DRAW);
        gl::IndexBuffer quadIbo(indices, sizeof(indices) / sizeof(u32), GL_STATIC_DRAW);

        {
            gl::VertexBufferLayout layout;
            layout.add<f32>(2);
            quadVao.applyBufferLayout(layout);
        }
        quadVao.unbind();

        glClearColor(0.01f, 0.011f, 0.01f, 1.0f);

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            // Bind screen framebuffer
            screenFB.bind();
            glViewport(0, 0, width, height);
            glClear(GL_COLOR_BUFFER_BIT);

            // Render scene
            vao.bind();
            shader.bind();
            glDrawElements(GL_TRIANGLES, ibo.count(), GL_UNSIGNED_INT, 0);

            screenFB.unbind();

            // Draw screen texture
            glViewport(0, 0, width, height);
            glClear(GL_COLOR_BUFFER_BIT);
            quadVao.bind();
            quadShader.bind();
            screenTexture.bind(0);
            quadShader.setUniform1i("screenTexture", 0);
            glDrawElements(GL_TRIANGLES, quadIbo.count(), GL_UNSIGNED_INT, 0);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
