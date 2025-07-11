#include <cstdio>
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
#include "glUtilities/TextureBuffer.h"

#include <cmath> // for std:floor and std::sin

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
uniform samplerBuffer buf;

void main() {
    vec3 color = texture(screenTexture, uv).rgb;
    vec4 bufColor = vec4(texelFetch(buf, 0).r, texelFetch(buf, 1).r, texelFetch(buf, 2).r, texelFetch(buf, 3).r);
    color *= bufColor.rgb;
    color = pow(color, vec3(1.0 / 2.2));
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

out vec2 uv;

void main() {
    gl_Position = vec4(aPos, 0, 1.0);
    uv = aPos + vec2(0.5);
}
)";

const char *fragmentShaderSource = R"(
#version 330 core
out vec4 fragColor;

in vec2 uv;

void main() {
    fragColor = vec4(uv, 1.0, 1.0);
}
)";

static int width = 1640, height = 1280;

static void getRetinaScaler(f32* xScale, f32* yScale) {
    GLFWwindow* temp = glfwCreateWindow(1, 1, "", NULL, NULL);
    glfwGetWindowContentScale(temp, xScale, yScale);
    glfwDestroyWindow(temp);
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);
    ::width = width;
    ::height = height;
}

int main() {
    if(!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GL_TRUE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    f32 xScale, yScale;
    getRetinaScaler(&xScale, &yScale);

    std::printf("Retina Sacler [%.2g, %.2g]", xScale, yScale);

    GLFWwindow* window = glfwCreateWindow(width / xScale, height / yScale, "TBO Demo", NULL, NULL);
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
        // Initialize TBO
        f32 data[] = { 1.0f, 0.0f, 1.0f, 1.0f };
        gl::TextureBuffer tbo(data, sizeof(data), GL_STATIC_DRAW, GL_R32F);

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
        gl::Framebuffer screenFB;

        gl::Texture2D::Construct con;
        con.width = 80;
        con.height = 60;
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
            glViewport(0, 0, screenTexture.getWidth(), screenTexture.getHeight());
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
            tbo.bind(1);
            quadShader.setUniform1i("buf", 1);
            glDrawElements(GL_TRIANGLES, quadIbo.count(), GL_UNSIGNED_INT, 0);

            // Update Data
            [&data](float t) {
                t = t * 0.5 + 0.5 - std::floor(t * 0.5 + 0.5);
                data[0] = 0.5f + 0.5f * std::sin(2.0f * 3.14159265f * (t + 0.0f));
                data[1] = 0.5f + 0.5f * std::sin(2.0f * 3.14159265f * (t + 0.33f));
                data[2] = 0.5f + 0.5f * std::sin(2.0f * 3.14159265f * (t + 0.66f));
            }(std::sin(glfwGetTime() * 0.3));

            // Update TBO data
            tbo.updateBuffer(data, 0, 3 * sizeof(f32));

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
