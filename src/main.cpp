#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <iostream>

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/VertexArray.h"
#include "glUtilities/VertexBufferLayout.h"
#include "glUtilities/VertexBuffer.h"
#include "glUtilities/IndexBuffer.h"

static f32 vertices[] = {
     0.5f,  0.5f, 0.0f,
    -0.5f,  0.5f, 0.0f,
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
};

static u32 indices[] = {
    0, 1, 2,
    0, 3, 2,
};

const char *vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

void main() {
    gl_Position = vec4(aPos, 1.0);
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
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    const int width = 800, height = 600;

    GLFWwindow* window = glfwCreateWindow(width, height, "Hello Ractangle", NULL, NULL);
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
        gl::VertexArray vao;

        gl::ShaderProgram shader;
        shader.attachShaderCode(GL_VERTEX_SHADER, vertexShaderSource);
        shader.attachShaderCode(GL_FRAGMENT_SHADER, fragmentShaderSource);
        shader.link();
        shader.bind();

        gl::VertexBuffer vbo(vertices, 12 * 4, GL_STATIC_DRAW);

        gl::IndexBuffer ibo(indices, sizeof(indices) / sizeof(u32), GL_STATIC_DRAW);

        gl::VertexBufferLayout layout;
        layout.add<f32>(3);
        vao.applyBufferLayout(layout);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            glClear(GL_COLOR_BUFFER_BIT);

            vao.bind();
            shader.bind();
            glDrawElements(GL_TRIANGLES, ibo.count(), GL_UNSIGNED_INT, 0);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
