#include <cstdio>
#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <iostream>

#include "RayScene.h"
#include "RayTracer.h"
#include "TraceableObject.h"

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

static u32 indices[] = {
    0, 1, 2,
    0, 3, 2,
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

    if (any(isnan(color)) || any(isinf(color)) || any(lessThan(color, vec3(0.0)))) {
        fragColor = vec4(0.0, 2.0, 0.0, 1.0);
        return;
    }

    color = pow(color, vec3(1.0 / 2.2));
    fragColor = vec4(color, 1.0);
}
)";

static int width = 2560, height = 1640;

static void getDPIScaler(f32* xScale, f32* yScale) {
    GLFWwindow* temp = glfwCreateWindow(1, 1, "", NULL, NULL);
    glfwGetWindowContentScale(temp, xScale, yScale);
    glfwDestroyWindow(temp);
}

static void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
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
    getDPIScaler(&xScale, &yScale);

    std::printf("Retina Sacler [%.2g, %.2g]\n", xScale, yScale);

    GLFWwindow* window = glfwCreateWindow(width / xScale, height / yScale, "Ray Tracer Demo - Spheres", NULL, NULL);
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
        // Initialize RayTracer
        Camera camera;
        camera.rayPerPixel = 9;
        camera.bounces = 8;
        camera.fov = 60;
        camera.resolution = { width * 0.5, height * 0.5 };
        camera.pitch = -25;
        camera.position = { 0, 0.35, 0.13 };
        camera.updateDirection();

        RayTracer raytracer;
        raytracer.initialize(camera);

        // Initialize RayScene
        RayScene scene;
        scene.initialize(camera);

        // Set up Scene
        {
            Material m;
            scene.addObject<Sphere>(m, glm::vec3{ 0, 0, 1 }, 0.12);

            scene.addObject<Sphere>(m, glm::vec3{ 0, -400 - 0.2, 1 }, 400);

            float l = 0.3;

            m.emissionColor = { 1, 0.2, 0.2 };
            m.emissionStrength = 140;
            scene.addObject<Sphere>(m, glm::vec3{ l, 0.5, 1.0 - l }, 0.03);

            m.emissionColor = { 0.2, 0.2, 1 };
            scene.addObject<Sphere>(m, glm::vec3{ -l, 0.5, 1.0 - l }, 0.03);

            m.emissionColor = { 0.2, 1.0, 0.2 };
            scene.addObject<Sphere>(m, glm::vec3{ 0, 0.5, 1 + l * sqrt(2) - 0.1 }, 0.03);

            m.emissionColor = { 1, 1, 1 };
            m.emissionStrength = 1;
            // scene.addObject<Sphere>(m, glm::vec3{ 0, 6, 30 }, 10);
        }

        scene.submit(); // submit scene to GPU

        // Initialize Framebuffer
        auto &screenTexture = raytracer.getCurrentFrame();
        gl::Framebuffer screenFB;
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

            double previous = glfwGetTime();
            screenFB.bind();
                glViewport(0, 0, screenTexture.getWidth(), screenTexture.getHeight());
                glClear(GL_COLOR_BUFFER_BIT);

                // Render scene
                quadVao.bind();
                raytracer.renderToTexture(scene);
                glDrawElements(GL_TRIANGLES, quadIbo.count(), GL_UNSIGNED_INT, 0);

                // Bind to next frame
                screenTexture = raytracer.getCurrentFrame();
                screenFB.attachTexture(screenTexture);

                if (!screenFB.isCompleted()) {
                    return -1;
                }

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

            double dt = (glfwGetTime() - previous);
            std::string title = "       " + std::to_string(raytracer.getFrameCount()) + "      " + std::to_string(dt * 1000);
            glfwSetWindowTitle(window, title.c_str());

            glfwPollEvents();
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
