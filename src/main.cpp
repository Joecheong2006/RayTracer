#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/Texture2D.h"
#include "glUtilities/Framebuffer.h"
#include "glUtilities/Quad.h"

#include "TraceableObject.h"
#include "RayEngine.h"

#include "RaySceneBuilder.h"

const char *screenVertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

out vec2 uv;

void main() {
    gl_Position = vec4(aPos, 1.0);
    uv = aPos.xy * 0.5 + 0.5;
}
)";

const char *screenFragmentShaderSource = R"(
#version 330 core
out vec4 fragColor;

in vec2 uv;

uniform sampler2D screenTexture;

void main() {
    vec3 color = texture(screenTexture, uv).rgb;

    if (any(isnan(color))) {
        fragColor = vec4(1.0, 0.0, 0.0, 1.0);
        return;
    }

    if (any(lessThan(color, vec3(0.0)))) {
        fragColor = vec4(0.0, 1.0, 0.0, 1.0);
        return;
    }

    if (any(isinf(color))) {
        fragColor = vec4(0.0, 0.0, 1.0, 1.0);
        return;
    }

    color = pow(color, vec3(1.0 / 2.2));
    fragColor = vec4(color, 1.0);
}
)";

static int width = 2048, height = 1280;

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

    const std::string title= "Ray Tracer Demo - Cornell Box";
    std::printf("Retina Sacler [%.2g, %.2g]\n", xScale, yScale);

    GLFWwindow *window = glfwCreateWindow(width / xScale, height / yScale, title.c_str(), NULL, NULL);
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
        // Initialize Camera
        RayCamera camera;
        camera.rayPerPixel = 1;
        camera.bounces = 15;
        camera.fov = 60;
        camera.resolution = { width * 0.5, height * 0.5 };
        camera.pitch = 0;
        camera.position = { 0, 1, -1.76 };
        camera.updateDirection();

        // Initialize RayEngine
        RayEngine rayEngine;
        rayEngine.initialize(camera);

        // Set up Scene
        {
            auto &scene = rayEngine.getScene();
            scene.setSkyColor({});

            RaySceneBuilder::BuildCornellBox(scene, glm::vec3{ -1, 0, 0 }, 2, 0.7);

            Material m;

            // Right Box
            RaySceneBuilder::BuildBox(scene, m,
                    glm::vec3{ 0.54, 0.54, 0.54 }, glm::vec3{0.32, 0.27, 0.7}, glm::angleAxis(glm::radians(-18.0f), glm::vec3(0, 1, 0)));

            m.roughness = 0;
            m.metallic = 1.0;
            // Left Box
            RaySceneBuilder::BuildBox(scene, m,
                    glm::vec3{ 0.6, 1.2, 0.6 }, glm::vec3{-0.35, 0.6, 1.2}, glm::angleAxis(glm::radians(18.0f), glm::vec3(0, 1, 0)));

            scene.submit(); // submit scene to GPU
        }

        // Initialize screen quad
        gl::ShaderProgram quadShader;
        quadShader.attachShaderCode(GL_VERTEX_SHADER, screenVertexShaderSource);
        quadShader.attachShaderCode(GL_FRAGMENT_SHADER, screenFragmentShaderSource);
        quadShader.link();
        quadShader.bind();

        glClearColor(0.01f, 0.011f, 0.01f, 1.0f);

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            // Bind screen framebuffer
            double previous = glfwGetTime();
            auto &raytracer = rayEngine.getRayTracer();
            auto &screenTexture = raytracer.getCurrentFrame();
            rayEngine.render();

            auto &quad = rayEngine.getQuad();

            // Draw screen texture
            glViewport(0, 0, width, height);
            glClear(GL_COLOR_BUFFER_BIT);
            quad.bind();
            quadShader.bind();
            screenTexture.bind(0);
            quadShader.setUniform1i("screenTexture", 0);
            glDrawElements(GL_TRIANGLES, quad.getCount(), GL_UNSIGNED_INT, 0);

            glfwSwapBuffers(window);

            double dt = (glfwGetTime() - previous);

            std::stringstream ss;
            ss << title << '\t' << raytracer.getFrameCount()
                << '\t' << std::fixed << std::setprecision(2) << dt * 1000.0 << "ms";

            glfwSetWindowTitle(window, ss.str().c_str());

            glfwPollEvents();
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
