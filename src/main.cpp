#include <cstdio>
#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "RayScene.h"
#include "RayTracer.h"
#include "TraceableObject.h"

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/Texture2D.h"
#include "glUtilities/Framebuffer.h"

#include "glUtilities/Quad.h"

const char *quadVertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

out vec2 uv;

void main() {
    gl_Position = vec4(aPos, 1.0);
    uv = aPos.xy * 0.5 + 0.5;
}
)";

const char *quadFragmentShaderSource = R"(
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

    const std::string title= "Ray Tracer Demo - Roughness and Metallic";
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
        camera.rayPerPixel = 4;
        camera.bounces = 8;
        camera.fov = 60;
        camera.resolution = { width * 0.5, height * 0.5 };
        camera.pitch = -50;
        camera.position = { 0, 1.5, 0.3 };
        camera.updateDirection();

        // Initialize RayTracer
        RayTracer raytracer;
        raytracer.initialize(camera);

        // Initialize RayScene
        RayScene scene;
        scene.initialize(camera);

        // Set up Scene
        {
            Material m;

            m.albedo = glm::vec3(.65, .05, .05);
            for (int i = 0; i <= 10; ++i) {
                for (int j = 0; j <= 1; ++j) {
                    m.roughness = i / 10.0;
                    m.metallic = j * (1 - i / 10.0);
                    scene.addObject<Sphere>(
                            m, glm::vec3{i * 0.3 - 10 * 0.5 * 0.3, 0, 2 - j * 0.5}, 0.1);
                }
            }

            m = Material();
            scene.addObject<Quad>(m, glm::vec3{ -5, -0.1, 0 }, glm::vec3{ 10, 0, 0 }, glm::vec3{ 0, 0, 10 });

            m.emissionColor = { 1, 1, 1 };
            m.emissionStrength = 100;
            scene.addObject<Sphere>(m, glm::vec3{ -5, 8, -15 }, 1.5);
        }

        scene.submit(); // submit scene to GPU

        // Initialize Framebuffer
        auto &screenTexture = raytracer.getCurrentFrame();
        gl::Framebuffer screenFB;

        // Initialize screen quad
        gl::Quad quad;

        gl::ShaderProgram quadShader;
        quadShader.attachShaderCode(GL_VERTEX_SHADER, quadVertexShaderSource);
        quadShader.attachShaderCode(GL_FRAGMENT_SHADER, quadFragmentShaderSource);
        quadShader.link();
        quadShader.bind();

        glClearColor(0.01f, 0.011f, 0.01f, 1.0f);

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            // Bind screen framebuffer
            double previous = glfwGetTime();
            screenFB.bind();

                // Bind to next frame
                screenTexture = raytracer.getCurrentFrame();
                screenFB.attachTexture(screenTexture);
                ASSERT(screenFB.isCompleted());

                glViewport(0, 0, screenTexture.getWidth(), screenTexture.getHeight());

                // Render scene
                quad.bind();
                raytracer.renderToTexture(scene);
                glDrawElements(GL_TRIANGLES, quad.getCount(), GL_UNSIGNED_INT, 0);

            screenFB.unbind();

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
