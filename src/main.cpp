#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <queue>

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/Texture2D.h"
#include "glUtilities/Framebuffer.h"
#include "glUtilities/Quad.h"

#include "TraceableObject.h"
#include "RayEngine.h"

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

// ACES Filmic
vec3 tonemap_aces(vec3 color) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    vec3 result = (color * (a * color + b)) / (color * (c * color + d) + e);
    return clamp(result, 0.0, 1.0);
}

const vec3 E_WHITE = vec3(0.997065, 1.002169, 0.988182);
const vec3 D65_WHITE = vec3(0.95047, 1.00000, 1.08883);

const mat3 BRADFORD_MA = mat3(
     0.8951, -0.7502,  0.0389,  // Column 0 (was row 0)
     0.2664,  1.7135, -0.0685,  // Column 1 (was row 1)
    -0.1614,  0.0367,  1.0296   // Column 2 (was row 2)
);

const mat3 BRADFORD_MA_INV = mat3(
     0.9870,  0.4323, -0.0085,  // Column 0
    -0.1471,  0.5184,  0.0400,  // Column 1
     0.1600,  0.0493,  0.9685   // Column 2
);

vec3 chromatic_adapt_E_to_D65(vec3 xyz) {
    vec3 rgb_src = BRADFORD_MA * E_WHITE;
    vec3 rgb_dst = BRADFORD_MA * D65_WHITE;
    vec3 gain = rgb_dst / rgb_src;
    vec3 adapted = BRADFORD_MA * xyz;
    adapted *= gain;
    adapted = BRADFORD_MA_INV * adapted;
    return adapted;
}

const mat3 XYZ_TO_RGB = mat3(
    3.2406, -0.9689,  0.0557,
   -1.5372,  1.8758, -0.2040,
   -0.4986,  0.0415,  1.0570
);

vec3 xyz_to_rgb(vec3 xyz) {
    return max(XYZ_TO_RGB * xyz, 0.0);
}

void main() {
    vec3 color = texture(screenTexture, uv).rgb;

    color = chromatic_adapt_E_to_D65(color);
    color = xyz_to_rgb(color);

    color = tonemap_aces(color);

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

    const std::string title= "Spectral Ray Tracer";
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
        camera.bounces = 5;
        camera.fov = 50;
        camera.resolution = { width, height };
        camera.pitch = 0;
        camera.position = { 0, 1, -2.2 };
        camera.updateDirection();

        // Initialize RayEngine
        RayEngine rayEngine;
        ASSERT(rayEngine.initialize(camera, RayTracer::Type::Spectral));
        rayEngine.changeResolution({ width * 0.5f, height * 0.5f });

        // Set up Scene
        {
            auto &scene = rayEngine.getScene();

            scene.setSkyColor({});

            // NOTE: Models are in res folder to be unziped
            scene.addModel("cornellBox.glb");
            scene.addModel("test1.glb");

            scene.submit(); // submit scene to GPU
        }

        // Initialize screen quad
        gl::ShaderProgram quadShader;
        quadShader.attachShaderCode(GL_VERTEX_SHADER, screenVertexShaderSource);
        quadShader.attachShaderCode(GL_FRAGMENT_SHADER, screenFragmentShaderSource);
        quadShader.link();
        quadShader.bind();

        glClearColor(0.01f, 0.011f, 0.01f, 1.0f);

        f32 avgRenderTime = 0;
        std::queue<f32> frameQueue({ 0, 0, 0 });

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            double previous = glfwGetTime();
            auto &raytracer = rayEngine.getRayTracer();
            auto &screenTexture = raytracer.getCurrentFrame();

            // Accumulate Scene
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

            f32 dt = (glfwGetTime() - previous) * 1000.0f;

            i32 frameCount = raytracer.getFrameCount();
            avgRenderTime -= frameQueue.front() - frameQueue.back();
            frameQueue.push(dt);
            frameQueue.pop();

            std::stringstream ss;
            ss << title
                << '\t'<< frameCount
                <<'\t' << std::fixed << std::setprecision(3) << avgRenderTime / 3.0 << "ms";

            glfwSetWindowTitle(window, ss.str().c_str());

            glfwPollEvents();
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
