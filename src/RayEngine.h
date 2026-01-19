#pragma once

#include <memory> // std::unique_ptr

#include "RayScene.h"
#include "RayTracer.h"

namespace gl {
    class Framebuffer;
    class Quad;
}

class RayEngine {
private:
    RayCamera m_camera;
    RayScene m_rayScene;
    RayTracer m_rayTracer;

    std::unique_ptr<gl::Framebuffer> m_framebuffer;
    std::unique_ptr<gl::Quad> m_quad;

public:

    RayEngine() = default;
    
    bool initialize(const RayCamera &camera, RayTracer::Type raytracerType);
    void changeResolution(glm::ivec2 resolution);
    void render();

    inline RayScene& getScene() { return m_rayScene; }
    inline RayTracer& getRayTracer() { return m_rayTracer; }

    inline gl::Quad& getQuad() { return *m_quad; }

};

