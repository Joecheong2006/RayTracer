#include "RayEngine.h"

#include "glUtilities/Framebuffer.h"
#include "glUtilities/Texture2D.h"
#include "glUtilities/Quad.h"

bool RayEngine::initialize(const RayCamera &camera, RayTracer::Type raytracerType) {
    m_camera = camera;
    m_rayScene.initialize();

    if (!m_rayTracer.initialize(camera.resolution, raytracerType)) {
        return false;
    }

    m_framebuffer = std::make_unique<gl::Framebuffer>();
    m_quad = std::make_unique<gl::Quad>();
    return true;
}

void RayEngine::changeResolution(glm::ivec2 resolution) {
    getRayTracer().changeResolution(resolution);
    m_camera.resolution = resolution;
}

void RayEngine::render() {
    m_framebuffer->bind();

    auto &screenTexture = m_rayTracer.getCurrentFrame();
    m_framebuffer->attachTexture(screenTexture);
    ASSERT(m_framebuffer->isCompleted());

    glViewport(0, 0, screenTexture.getWidth(), screenTexture.getHeight());

    m_quad->bind();
    m_rayTracer.renderToTexture(m_camera, m_rayScene);
    glDrawElements(GL_TRIANGLES, m_quad->getCount(), GL_UNSIGNED_INT, 0);

    m_framebuffer->unbind();
}

