#pragma once
#include "glm/glm.hpp"
#include "util.h"

#include <memory> // std::unique_ptr

struct RayCamera;
class RayScene;

namespace gl {
    class Texture2D;
    class ShaderProgram;
}

class RayTracer {
private:
    std::unique_ptr<gl::ShaderProgram> m_shader;
    std::unique_ptr<gl::Texture2D> m_frames[2];

    u32 m_frameCount = 1;
    i32 m_frameIndex = 0;

public:
    explicit RayTracer() = default;
    void initialize(glm::ivec2 resolution);
    void renderToTexture(const RayCamera &camera, const RayScene &scene);
    void changeResolution(glm::ivec2 resolution);

    inline gl::Texture2D &getCurrentFrame() const { return *m_frames[m_frameIndex]; }
    inline gl::Texture2D &getPreviousFrame() const { return *m_frames[!m_frameIndex]; }
    inline u32 getFrameCount() const { return m_frameCount; }

};
