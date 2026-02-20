#pragma once
#include "Material.h"
#include "TraceableObject.h"

#include "glUtilities/TextureBuffer.h"
#include "glm/glm.hpp"

#include <string> // std::string
#include <vector> // std::vector
#include <memory> // std::unique_ptr

namespace gl {
    class ShaderProgram;
}

class RayScene {
private:
    glm::vec3 m_skyColor = { 0.5, 0.7, 1.0 };
    glm::vec3 m_linearSkyColor = glm::pow(m_skyColor, glm::vec3(2.2));

    // Materials
    std::vector<Material> m_materials;
    std::unique_ptr<gl::TextureBuffer> m_materialsTexBuffer;
    std::vector<f32> m_materialsBuffer;

    // Primitive objects
    std::vector<std::unique_ptr<TraceableObject>> m_traceableObjects;
    std::unique_ptr<gl::TextureBuffer> m_objectsTexBuffer;
    std::vector<f32> m_objectsBuffer;

    // Models
    std::vector<std::unique_ptr<Model>> m_modelObjects;
    std::unique_ptr<gl::TextureBuffer> m_modelObjectsTexBuffer;
    std::vector<f32> m_modelObjectsBuffer;

    // Textures
    std::unique_ptr<gl::TextureBuffer> m_texturesTexBuffer;
    std::vector<f32> m_texturesBuffer;

public:
    explicit RayScene() = default;

    void initialize();
    void bindShader(gl::ShaderProgram &shader) const;
    void submit();

    void setSkyColor(glm::vec3 skyColor);
    glm::vec3 getSkyColor() const;
    glm::vec3 getLinearSkyColor() const;
    inline u32 getObjectsCount() const { return static_cast<u32>(m_traceableObjects.size()); }
    inline u32 getModelsCount() const { return static_cast<u32>(m_modelObjects.size()); }

    void addModel(const std::string &modelPath);
    void addObject(const TraceableObject &obj, const Material &material);
    void addObject(const TraceableObject &obj, int materialIndex);

};

