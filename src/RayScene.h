#pragma once

#include "Material.h"
#include "TraceableObject.h"
#include "glUtilities/TextureBuffer.h"

#include <string> // std::string
#include <vector> // std::vector
#include <memory> // std::unique_ptr

struct RayCamera {
    glm::vec3 position = { 0, 0, 0 };
    f32 yaw = 90, pitch = 0, fov = 45;

    glm::vec3 up = glm::vec3(0, 1, 0),
              right = glm::vec3(1, 0, 0),
              forward = glm::vec3(0, 0, 1);

    glm::ivec2 resolution = { 1024, 1024 };
    i32 bounces = 5, rayPerPixel = 1;

    void updateDirection();
};

class RayScene {
private:
    glm::vec3 m_skyColor = { 0.5, 0.7, 1.0 };

    // Materials
    std::unique_ptr<gl::TextureBuffer> m_materialTexturesTexBuffer;
    std::vector<i32> m_materialTexturesBuffer;

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

    void load_material(const Material &material);

public:
    explicit RayScene() = default;

    void initialize();
    void bindObjects(i32 slot) const;
    void bindModelObjects(i32 slot) const;
    void bindTextures(i32 slot) const;
    void bindMaterials(i32 slot) const;
    void bindMaterialTextures(i32 slot) const;
    void submit();

    void setSkyColor(glm::vec3 skyColor);
    glm::vec3 getSkyColor() const;
    inline u32 getObjectsCount() const { return static_cast<u32>(m_traceableObjects.size()); }
    inline u32 getModelsCount() const { return static_cast<u32>(m_modelObjects.size()); }

    void addModel(const std::string &modelPath);

    template <typename T, typename... Args>
    void addObject(const Material &material, Args&&... args) {
        m_traceableObjects.push_back(std::make_unique<T>(std::forward<Args>(args)...));

        auto &object = m_traceableObjects.back();

        object->m_materialIndex = m_materials.size();
        object->writeHeader(m_objectsBuffer);
        object->write(m_objectsBuffer);

        m_materials.push_back(material);
        load_material(material);
    }

    template <typename T, typename... Args>
    void addObject(int materialIndex, Args&&... args) {
        ASSERT(materialIndex >= 0 && materialIndex < m_materials.size());
        m_traceableObjects.push_back(std::make_unique<T>(std::forward<Args>(args)...));

        auto &object = m_traceableObjects.back();

        object->m_materialIndex = materialIndex;
        object->writeHeader(m_objectsBuffer);
        object->write(m_objectsBuffer);
    }

};

