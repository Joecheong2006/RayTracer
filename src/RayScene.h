#pragma once

#include "Material.h"
#include "TraceableObject.h"
#include "glUtilities/TextureBuffer.h"
#include "glUtilities/util.h"

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
    friend class TraceableObject;
private:
    RayCamera m_camera;

    glm::vec3 m_skyColor = { 0.5, 0.7, 1.0 };

    // Materials
    std::vector<Material> m_materials;
    std::unique_ptr<gl::TextureBuffer> m_materialsTexBuffer;
    std::vector<glm::vec4> m_materialsBuffer;

    // Objects
    std::vector<std::unique_ptr<TraceableObject>> m_traceableObjects;
    std::unique_ptr<gl::TextureBuffer> m_objectsTexBuffer;
    std::vector<glm::vec4> m_objectsBuffer;

    u32 m_objectBottomIndex = 0;

    void load_material(i32 materialIndex);
    void load_objects();

public:
    static std::vector<Triangle> LoadModel(std::string modelPath);

public:
    explicit RayScene() = default;

    void initialize(const RayCamera &camera);
    void bindObjects(i32 slot) const;
    void bindMaterials(i32 slot) const;
    void submit();

    void setSkyColor(glm::vec3 skyColor);
    glm::vec3 getSkyColor() const;
    i32 getMaterialCount() const { return m_materials.size(); }
    
    inline u32 getObjectsCount() const { return static_cast<u32>(m_traceableObjects.size()); }

    const RayCamera &getCamera() const { return m_camera; }

    template <typename T, typename... Args>
    struct trace_initalize {
        trace_initalize(RayScene&) {}
    };

    template <typename... Args>
    struct trace_initalize<Model, Args...> {
        trace_initalize(RayScene &scene) {
            auto model = static_cast<Model&>(*scene.m_traceableObjects.back().get());
            for (auto &triangle : model.triangles) {
                scene.m_traceableObjects.push_back(std::make_unique<Triangle>(triangle));
                auto &object = scene.m_traceableObjects.back();
                object->m_materialIndex = model.m_materialIndex;
            }
        }
    };

    template <typename T, typename... Args>
    void addObject(const Material &material, Args&&... args) {
        m_traceableObjects.push_back(std::make_unique<T>(std::forward<Args>(args)...));

        auto &object = m_traceableObjects.back();
        object->m_materialIndex = m_materials.size();
        m_materials.push_back(material);

        trace_initalize<T, Args...>(*this);
    }

    template <typename T, typename... Args>
    void addObject(int materialIndex, Args&&... args) {
        ASSERT(materialIndex >= 0 && materialIndex < m_materials.size());
        m_traceableObjects.push_back(std::make_unique<T>(std::forward<Args>(args)...));

        auto &object = m_traceableObjects.back();
        object->m_materialIndex = materialIndex;

        trace_initalize<T, Args...>(*this);
    }

};

