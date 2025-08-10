#pragma once
#include "RayScene.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct RaySceneBuilder {
    static void RoughnessMetallicDemo(RayScene &scene);
    static void ThreeColorDemo(RayScene &scene);
    static void BuildCornellBox(RayScene &scene, glm::vec3 pos, float boxLen, float lightLen);
    static void BuildBox(RayScene &scene, const Material &material, glm::vec3 size, glm::vec3 pos, glm::quat q);

};

