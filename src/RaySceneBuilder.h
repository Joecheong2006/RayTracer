#pragma once
#include "Material.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RayScene;
struct RaySceneBuilder {
    static void RoughnessMetallicDemo(RayScene &scene);
    static void ThreeColorDemo(RayScene &scene);
    static void BuildCornellBox(RayScene &scene, glm::vec3 pos, float boxLen, float lightLen, float emissionStrenth, bool includedFront = false);
    static void BuildBox(RayScene &scene, const Material &material, glm::vec3 size, glm::vec3 pos, glm::quat q);

};

