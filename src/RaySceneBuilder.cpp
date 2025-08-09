#include "RaySceneBuilder.h"

void RaySceneBuilder::RoughnessMetallicDemo(RayScene &scene) {
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

