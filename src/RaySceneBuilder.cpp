#include "RaySceneBuilder.h"
#include "RayScene.h"
#include <glm/gtc/matrix_transform.hpp> // for glm::mat4_cast

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

void RaySceneBuilder::ThreeColorDemo(RayScene &scene) {
    Material m;
    scene.addObject<Sphere>(m, glm::vec3{ 0, 0, 1 }, 0.12);

    scene.addObject<Quad>(m, glm::vec3{ -5, -0.1, 0 }, glm::vec3{ 10, 0, 0 }, glm::vec3{ 0, 0, 10 });

    float l = 0.3;

    m.emissionColor = { 1, 0.2, 0.2 };
    m.emissionStrength = 140;
    scene.addObject<Sphere>(m, glm::vec3{ l, 0.5, 1.0 - l }, 0.03);

    m.emissionColor = { 0.2, 0.2, 1 };
    scene.addObject<Sphere>(m, glm::vec3{ -l, 0.5, 1.0 - l }, 0.03);

    m.emissionColor = { 0.2, 1.0, 0.2 };
    scene.addObject<Sphere>(m, glm::vec3{ 0, 0.5, 1 + l * sqrt(2) - 0.1 }, 0.03);

    m.emissionColor = { 1, 1, 1 };
    m.emissionStrength = 1;
}
 
void RaySceneBuilder::BuildCornellBox(RayScene &scene, glm::vec3 pos, float boxLen, float lightLen, float emissionStrenth, bool includedFront) {
    glm::vec3 red   = glm::vec3(.65, .05, .05);
    glm::vec3 green = glm::vec3(.12, .45, .15);
    glm::vec3 white = glm::vec3(1.0, 1.0, 1.0);

    Material m;
    m.albedo = glm::vec3(0.0);
    m.emissionColor = glm::vec3(1.0);
    m.emissionStrength = emissionStrenth;

    // Construct Light
    scene.addObject<Quad>(m,
                pos + glm::vec3{ (boxLen - lightLen) * 0.5, boxLen - 1e-4, boxLen * 0.5 - lightLen },
                glm::vec3{ lightLen, 0, 0 },
                glm::vec3{ 0, 0, lightLen }
            );

    // Construct Quads
    m = Material();

    // Left Quad
    m.albedo = red;
    scene.addObject<Quad>(m,
                pos,
                glm::vec3{ 0, boxLen, 0 },
                glm::vec3{ 0, 0, boxLen }
            );

    // Right Quad
    m.albedo = green;
    scene.addObject<Quad>(m,
                pos + glm::vec3{ boxLen, 0, 0 },
                glm::vec3{ 0, boxLen, 0 },
                glm::vec3{ 0, 0, boxLen }
            );

    m = Material();

    // Down Quad
    scene.addObject<Quad>(m,
                pos,
                glm::vec3{ boxLen, 0, 0 },
                glm::vec3{ 0, 0, boxLen }
            );

    // Top Quad
    scene.addObject<Quad>(m,
                pos + glm::vec3{ 0, boxLen, 0 },
                glm::vec3{ boxLen, 0, 0 },
                glm::vec3{ 0, 0, boxLen }
            );

    // Back Quad
    scene.addObject<Quad>(m,
                pos + glm::vec3{ 0, 0, boxLen },
                glm::vec3{ 0, boxLen, 0 },
                glm::vec3{ boxLen, 0, 0 }
            );

    // Front Quad
    if (includedFront) {
        scene.addObject<Quad>(m,
                    pos + glm::vec3{ 0, 0, 0 },
                    glm::vec3{ boxLen, 0, 0 },
                    glm::vec3{ 0, boxLen, 0 },
                    true
                );
    }
}

void RaySceneBuilder::BuildBox(RayScene &scene, const Material &material, glm::vec3 size, glm::vec3 pos, glm::quat q) {
    glm::mat3 R = glm::mat4_cast(q);

    glm::vec3 vertics[] = {
        glm::vec3{  size.x,  size.y,  size.z } * 0.5f * R,
        glm::vec3{ -size.x,  size.y,  size.z } * 0.5f * R,
        glm::vec3{  size.x, -size.y,  size.z } * 0.5f * R,
        glm::vec3{ -size.x, -size.y,  size.z } * 0.5f * R,
        glm::vec3{  size.x,  size.y, -size.z } * 0.5f * R,
        glm::vec3{ -size.x,  size.y, -size.z } * 0.5f * R,
        glm::vec3{  size.x, -size.y, -size.z } * 0.5f * R,
        glm::vec3{ -size.x, -size.y, -size.z } * 0.5f * R
    };

    // Top Quad
    scene.addObject<Quad>(material,
                pos + vertics[0],
                vertics[1] - vertics[0],
                vertics[4] - vertics[0]
            );

    // Down Quad
    scene.addObject<Quad>(material,
                pos + vertics[2],
                vertics[3] - vertics[2],
                vertics[6] - vertics[2]
            );

    // Left Quad
    scene.addObject<Quad>(material,
                pos + vertics[1],
                vertics[5] - vertics[1],
                vertics[3] - vertics[1]
            );

    // Right Quad
    scene.addObject<Quad>(material,
                pos + vertics[0],
                vertics[2] - vertics[0],
                vertics[4] - vertics[0]
            );

    // Back Quad
    scene.addObject<Quad>(material,
                pos + vertics[0],
                vertics[1] - vertics[0],
                vertics[2] - vertics[0]
            );

    // Front Quad
    scene.addObject<Quad>(material,
                pos + vertics[4],
                vertics[6] - vertics[4],
                vertics[5] - vertics[4]
            );

}

