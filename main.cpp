#include <iostream>
#include <limits>
#include <cmath>
#include <optional>

#include "parser.h"
#include "ppmimage.h"
#include "tinymath.h"

struct Ray {
    tinymath::vec3f origin;
    tinymath::vec3f direction;
};

std::optional<float> intersect(const Ray & ray, const parser::Sphere & sphere, const std::vector<tinymath::vec3f> & vertex_data) {
    tinymath::vec3f sphereCenter = vertex_data[sphere.center_vertex_id - 1];

    tinymath::vec3f direction = tinymath::normalize(ray.direction);
    float fst = tinymath::dot(direction, ray.origin - sphereCenter);
    float snd = tinymath::dot(direction, direction);
    float thrd = tinymath::dot(ray.origin - sphereCenter, ray.origin - sphereCenter) - sphere.radius * sphere.radius;
    float determinant = fst * fst - snd * thrd;

    if (determinant < 0) {
        return std::nullopt;
    }
    
    float t = (-tinymath::dot(direction, (ray.origin - sphereCenter)) - std::sqrt(determinant)) / snd;
    return t;
}

std::optional<float> intersect(const Ray & ray, const parser::Face & triangle, const std::vector<tinymath::vec3f> & vertex_data) {
    tinymath::vec3f vertexA = vertex_data[triangle.v0_id - 1];
    tinymath::vec3f vertexB = vertex_data[triangle.v1_id - 1];
    tinymath::vec3f vertexC = vertex_data[triangle.v2_id - 1];

    tinymath::vec3f normal = tinymath::cross(vertexB - vertexA,
        vertexC - vertexA);
    tinymath::vec3f direction = tinymath::normalize(ray.direction);

    const float almostZero = 1e-6;

    float denom = tinymath::dot(normal, direction);
    // TODO add check for being in the [t_min, t_max] range
    if (std::abs(denom) < almostZero)
        return std::nullopt;

    float intersectLength = tinymath::dot(normal, vertexA - ray.origin) / denom;
    tinymath::vec3f planeIntersection = ray.origin + intersectLength * direction;

    tinymath::vec3f center = (vertexA + vertexB + vertexC) / 3;

    auto CG = center - vertexC;
    auto CB = vertexB - vertexC;
    auto CP = planeIntersection - vertexC;
    auto CGxCB = tinymath::cross(CG, CB);
    auto CPxCB = tinymath::cross(CP, CB);
    if (tinymath::normalize(CGxCB) != tinymath::normalize(CPxCB)
        && tinymath::length(CPxCB) > 0)
        return std::nullopt;

    auto AG = center - vertexA;
    auto AC = vertexC - vertexA;
    auto AP = planeIntersection - vertexA;
    auto AGxAC = tinymath::cross(AG, AC);
    auto APxAC = tinymath::cross(AP, AC);
    if (tinymath::normalize(AGxAC) != tinymath::normalize(APxAC)
        && tinymath::length(APxAC) > 0)
        return std::nullopt;

    auto BG = center - vertexB;
    auto BA = vertexA - vertexB;
    auto BP = planeIntersection - vertexB;
    auto BGxBA = tinymath::cross(BG, BA);
    auto BPxBA = tinymath::cross(BP, BA);
    if (tinymath::normalize(BGxBA) != tinymath::normalize(BPxBA)
        && tinymath::length(BPxBA) > 0)
        return std::nullopt;

    return intersectLength;
}
#if 0
    // barycentric
    float w1_denom = ((vertexB.y - vertexA.y) * (vertexC.x - vertexA.x)
                     - (vertexB.x - vertexA.x) * (vertexC.y - vertexA.y));
    if (std::abs(w1_denom) < almostZero)
        return std::nullopt;

    float weight1 = (vertexA.x * (vertexC.y - vertexA.y)
                     + (planeIntersection.y - vertexA.y) * (vertexC.x - vertexA.x)
                     - planeIntersection.x * (vertexC.y - vertexA.y))
                    / w1_denom;

    if (weight1 < almostZero)
        return std::nullopt;

    float w2_denom = vertexC.y - vertexA.y;
    if (std::abs(w2_denom) < almostZero)
        return std::nullopt;

    float weight2 =  (planeIntersection.y - vertexA.y - weight1 * (vertexB.y - vertexA.y))
                     / w2_denom;

    if (weight2 < almostZero)
        return std::nullopt;
     
    if (weight1 + weight2 > 1)
        return std::nullopt;

    return intersectLength;
#endif

int main(int argc, char* argv[]) {
    parser::Scene scene;

    scene.loadFromXml("../hw1_sample_scenes/simple.xml");

    for (const auto & camera : scene.cameras) {

        int width = camera.image_width;
        int height = camera.image_height;

        PPMImage image(width, height);

        float imageLeft = camera.near_plane.x;
        float imageRight = camera.near_plane.y;
        float imageBottom = camera.near_plane.z;
        float imageTop = camera.near_plane.w;

        tinymath::vec3f right = tinymath::cross(camera.up, -1 * camera.gaze);

        tinymath::vec3f topLeftVector = camera.position + camera.gaze * camera.near_distance
            + camera.up * imageTop
            + right * imageLeft;


        float widthPerPixel = (imageRight - imageLeft) / width;
        float heightPerPixel = (imageTop - imageBottom) / height;

        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                Ray rayToPixel;
                rayToPixel.origin = camera.position;

                float uOffset = widthPerPixel * (i + 0.5f);
                float vOffset = heightPerPixel * (j + 0.5f);
                rayToPixel.direction = (topLeftVector
                    + uOffset * right
                    - vOffset * camera.up) - rayToPixel.origin;


                enum class ObjectType {
                    TRIANGLE,
                    SPHERE
                };
                const float infinity = std::numeric_limits<float>::max();
                float t = infinity;
                ObjectType minObject;
                const parser::Face * minFace;
                const parser::Sphere * minSphere;

                for (const auto & sphere : scene.spheres) {
                    std::optional<float> intersected = intersect(rayToPixel, sphere, scene.vertex_data);
                    if (intersected) {
                        // FIXME t_current should come from intersect function, currently
                        // it is the length to pixel
                        float t_current = intersected.value();
                        // float t_current = tinymath::length(rayToPixel.direction);
                        if (t_current < t) {
                            t = t_current;
                            minSphere = &sphere;
                            minObject = ObjectType::SPHERE;
                        }
                    }
                }

                for (const auto & triangle : scene.triangles) {
                    std::optional<float> intersected = intersect(rayToPixel, triangle.indices, scene.vertex_data);
                    if (intersected) {
                        //float t_current = tinymath::length(rayToPixel.direction);
                        float t_current = intersected.value();
                        if (t_current < t) {
                            t = t_current;
                            minFace = &triangle.indices;
                            minObject = ObjectType::TRIANGLE;
                        }
                    }
                }

                for (const auto & mesh : scene.meshes) {
                    for (const auto & face : mesh.faces) {
                        std::optional<float> intersected = intersect(rayToPixel, face, scene.vertex_data);
                        if (intersected) {
                            //float t_current = tinymath::length(rayToPixel.direction);
                            float t_current = intersected.value();
                            if (t_current < t) {
                                t = t_current;
                                minFace = &face;
                                minObject = ObjectType::TRIANGLE;
                            }
                        }
                    }
                }

                PPMColor color;
                if (t != infinity) { // ray hit an object
                    if (minObject == ObjectType::SPHERE) {
                        // color = minSphere -> color;
                        color = { 255, 255, 255 };
                        //color = { (int)(t * 100), (int)t, (int)t };
                    }
                    if (minObject == ObjectType::TRIANGLE) {
                        // color = minTriangle -> color;
                        color = { 255, 255, 255 };
                        //color = { (int)(t * 100), (int)t, (int)t };
                    }
                    // if triangle ...
                    //
                }
                else {
                    color = { 0, 0, 0 };
                }

                image.setPixel(i, j, color);
            }
        }

        std::cout << "writing..." << std::endl;
        image.writeFile(camera.image_name);
    }
}
