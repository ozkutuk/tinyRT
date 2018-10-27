#include <cmath>
#include <iostream>
#include <limits>
#include <optional>

#include "parser.h"
#include "ppmimage.h"
#include "tinymath.h"

struct Ray {
    tinymath::vec3f origin;
    tinymath::vec3f direction;
};

std::optional<float>
intersect(const Ray &ray, const parser::Sphere &sphere,
          const std::vector<tinymath::vec3f> &vertex_data) {
    tinymath::vec3f sphereCenter = vertex_data[sphere.center_vertex_id - 1];

    tinymath::vec3f direction = tinymath::normalize(ray.direction);
    float fst = tinymath::dot(direction, ray.origin - sphereCenter);
    float snd = tinymath::dot(direction, direction);
    float thrd =
        tinymath::dot(ray.origin - sphereCenter, ray.origin - sphereCenter) -
        sphere.radius * sphere.radius;
    float determinant = fst * fst - snd * thrd;

    if (determinant < 0) {
        return std::nullopt;
    }

    float t = (-tinymath::dot(direction, (ray.origin - sphereCenter)) -
               std::sqrt(determinant)) /
              snd;
    return t;
}

std::optional<float>
intersect(const Ray &ray, const parser::Face &triangle,
          const std::vector<tinymath::vec3f> &vertex_data) {
    tinymath::vec3f vertexA = vertex_data[triangle.v0_id - 1];
    tinymath::vec3f vertexB = vertex_data[triangle.v1_id - 1];
    tinymath::vec3f vertexC = vertex_data[triangle.v2_id - 1];

    tinymath::vec3f edgeBC = vertexC - vertexB;
    tinymath::vec3f edgeBA = vertexA - vertexB;

    tinymath::vec3f direction = tinymath::normalize(ray.direction);
    tinymath::vec3f pvec = tinymath::cross(direction, edgeBA);
    float determinant = tinymath::dot(edgeBC, pvec);

    // no face-culling
    const float almostZero = 1e-6;
    if (std::abs(determinant) < almostZero)
        return std::nullopt;

    float inverseDeterminant = 1 / determinant;

    tinymath::vec3f tvec = ray.origin - vertexB;
    float u = tinymath::dot(tvec, pvec) * inverseDeterminant;
    if (u < 0 || u > 1)
        return std::nullopt;

    tinymath::vec3f qvec = tinymath::cross(tvec, edgeBC);
    float v = tinymath::dot(direction, qvec) * inverseDeterminant;
    if (v < 0 || u + v > 1)
        return std::nullopt;

    float t = tinymath::dot(edgeBA, qvec) * inverseDeterminant;
    return t;
}

int main(int argc, char *argv[]) {
    parser::Scene scene;

    scene.loadFromXml("../hw1_sample_scenes/bunny.xml");

    for (const auto &camera : scene.cameras) {

        int width = camera.image_width;
        int height = camera.image_height;

        PPMImage image(width, height);

        float imageLeft = camera.near_plane.x;
        float imageRight = camera.near_plane.y;
        float imageBottom = camera.near_plane.z;
        float imageTop = camera.near_plane.w;

        tinymath::vec3f right = tinymath::cross(camera.up, -1 * camera.gaze);

        tinymath::vec3f topLeftVector =
            camera.position + camera.gaze * camera.near_distance +
            camera.up * imageTop + right * imageLeft;

        float widthPerPixel = (imageRight - imageLeft) / width;
        float heightPerPixel = (imageTop - imageBottom) / height;

        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                Ray rayToPixel;
                rayToPixel.origin = camera.position;

                float uOffset = widthPerPixel * (i + 0.5f);
                float vOffset = heightPerPixel * (j + 0.5f);
                rayToPixel.direction =
                    (topLeftVector + uOffset * right - vOffset * camera.up) -
                    rayToPixel.origin;

                enum class ObjectType { TRIANGLE, SPHERE };
                const float infinity = std::numeric_limits<float>::max();
                float t = infinity;
                ObjectType minObject;
                const parser::Face *minFace;
                const parser::Sphere *minSphere;

                for (const auto &sphere : scene.spheres) {
                    std::optional<float> intersected =
                        intersect(rayToPixel, sphere, scene.vertex_data);
                    if (intersected) {
                        float t_current = intersected.value();
                        if (t_current < t) {
                            t = t_current;
                            minSphere = &sphere;
                            minObject = ObjectType::SPHERE;
                        }
                    }
                }

                for (const auto &triangle : scene.triangles) {
                    std::optional<float> intersected = intersect(
                        rayToPixel, triangle.indices, scene.vertex_data);
                    if (intersected) {
                        float t_current = intersected.value();
                        if (t_current < t) {
                            t = t_current;
                            minFace = &triangle.indices;
                            minObject = ObjectType::TRIANGLE;
                        }
                    }
                }

                for (const auto &mesh : scene.meshes) {
                    for (const auto &face : mesh.faces) {
                        std::optional<float> intersected =
                            intersect(rayToPixel, face, scene.vertex_data);
                        if (intersected) {
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
                        color = {255, 255, 255};
                        // color = { (int)(t * 100), (int)t, (int)t };
                    }
                    if (minObject == ObjectType::TRIANGLE) {
                        // color = minTriangle -> color;
                        color = {255, 255, 255};
                        // color = { (int)(t * 100), (int)t, (int)t };
                    }
                    // if triangle ...
                    //
                } else {
                    color = {0, 0, 0};
                }

                image.setPixel(i, j, color);
            }
        }

        std::cout << "writing..." << std::endl;
        image.writeFile(camera.image_name);
    }
}
