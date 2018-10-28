#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <optional>

#include "parser.h"
#include "ppmimage.h"
#include "tinymath.h"

struct Ray {
    tinymath::vec3f origin;
    tinymath::vec3f direction;
};

struct intersectData {
    float t;
    tinymath::vec3f normal;
};

tinymath::vec3f clamp(tinymath::vec3f v) {
    v.x = std::min(std::max(0.0f, v.x), 255.0f);
    v.y = std::min(std::max(0.0f, v.y), 255.0f);
    v.z = std::min(std::max(0.0f, v.z), 255.0f);
    return v;
}

std::optional<intersectData>
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

    intersectData result;
    result.t = t;
    result.normal =
        tinymath::normalize((ray.origin + t * direction) - sphereCenter);
    return result;
}

std::optional<intersectData>
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

    // May move this check up inorder to speed things up
    // not sure if it would speed up or slow down though
    if (t < 0)
        return std::nullopt;

    // TODO there must be a precomputed normal somewhere here...
    intersectData result;
    result.t = t;
    result.normal = tinymath::normalize(tinymath::cross(edgeBC, edgeBA));
    return result;
}

int main(int argc, char *argv[]) {
    parser::Scene scene;

    scene.loadFromXml("../hw1_sample_scenes/simple_shading.xml");

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
                int currentMaterial = 0;
                tinymath::vec3f currentNormal;

                for (const auto &sphere : scene.spheres) {
                    std::optional<intersectData> intersected =
                        intersect(rayToPixel, sphere, scene.vertex_data);
                    if (intersected) {
                        intersectData intersection = intersected.value();
                        float t_current = intersection.t;
                        if (t_current < t) {
                            t = t_current;
                            currentNormal = intersection.normal;
                            minSphere = &sphere;
                            minObject = ObjectType::SPHERE;
                            currentMaterial = sphere.material_id;
                        }
                    }
                }

                for (const auto &triangle : scene.triangles) {
                    std::optional<intersectData> intersected = intersect(
                        rayToPixel, triangle.indices, scene.vertex_data);
                    if (intersected) {
                        intersectData intersection = intersected.value();
                        float t_current = intersection.t;
                        if (t_current < t) {
                            t = t_current;
                            currentNormal = intersection.normal;
                            minFace = &triangle.indices;
                            minObject = ObjectType::TRIANGLE;
                            currentMaterial = triangle.material_id;
                        }
                    }
                }

                for (const auto &mesh : scene.meshes) {
                    for (const auto &face : mesh.faces) {
                        std::optional<intersectData> intersected =
                            intersect(rayToPixel, face, scene.vertex_data);
                        if (intersected) {
                            intersectData intersection = intersected.value();
                            float t_current = intersection.t;
                            if (t_current < t) {
                                t = t_current;
                                currentNormal = intersection.normal;
                                minFace = &face;
                                minObject = ObjectType::TRIANGLE;
                                currentMaterial = mesh.material_id;
                            }
                        }
                    }
                }

                PPMColor color;
                if (t != infinity) { // ray hit an object
                    parser::Material material =
                        scene.materials[currentMaterial - 1];

                    tinymath::vec3f intersectionPoint =
                        rayToPixel.origin +
                        tinymath::normalize(rayToPixel.direction) * t;

                    tinymath::vec3f toEye = rayToPixel.origin - intersectionPoint;

                    tinymath::vec3f totalDiffuse;
                    tinymath::vec3f totalSpecular;

                    for (const auto & light : scene.point_lights) {

                        tinymath::vec3f lightDirection =
                            light.position - intersectionPoint;

                        float cosTheta = std::max(
                                0.0f, tinymath::dot(tinymath::normalize(lightDirection),
                                    currentNormal));
                        float lightDistance = tinymath::length(lightDirection);
                        tinymath::vec3f lightIntensity =
                            light.intensity;

                        float cosThetaOverDistanceSquared =
                            cosTheta / (lightDistance * lightDistance);

                        tinymath::vec3f diffuse;
                        diffuse.x = (lightIntensity.x * material.diffuse.x) *
                            cosThetaOverDistanceSquared;
                        diffuse.y = (lightIntensity.y * material.diffuse.y) *
                            cosThetaOverDistanceSquared;
                        diffuse.z = (lightIntensity.z * material.diffuse.z) *
                            cosThetaOverDistanceSquared;

                        totalDiffuse += diffuse;

                        tinymath::vec3f specular;
                        tinymath::vec3f halfVector = tinymath::normalize(
                                                     tinymath::normalize(lightDirection) +
                                                     tinymath::normalize(toEye));
                        float cosAlpha = std::max(0.0f, tinymath::dot(currentNormal, halfVector));
                        float cosAlphaWithPhongAndR = std::pow(cosAlpha, material.phong_exponent) / (lightDistance * lightDistance);

                        specular.x = (lightIntensity.x * material.specular.x) * cosAlphaWithPhongAndR;  
                        specular.y = (lightIntensity.y * material.specular.y) * cosAlphaWithPhongAndR;  
                        specular.z = (lightIntensity.z * material.specular.z) * cosAlphaWithPhongAndR;  

                        totalSpecular += specular;
                    }


                    tinymath::vec3f ambient;
                    ambient.x = scene.ambient_light.x * material.ambient.x;
                    ambient.y= scene.ambient_light.y * material.ambient.y;
                    ambient.z = scene.ambient_light.z * material.ambient.z;



                    tinymath::vec3f phong = clamp(totalDiffuse + ambient + totalSpecular);



                    color = {static_cast<int>(phong.x),
                             static_cast<int>(phong.y),
                             static_cast<int>(phong.z)};
#if 0
                    if (minObject == ObjectType::SPHERE) {
                        color = {255, 255, 255};
                    }
                    if (minObject == ObjectType::TRIANGLE) {
                        color = {255, 255, 255};
                    }
#endif
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
