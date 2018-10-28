#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <optional>

#include "parser.h"
#include "ppmimage.h"
#include "tinymath.h"

const float infinity = std::numeric_limits<float>::max();

struct imageConfig {
    float left;
    float right;
    float bottom;
    float top;
    float widthPerPixel;
    float heightPerPixel;
    tinymath::vec3f topLeft;
};

struct Ray {
    tinymath::vec3f origin;
    tinymath::vec3f direction;
    float t;
};

struct intersectData {
    float t;
    tinymath::vec3f normal;
};

parser::Box calculateBoundingBox(const parser::Mesh & mesh, const std::vector<tinymath::vec3f> & vertex_data) {
    parser::Box boundingBox;

    for (const auto &face : mesh.faces) {
        tinymath::vec3f vertexA = vertex_data[face.v0_id - 1];
        tinymath::vec3f vertexB = vertex_data[face.v1_id - 1];
        tinymath::vec3f vertexC = vertex_data[face.v2_id - 1];

        if (vertexA.x > boundingBox.maxExtent.x)
            boundingBox.maxExtent.x = vertexA.x;

        if (vertexA.y > boundingBox.maxExtent.y)
            boundingBox.maxExtent.y = vertexA.y;

        if (vertexA.z > boundingBox.maxExtent.z)
            boundingBox.maxExtent.z = vertexA.z;

        if (vertexA.x < boundingBox.minExtent.x)
            boundingBox.minExtent.x = vertexA.x;

        if (vertexA.y < boundingBox.minExtent.y)
            boundingBox.minExtent.y = vertexA.y;

        if (vertexA.z < boundingBox.minExtent.z)
            boundingBox.minExtent.z = vertexA.z;

        if (vertexB.x > boundingBox.maxExtent.x)
            boundingBox.maxExtent.x = vertexB.x;

        if (vertexB.y > boundingBox.maxExtent.y)
            boundingBox.maxExtent.y = vertexB.y;

        if (vertexB.z > boundingBox.maxExtent.z)
            boundingBox.maxExtent.z = vertexB.z;

        if (vertexB.x < boundingBox.minExtent.x)
            boundingBox.minExtent.x = vertexB.x;

        if (vertexB.y < boundingBox.minExtent.y)
            boundingBox.minExtent.y = vertexB.y;

        if (vertexB.z < boundingBox.minExtent.z)
            boundingBox.minExtent.z = vertexB.z;

        if (vertexC.x > boundingBox.maxExtent.x)
            boundingBox.maxExtent.x = vertexC.x;

        if (vertexC.y > boundingBox.maxExtent.y)
            boundingBox.maxExtent.y = vertexC.y;

        if (vertexC.z > boundingBox.maxExtent.z)
            boundingBox.maxExtent.z = vertexC.z;

        if (vertexC.x < boundingBox.minExtent.x)
            boundingBox.minExtent.x = vertexC.x;

        if (vertexC.y < boundingBox.minExtent.y)
            boundingBox.minExtent.y = vertexC.y;

        if (vertexC.z < boundingBox.minExtent.z)
            boundingBox.minExtent.z = vertexC.z;

    }

    return boundingBox;
}

tinymath::vec3f clamp(tinymath::vec3f v) {
    v.x = std::min(std::max(0.0f, v.x), 255.0f);
    v.y = std::min(std::max(0.0f, v.y), 255.0f);
    v.z = std::min(std::max(0.0f, v.z), 255.0f);
    return v;
}

bool intersect(const Ray &ray, const parser::Box& box) {

    float tmin = (box.minExtent.x - ray.origin.x) / ray.direction.x;
    float tmax = (box.maxExtent.x - ray.origin.x) / ray.direction.x;

    if (tmin > tmax)
        std::swap(tmin, tmax);

    float tymin = (box.minExtent.y - ray.origin.y) / ray.direction.y;
    float tymax = (box.maxExtent.y - ray.origin.y) / ray.direction.y;

    if (tymin > tymax) std::swap(tymin, tymax);

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;

    if (tymax < tmax)
        tmax = tymax;

    float tzmin = (box.minExtent.z - ray.origin.z) / ray.direction.z;
    float tzmax = (box.maxExtent.z - ray.origin.z) / ray.direction.z;

    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;

    if (tzmax < tmax)
        tmax = tzmax;

    return true; 
}

std::optional<intersectData>
intersect(const Ray &ray, const parser::Sphere &sphere,
          const std::vector<tinymath::vec3f> &vertex_data) {
    tinymath::vec3f sphereCenter = vertex_data[sphere.center_vertex_id - 1];

    float fst = tinymath::dot(ray.direction, ray.origin - sphereCenter);
    float snd = tinymath::dot(ray.direction, ray.direction);
    float thrd =
        tinymath::dot(ray.origin - sphereCenter, ray.origin - sphereCenter) -
        sphere.radius * sphere.radius;
    float determinant = fst * fst - snd * thrd;

    const float almostZero = 1e-6;
    if (determinant < almostZero)
        return std::nullopt;
    //if (determinant < 0) {
    //    return std::nullopt;
    //}

    float t = (-tinymath::dot(ray.direction, (ray.origin - sphereCenter)) -
               std::sqrt(determinant)) /
              snd;

    if (t < almostZero)
        return std::nullopt;
    
    intersectData result;
    result.t = t;
    result.normal =
        tinymath::normalize((ray.origin + t * ray.direction) - sphereCenter);
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

    tinymath::vec3f pvec = tinymath::cross(ray.direction, edgeBA);
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
    float v = tinymath::dot(ray.direction, qvec) * inverseDeterminant;
    if (v < 0 || u + v > 1)
        return std::nullopt;

    float t = tinymath::dot(edgeBA, qvec) * inverseDeterminant;

    // May move this check up inorder to speed things up
    // not sure if it would speed up or slow down though
    if (t < almostZero)
        return std::nullopt;

    intersectData result;
    result.t = t;
    result.normal = triangle.normal; 
    return result;
}

struct traceData {
    tinymath::vec3f normal;
    int material;
    float t;
};

traceData trace(const Ray & ray, const parser::Scene & scene) {
    float t = infinity;
    int currentMaterial = 0;
    tinymath::vec3f currentNormal;

    for (const auto &sphere : scene.spheres) {
        std::optional<intersectData> intersected =
            intersect(ray, sphere, scene.vertex_data);
        if (intersected) {
            intersectData intersection = intersected.value();
            float t_current = intersection.t;
            if (t_current < t) {
                t = t_current;
                currentNormal = intersection.normal;
                currentMaterial = sphere.material_id;
            }
        }
    }

    for (const auto &triangle : scene.triangles) {
        std::optional<intersectData> intersected = intersect(
                ray, triangle.indices, scene.vertex_data);
        if (intersected) {
            intersectData intersection = intersected.value();
            float t_current = intersection.t;
            if (t_current < t) {
                t = t_current;
                currentNormal = intersection.normal;
                currentMaterial = triangle.material_id;
            }
        }
    }

    for (const auto &mesh : scene.meshes) {
        
        if(intersect(ray, mesh.boundingBox) == false)
            continue;

        for (const auto &face : mesh.faces) {
            std::optional<intersectData> intersected =
                intersect(ray, face, scene.vertex_data);
            if (intersected) {
                intersectData intersection = intersected.value();
                float t_current = intersection.t;
                if (t_current < t) {
                    t = t_current;
                    currentNormal = intersection.normal;
                    currentMaterial = mesh.material_id;
                }
            }
        }
    }

    traceData result;
    result.t = t;
    result.material = currentMaterial;
    result.normal = currentNormal;
    return result;
}

tinymath::vec3f calculateAmbient(const parser::Material & material, const tinymath::vec3f & ambient) {

    tinymath::vec3f calculated;
    calculated.x = ambient.x * material.ambient.x;
    calculated.y= ambient.y * material.ambient.y;
    calculated.z = ambient.z * material.ambient.z;
    return calculated;    
}

tinymath::vec3f calculateDiffuse(const parser::Material & material,
                                 const tinymath::vec3f & lightDirection,
                                 const tinymath::vec3f & intersectionNormal,
                                 const tinymath::vec3f & lightIntensity) {

    float cosTheta = std::max(0.0f, tinymath::dot(tinymath::normalize(lightDirection), intersectionNormal));
    float lightDistance = tinymath::length(lightDirection);
    float cosThetaOverDistanceSquared = cosTheta / (lightDistance * lightDistance);

    tinymath::vec3f calculated;
    calculated.x = (lightIntensity.x * material.diffuse.x) * cosThetaOverDistanceSquared;
    calculated.y = (lightIntensity.y * material.diffuse.y) * cosThetaOverDistanceSquared;
    calculated.z = (lightIntensity.z * material.diffuse.z) * cosThetaOverDistanceSquared;

    return calculated;
}

tinymath::vec3f calculateSpecular(const parser::Material & material,
                                 const tinymath::vec3f & lightDirection,
                                 const tinymath::vec3f & intersectionNormal,
                                 const tinymath::vec3f & lightIntensity,
                                 const tinymath::vec3f & toEye) {

    tinymath::vec3f halfVector = tinymath::normalize(tinymath::normalize(lightDirection)
                                                     + tinymath::normalize(toEye));
    float cosAlpha = std::max(0.0f, tinymath::dot(intersectionNormal, halfVector));
    float lightDistance = tinymath::length(lightDirection);
    float cosAlphaWithPhongAndR = std::pow(cosAlpha, material.phong_exponent) / (lightDistance * lightDistance);

    tinymath::vec3f calculated;
    calculated.x = (lightIntensity.x * material.specular.x) * cosAlphaWithPhongAndR;  
    calculated.y = (lightIntensity.y * material.specular.y) * cosAlphaWithPhongAndR;  
    calculated.z = (lightIntensity.z * material.specular.z) * cosAlphaWithPhongAndR;  

    return calculated;
}

Ray castShadowRay(const tinymath::vec3f & direction, const tinymath::vec3f & origin, float epsilon) {
    // this function expects normalized direction
    Ray shadowRay;
    shadowRay.direction = direction;
    tinymath::vec3f offset =  epsilon * direction;
    shadowRay.origin = origin + offset;
    return shadowRay;
}

tinymath::vec3f calculatePhong(const Ray & ray, const traceData & intersection, const parser::Scene & scene) {

    parser::Material material = scene.materials[intersection.material - 1];
    tinymath::vec3f intersectionPoint = ray.origin +
        ray.direction * intersection.t;
    tinymath::vec3f toEye = ray.origin - intersectionPoint;

    tinymath::vec3f phong;
    tinymath::vec3f totalDiffuse;
    tinymath::vec3f totalSpecular;

    // apply ambient to everywhere
    tinymath::vec3f ambient = calculateAmbient(material, scene.ambient_light);
    phong = ambient;

    for (const auto & light : scene.point_lights) {

        tinymath::vec3f lightDirection =
            light.position - intersectionPoint;

        Ray shadowRay =
            castShadowRay(tinymath::normalize(lightDirection), intersectionPoint, scene.shadow_ray_epsilon);
        auto shadowIntersect = trace(shadowRay, scene);

        // calculate diffuse and specular only if point is not in the shadow
        if (shadowIntersect.t >= tinymath::length(lightDirection)) {

            tinymath::vec3f diffuse = calculateDiffuse(material, lightDirection,
                    intersection.normal, light.intensity);
            totalDiffuse += diffuse;

            tinymath::vec3f specular = calculateSpecular(material, lightDirection,
                    intersection.normal, light.intensity,
                    toEye);
            totalSpecular += specular;

        }
    }

    phong += totalSpecular;
    phong += totalDiffuse;
    phong = clamp(phong);

    return phong;
}

PPMColor calculateColor(const Ray & ray, const traceData & intersection, const parser::Scene & scene) {
    PPMColor color;
    if (intersection.t != infinity) { // ray hit an object

        tinymath::vec3f phong = calculatePhong(ray, intersection, scene);

        color = { static_cast<int>(phong.x),
                  static_cast<int>(phong.y),
                  static_cast<int>(phong.z) };

    } else {
        color = { scene.background_color.x,
                  scene.background_color.y,
                  scene.background_color.z };
    }
    
    return color;
}

void putPixel(int x, int y, PPMImage & image, const imageConfig & config,
              const parser::Camera & camera, const parser::Scene & scene) {

    float uOffset = config.widthPerPixel * (x + 0.5f);
    float vOffset = config.heightPerPixel * (y + 0.5f);
    tinymath::vec3f right = tinymath::cross(camera.up, -1 * camera.gaze);

    Ray rayToPixel;
    rayToPixel.origin = camera.position;
    rayToPixel.direction =
        (config.topLeft + uOffset * right - vOffset * camera.up) -
        rayToPixel.origin;
    rayToPixel.t = tinymath::length(rayToPixel.direction);
    rayToPixel.direction = tinymath::normalize(rayToPixel.direction);

    auto intersection = trace(rayToPixel, scene);
    auto color = calculateColor(rayToPixel, intersection, scene);
    image.setPixel(x, y, color);
}

void createImage(const parser::Camera & camera, const parser::Scene & scene) {

    int width = camera.image_width;
    int height = camera.image_height;

    PPMImage image(width, height);

    imageConfig config;

    config.left   = camera.near_plane.x;
    config.right  = camera.near_plane.y;
    config.bottom = camera.near_plane.z;
    config.top    = camera.near_plane.w;
    config.widthPerPixel = (config.right - config.left) / width;
    config.heightPerPixel = (config.top - config.bottom) / height;

    tinymath::vec3f right = tinymath::cross(camera.up, -1 * camera.gaze);

    config.topLeft = camera.position + camera.gaze * camera.near_distance 
                     + camera.up * config.top + right * config.left;

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            putPixel(i, j, image, config, camera, scene);
        }
    }

    std::cout << "writing to file..." << std::endl;
    image.writeFile(camera.image_name);
}

void calculateNormal(parser::Face & face, const std::vector<tinymath::vec3f> & vertex_data) {

    tinymath::vec3f vertexA = vertex_data[face.v0_id - 1];
    tinymath::vec3f vertexB = vertex_data[face.v1_id - 1];
    tinymath::vec3f vertexC = vertex_data[face.v2_id - 1];

    tinymath::vec3f edgeBC = vertexC - vertexB;
    tinymath::vec3f edgeBA = vertexA - vertexB;

    face.normal = tinymath::normalize(tinymath::cross(edgeBC, edgeBA));
}

void calculateAllNormals(parser::Scene & scene) {

    for (auto & mesh : scene.meshes)
        for (auto & face : mesh.faces)
            calculateNormal(face, scene.vertex_data);

    for (auto & triangle : scene.triangles)
        calculateNormal(triangle.indices, scene.vertex_data);
}


int main(int argc, char *argv[]) {
    parser::Scene scene;

    scene.loadFromXml("../hw1_sample_scenes/bunny.xml");

    std::cout << "calculating bounding boxes of the meshes..." << std::endl;
    for (auto & mesh : scene.meshes)
        mesh.boundingBox = calculateBoundingBox(mesh, scene.vertex_data);
    
    std::cout << "pre-computing normals of faces..." << std::endl;
    calculateAllNormals(scene);

    for (const auto &camera : scene.cameras) {
        std::cout << "raytracing..." << std::endl;
        createImage(camera, scene);
    }
        
}
