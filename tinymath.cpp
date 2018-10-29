#include "tinymath.h"

#include <cmath>

namespace tinymath {

vec3f::vec3f(float x, float y, float z)
    : x(x), y(y), z(z) { }

vec3f & vec3f::operator+=(const vec3f & rhs) {

    raw4 = _mm_add_ps(this->raw4, rhs.raw4);
    //x += rhs.x;
    //y += rhs.y;
    //z += rhs.z;

    return *this;
}

vec3f operator+(vec3f lhs, const vec3f & rhs) {
    lhs += rhs;
    return lhs;
}

vec3f & vec3f::operator-=(const vec3f & rhs) {
    raw4 = _mm_sub_ps(this->raw4, rhs.raw4);
    //x -= rhs.x;
    //y -= rhs.y;
    //z -= rhs.z;

    return *this;
}

vec3f operator-(vec3f lhs, const vec3f & rhs) {
    lhs -= rhs;
    return lhs;
}

vec3f & vec3f::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;

    return *this;
}

vec3f operator*(vec3f lhs, float scalar) {
    lhs *= scalar;
    return lhs;
}

vec3f operator*(float scalar, vec3f rhs) {
    rhs *= scalar;
    return rhs;
}

vec3f & vec3f::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;

    return *this;
}

vec3f operator/(vec3f lhs, float scalar) {
    lhs /= scalar;
    return lhs;
}

float length(const vec3f & vec) {
    float result = std::sqrt( vec.x * vec.x
                            + vec.y * vec.y
                            + vec.z * vec.z );

    return result;
}

vec3f normalize(vec3f vec) {
    float vecLength = length(vec);
    if (vecLength == 0.0f)
        return vec3f(); // return zero vector

    vec /= vecLength;
    return vec;
}

float dot(const vec3f & lhs, const vec3f & rhs) {
    union {
        __m128 result4;
        float result[4];
    };
    result4 = _mm_mul_ps(lhs.raw4, rhs.raw4);
    return result[0] + result[1] + result[2];
    //return lhs.x * rhs.x
    //     + lhs.y * rhs.y
    //     + lhs.z * rhs.z;
}

vec3f cross(const vec3f & lhs, const vec3f & rhs) {
    vec3f result;
    result.x = lhs.y * rhs.z - lhs.z * rhs.y;
    result.y = lhs.z * rhs.x - lhs.x * rhs.z;
    result.z = lhs.x * rhs.y - lhs.y * rhs.x;
    return result;
}

} // namespace tinymath
