// shared basics: includes, window constants, small vector / matrix maths, orbit camera
#pragma once
#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#ifndef __APPLE__
#include <GL/glext.h>
#endif
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// config
const float WIDTH = 1000.f;
const float HEIGHT = 800.f;
const float FOCAL = 700.f;      // perspective strength in pixels at HEIGHT, same as the sphere sim
const float PI = 3.14159265f;

// ---------------------------------------------------------------------------------------------
// math, column-major like OpenGL and glTF
// ---------------------------------------------------------------------------------------------
struct vec3 {
    float x, y, z;
};

vec3 operator+(vec3 a, vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
vec3 operator-(vec3 a, vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
vec3 operator*(vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float length(vec3 a) { return std::sqrt(dot(a, a)); }
vec3 normalize(vec3 a) { return a * (1.f / length(a)); }
vec3 cross(vec3 a, vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

struct mat4 {
    std::array<float, 16> m{};  // m[col * 4 + row]
    float &at(int row, int col) { return m[col * 4 + row]; }
    float at(int row, int col) const { return m[col * 4 + row]; }
};

mat4 identity() {
    mat4 r;
    for (int k = 0; k < 4; k++) r.at(k, k) = 1.f;
    return r;
}

mat4 operator*(const mat4 &a, const mat4 &b) {
    mat4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.f;
            for (int k = 0; k < 4; k++) s += a.at(i, k) * b.at(k, j);
            r.at(i, j) = s;
        }
    return r;
}

vec3 transformPoint(const mat4 &a, vec3 p) {
    return {a.at(0, 0) * p.x + a.at(0, 1) * p.y + a.at(0, 2) * p.z + a.at(0, 3),
            a.at(1, 0) * p.x + a.at(1, 1) * p.y + a.at(1, 2) * p.z + a.at(1, 3),
            a.at(2, 0) * p.x + a.at(2, 1) * p.y + a.at(2, 2) * p.z + a.at(2, 3)};
}

mat4 translate(vec3 t) {
    mat4 r = identity();
    r.at(0, 3) = t.x;
    r.at(1, 3) = t.y;
    r.at(2, 3) = t.z;
    return r;
}

mat4 rotateX(float a) {
    mat4 r = identity();
    float c = std::cos(a), s = std::sin(a);
    r.at(1, 1) = c; r.at(1, 2) = -s;
    r.at(2, 1) = s; r.at(2, 2) = c;
    return r;
}

mat4 rotateZ(float a) {
    mat4 r = identity();
    float c = std::cos(a), s = std::sin(a);
    r.at(0, 0) = c; r.at(0, 1) = -s;
    r.at(1, 0) = s; r.at(1, 1) = c;
    return r;
}

mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    mat4 r;
    float f = 1.f / std::tan(fovY / 2.f);
    r.at(0, 0) = f / aspect;
    r.at(1, 1) = f;
    r.at(2, 2) = (zFar + zNear) / (zNear - zFar);
    r.at(2, 3) = 2.f * zFar * zNear / (zNear - zFar);
    r.at(3, 2) = -1.f;
    return r;
}

std::string numToStr(float num, int prec = 2) {
    std::ostringstream temp;
    temp << std::fixed << std::setprecision(prec) << num;
    return temp.str();
}

// ---------------------------------------------------------------------------------------------
// camera, same yaw/pitch/dist orbit and drag feel as the sphere sim
// ---------------------------------------------------------------------------------------------
struct camera {
    float yaw;
    float pitch;
    float dist;
};

// world -> GL eye space. Same rotation as toView() in the sphere sim, but that one maps into a
// left-handed view (z forward) which mirrors the scene. Flipping world z on the way in and eye z
// on the way out keeps the exact drag behaviour without mirroring the engine.
mat4 viewMatrix(const camera &cam, vec3 target) {
    float cy = std::cos(cam.yaw), sy = std::sin(cam.yaw);
    float cp = std::cos(cam.pitch), sp = std::sin(cam.pitch);
    // toView() with z -> -z:  x1 = x*cy + z*sy,  z1 = x*sy - z*cy,  y2 = y*cp + z1*sp,
    // z2 = -y*sp + z1*cp, and GL eye z = -(z2 + dist). The rows form a proper rotation (det +1).
    mat4 r = identity();
    r.at(0, 0) = cy;       r.at(0, 1) = 0.f;  r.at(0, 2) = sy;
    r.at(1, 0) = sy * sp;  r.at(1, 1) = cp;   r.at(1, 2) = -cy * sp;
    r.at(2, 0) = -sy * cp; r.at(2, 1) = sp;   r.at(2, 2) = cy * cp;
    r.at(2, 3) = -cam.dist;
    return r * translate(target * -1.f);
}

