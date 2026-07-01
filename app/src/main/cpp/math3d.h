#pragma once
// Tiny column-major 4x4 matrix + quaternion helpers (no external libs).
// Hamilton quaternion convention (x,y,z,w), matching the ALVR/Pico pose frame.
#include <cmath>

struct Mat4 { float m[16]; };

inline Mat4 mat4Identity() {
    Mat4 r{}; r.m[0]=r.m[5]=r.m[10]=r.m[15]=1.0f; return r;
}

inline Mat4 mat4Mul(const Mat4 &a, const Mat4 &b) {
    Mat4 r{};
    for (int c = 0; c < 4; c++)
        for (int row = 0; row < 4; row++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += a.m[k*4+row] * b.m[c*4+k];
            r.m[c*4+row] = s;
        }
    return r;
}

inline Mat4 mat4Translate(float x, float y, float z) {
    Mat4 r = mat4Identity();
    r.m[12]=x; r.m[13]=y; r.m[14]=z; return r;
}

// Perspective, OpenGL convention (camera looks down -Z, clip z in [-1,1]).
inline Mat4 mat4Perspective(float fovyRad, float aspect, float zn, float zf) {
    Mat4 r{};
    float f = 1.0f / tanf(fovyRad * 0.5f);
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    r.m[10] = (zf + zn) / (zn - zf);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * zf * zn) / (zn - zf);
    return r;
}

// Rotation matrix from a unit quaternion (x,y,z,w).
inline Mat4 quatToMat4(float x, float y, float z, float w) {
    Mat4 r = mat4Identity();
    float xx=x*x, yy=y*y, zz=z*z, xy=x*y, xz=x*z, yz=y*z, wx=w*x, wy=w*y, wz=w*z;
    r.m[0]=1-2*(yy+zz); r.m[1]=2*(xy+wz);   r.m[2]=2*(xz-wy);
    r.m[4]=2*(xy-wz);   r.m[5]=1-2*(xx+zz); r.m[6]=2*(yz+wx);
    r.m[8]=2*(xz+wy);   r.m[9]=2*(yz-wx);   r.m[10]=1-2*(xx+yy);
    return r;
}

inline Mat4 mat4Transpose3x3(const Mat4 &a) {  // inverse of a pure rotation
    Mat4 r = a;
    r.m[1]=a.m[4]; r.m[4]=a.m[1];
    r.m[2]=a.m[8]; r.m[8]=a.m[2];
    r.m[6]=a.m[9]; r.m[9]=a.m[6];
    return r;
}

// Quaternion helpers for rotational TimeWarp. Hamilton convention (x,y,z,w),
// matching quatToMat4 above and the ALVR/Pico pose frame.
struct Quat { float x, y, z, w; };
inline Quat quatConj(Quat q) { return { -q.x, -q.y, -q.z, q.w }; }
inline Quat quatMul(Quat a, Quat b) {   // a * b
    return {
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    };
}
// Fill a column-major 3x3 (9 floats, for glUniformMatrix3fv) from a quaternion.
inline void quatToMat3(Quat q, float *m) {
    Mat4 r = quatToMat4(q.x, q.y, q.z, q.w);   // already column-major
    m[0]=r.m[0]; m[1]=r.m[1]; m[2]=r.m[2];
    m[3]=r.m[4]; m[4]=r.m[5]; m[5]=r.m[6];
    m[6]=r.m[8]; m[7]=r.m[9]; m[8]=r.m[10];
}
inline Quat quatNorm(Quat q) {
    float n = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (n < 1e-9f) return { 0,0,0,1 };
    return { q.x/n, q.y/n, q.z/n, q.w/n };
}
// Rotate vector v by quaternion q (v' = q v q*). out may alias v safely (uses temps).
inline void quatRotateVec(Quat q, const float v[3], float out[3]) {
    float tx = 2.0f * (q.y*v[2] - q.z*v[1]);
    float ty = 2.0f * (q.z*v[0] - q.x*v[2]);
    float tz = 2.0f * (q.x*v[1] - q.y*v[0]);
    float ox = v[0] + q.w*tx + (q.y*tz - q.z*ty);
    float oy = v[1] + q.w*ty + (q.z*tx - q.x*tz);
    float oz = v[2] + q.w*tz + (q.x*ty - q.y*tx);
    out[0] = ox; out[1] = oy; out[2] = oz;
}
// Scale the rotation angle of a quaternion by k (axis preserved). Used to
// extrapolate the per-sample rotation step forward (k = predict/dt).
inline Quat quatScaleAngle(Quat q, float k) {
    q = quatNorm(q);
    if (q.w < 0) { q.x=-q.x; q.y=-q.y; q.z=-q.z; q.w=-q.w; }  // shortest arc
    float w = q.w > 1.0f ? 1.0f : q.w;
    float ang = 2.0f * acosf(w);
    float s = sqrtf(1.0f - w*w);
    if (s < 1e-6f) return { 0,0,0,1 };                        // ~no rotation
    float na = ang * k * 0.5f;
    float sn = sinf(na) / s;
    return { q.x*sn, q.y*sn, q.z*sn, cosf(na) };
}

