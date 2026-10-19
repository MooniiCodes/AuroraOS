/* Vectors, quaternions and affine transforms in float. The ARM9 has no FPU,
 * so these run in software: fine for a few dozen matrices a frame, while
 * per-vertex work is fixed point (anim.c). */
#ifndef PLAYTEST_VMATH_H
#define PLAYTEST_VMATH_H

#include "game.h"

#include <math.h>

#define PI_F 3.14159265f

static inline V3 v3(float x, float y, float z) {
  V3 v = {x, y, z};
  return v;
}
static inline V3 v3_add(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 v3_sub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 v3_scale(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float v3_dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 v3_cross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline float v3_len(V3 a) { return sqrtf(v3_dot(a, a)); }
static inline V3 v3_norm(V3 a) {
  float l = v3_len(a);
  return l > 1e-12f ? v3_scale(a, 1.0f / l) : v3(0.0f, 1.0f, 0.0f);
}
static inline V3 v3_lerp(V3 a, V3 b, float t) {
  return v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t);
}

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

static inline Quat q_identity(void) {
  Quat q = {0.0f, 0.0f, 0.0f, 1.0f};
  return q;
}

/* a then b is q_mul(b, a): the product applies its right operand first. */
static inline Quat q_mul(Quat a, Quat b) {
  Quat q = {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
  return q;
}

static inline Quat q_conj(Quat a) {
  Quat q = {-a.x, -a.y, -a.z, a.w};
  return q;
}

static inline Quat q_norm(Quat a) {
  float l = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  Quat q;
  if (l < 1e-12f)
    return q_identity();
  l = 1.0f / l;
  q.x = a.x * l;
  q.y = a.y * l;
  q.z = a.z * l;
  q.w = a.w * l;
  return q;
}

/* `axis` must be a unit vector. */
static inline Quat q_axis(V3 axis, float angle) {
  float s = sinf(angle * 0.5f);
  Quat q = {axis.x * s, axis.y * s, axis.z * s, cosf(angle * 0.5f)};
  return q;
}

static inline V3 q_rotate(Quat q, V3 v) {
  V3 u = v3(q.x, q.y, q.z);
  V3 t = v3_scale(v3_cross(u, v), 2.0f);
  return v3_add(v3_add(v, v3_scale(t, q.w)), v3_cross(u, t));
}

/* Along the shorter arc, renormalised: close enough to slerp for keys a
 * frame or two apart. */
static inline Quat q_nlerp(Quat a, Quat b, float t) {
  float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  float s = d < 0.0f ? -t : t;
  Quat q = {a.x + (b.x * s - a.x * t), a.y + (b.y * s - a.y * t),
            a.z + (b.z * s - a.z * t), a.w + (b.w * s - a.w * t)};
  return q_norm(q);
}

void aff_identity(Aff *a);
void aff_trs(Aff *a, V3 t, Quat r, V3 s);
/* out = a * b; out may be either. */
void aff_mul(Aff *out, const Aff *a, const Aff *b);
V3 aff_point(const Aff *a, V3 p);
V3 aff_dir(const Aff *a, V3 v);
/* False for a singular matrix (out is then the identity). */
bool aff_inverse(Aff *out, const Aff *a);
/* The upper 3x3's inverse transpose, for normals. */
void aff_normal_matrix(float out[3][3], const Aff *a);
/* glTF's column-major 4x4. */
void aff_from_gltf(Aff *a, const float m[16]);
/* Translation, rotation and scale, assuming no shear. */
void aff_decompose(const Aff *a, V3 *t, Quat *r, V3 *s);
/* A rotation by `yaw` about +Y, then a move to `pos`. */
void aff_yaw(Aff *a, V3 pos, float yaw);

/* A fixed-point value with `frac` fraction bits as a float, by building the
 * bits: much cheaper than libgcc's conversion. */
static inline float fix2f(int32_t v, int frac) {
  union {
    uint32_t u;
    float f;
  } r;
  uint32_t a, s = 0;
  int lz;
  if (!v)
    return 0.0f;
  a = (uint32_t)v;
  if (v < 0) {
    s = 0x80000000u;
    a = 0u - a;
  }
  lz = __builtin_clz(a);
  a <<= lz; /* the leading one in bit 31 */
  r.u = s | ((uint32_t)(127 + 31 - lz - frac) << 23) | ((a >> 8) & 0x7FFFFFu);
  return r.f;
}

#endif
