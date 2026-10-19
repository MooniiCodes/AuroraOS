#include "vmath.h"

void aff_identity(Aff *a) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 4; c++)
      a->m[r][c] = r == c ? 1.0f : 0.0f;
}

void aff_trs(Aff *a, V3 t, Quat q, V3 s) {
  float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

  a->m[0][0] = (1.0f - 2.0f * (yy + zz)) * s.x;
  a->m[0][1] = 2.0f * (xy - wz) * s.y;
  a->m[0][2] = 2.0f * (xz + wy) * s.z;
  a->m[1][0] = 2.0f * (xy + wz) * s.x;
  a->m[1][1] = (1.0f - 2.0f * (xx + zz)) * s.y;
  a->m[1][2] = 2.0f * (yz - wx) * s.z;
  a->m[2][0] = 2.0f * (xz - wy) * s.x;
  a->m[2][1] = 2.0f * (yz + wx) * s.y;
  a->m[2][2] = (1.0f - 2.0f * (xx + yy)) * s.z;
  a->m[0][3] = t.x;
  a->m[1][3] = t.y;
  a->m[2][3] = t.z;
}

void aff_mul(Aff *out, const Aff *a, const Aff *b) {
  Aff t;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 4; c++)
      t.m[r][c] = a->m[r][0] * b->m[0][c] + a->m[r][1] * b->m[1][c] +
                  a->m[r][2] * b->m[2][c];
    t.m[r][3] += a->m[r][3];
  }
  *out = t;
}

V3 aff_point(const Aff *a, V3 p) {
  return v3(a->m[0][0] * p.x + a->m[0][1] * p.y + a->m[0][2] * p.z + a->m[0][3],
            a->m[1][0] * p.x + a->m[1][1] * p.y + a->m[1][2] * p.z + a->m[1][3],
            a->m[2][0] * p.x + a->m[2][1] * p.y + a->m[2][2] * p.z + a->m[2][3]);
}

V3 aff_dir(const Aff *a, V3 v) {
  return v3(a->m[0][0] * v.x + a->m[0][1] * v.y + a->m[0][2] * v.z,
            a->m[1][0] * v.x + a->m[1][1] * v.y + a->m[1][2] * v.z,
            a->m[2][0] * v.x + a->m[2][1] * v.y + a->m[2][2] * v.z);
}

/* Cofactors of the upper 3x3: c[i][j] is the cofactor of element (i, j). */
static float cofactors(float c[3][3], const Aff *a) {
  const float(*m)[4] = a->m;
  c[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
  c[0][1] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
  c[0][2] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
  c[1][0] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
  c[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
  c[1][2] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
  c[2][0] = m[0][1] * m[1][2] - m[0][2] * m[1][1];
  c[2][1] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
  c[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  return m[0][0] * c[0][0] + m[0][1] * c[0][1] + m[0][2] * c[0][2];
}

bool aff_inverse(Aff *out, const Aff *a) {
  float c[3][3], det = cofactors(c, a), inv;
  Aff t;
  if (det > -1e-20f && det < 1e-20f) {
    aff_identity(out);
    return false;
  }
  inv = 1.0f / det;
  /* The inverse is the transposed cofactors over the determinant. */
  for (int r = 0; r < 3; r++)
    for (int k = 0; k < 3; k++)
      t.m[r][k] = c[k][r] * inv;
  for (int r = 0; r < 3; r++)
    t.m[r][3] = -(t.m[r][0] * a->m[0][3] + t.m[r][1] * a->m[1][3] +
                  t.m[r][2] * a->m[2][3]);
  *out = t;
  return true;
}

void aff_normal_matrix(float out[3][3], const Aff *a) {
  float c[3][3], det = cofactors(c, a);
  float s = det < 0.0f ? -1.0f : 1.0f; /* the scale is renormalised anyway */
  for (int r = 0; r < 3; r++)
    for (int k = 0; k < 3; k++)
      out[r][k] = c[r][k] * s;
}

void aff_from_gltf(Aff *a, const float m[16]) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 4; c++)
      a->m[r][c] = m[c * 4 + r];
}

void aff_decompose(const Aff *a, V3 *t, Quat *r, V3 *s) {
  float m[3][3], tr;
  V3 sc;
  Quat q;

  *t = v3(a->m[0][3], a->m[1][3], a->m[2][3]);
  sc.x = sqrtf(a->m[0][0] * a->m[0][0] + a->m[1][0] * a->m[1][0] +
               a->m[2][0] * a->m[2][0]);
  sc.y = sqrtf(a->m[0][1] * a->m[0][1] + a->m[1][1] * a->m[1][1] +
               a->m[2][1] * a->m[2][1]);
  sc.z = sqrtf(a->m[0][2] * a->m[0][2] + a->m[1][2] * a->m[1][2] +
               a->m[2][2] * a->m[2][2]);
  {
    float c[3][3];
    if (cofactors(c, a) < 0.0f)
      sc.x = -sc.x; /* a mirror: put it on X */
  }
  for (int i = 0; i < 3; i++) {
    m[i][0] = sc.x != 0.0f ? a->m[i][0] / sc.x : 0.0f;
    m[i][1] = sc.y != 0.0f ? a->m[i][1] / sc.y : 0.0f;
    m[i][2] = sc.z != 0.0f ? a->m[i][2] / sc.z : 0.0f;
  }
  /* Shepperd's method: divide by the largest of the four candidates. */
  tr = m[0][0] + m[1][1] + m[2][2];
  if (tr > 0.0f) {
    float k = sqrtf(tr + 1.0f) * 2.0f;
    q.w = 0.25f * k;
    q.x = (m[2][1] - m[1][2]) / k;
    q.y = (m[0][2] - m[2][0]) / k;
    q.z = (m[1][0] - m[0][1]) / k;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    float k = sqrtf(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
    q.w = (m[2][1] - m[1][2]) / k;
    q.x = 0.25f * k;
    q.y = (m[0][1] + m[1][0]) / k;
    q.z = (m[0][2] + m[2][0]) / k;
  } else if (m[1][1] > m[2][2]) {
    float k = sqrtf(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
    q.w = (m[0][2] - m[2][0]) / k;
    q.x = (m[0][1] + m[1][0]) / k;
    q.y = 0.25f * k;
    q.z = (m[1][2] + m[2][1]) / k;
  } else {
    float k = sqrtf(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
    q.w = (m[1][0] - m[0][1]) / k;
    q.x = (m[0][2] + m[2][0]) / k;
    q.y = (m[1][2] + m[2][1]) / k;
    q.z = 0.25f * k;
  }
  *r = q_norm(q);
  *s = sc;
}

void aff_yaw(Aff *a, V3 pos, float yaw) {
  float c = cosf(yaw), s = sinf(yaw);
  a->m[0][0] = c;
  a->m[0][1] = 0.0f;
  a->m[0][2] = s;
  a->m[0][3] = pos.x;
  a->m[1][0] = 0.0f;
  a->m[1][1] = 1.0f;
  a->m[1][2] = 0.0f;
  a->m[1][3] = pos.y;
  a->m[2][0] = -s;
  a->m[2][1] = 0.0f;
  a->m[2][2] = c;
  a->m[2][3] = pos.z;
}
