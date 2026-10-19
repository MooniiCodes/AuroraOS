/* Collision against the room's triangles. A grid over X and Z narrows each
 * query to nearby triangles, their integer bounds reject most of those, and
 * only the rest get the float tests (closest point, ray), which are slow on
 * the ARM9's soft float. Floors are triangles facing up or down within 53
 * degrees of vertical (meshes are double-sided, so either winding counts);
 * everything else is a wall. */
#include "vmath.h"

#include <stdlib.h>
#include <string.h>

#define CELL_MIN  0.5f
#define CELLS_MAX 160 /* along X or Z */
#define CAND_MAX  4096
#define WALKABLE  0.6f /* |normal.y| */

static uint32_t cand[CAND_MAX];

static int cell_x(const World *w, float x) {
  int c = (int)((x - w->x0) / w->cell);
  return c < 0 ? 0 : c >= w->nx ? w->nx - 1 : c;
}

static int cell_z(const World *w, float z) {
  int c = (int)((z - w->z0) / w->cell);
  return c < 0 ? 0 : c >= w->nz ? w->nz - 1 : c;
}

bool world_build(World *w) {
  float ex = w->bmax.x - w->bmin.x, ez = w->bmax.z - w->bmin.z;
  uint32_t ncell, total = 0;
  uint32_t *fill;

  w->cell = CELL_MIN;
  if (ex / w->cell > CELLS_MAX - 2)
    w->cell = ex / (CELLS_MAX - 2);
  if (ez / w->cell > CELLS_MAX - 2)
    w->cell = ez / (CELLS_MAX - 2);
  w->x0 = w->bmin.x - w->cell * 0.5f;
  w->z0 = w->bmin.z - w->cell * 0.5f;
  w->nx = (int)(ex / w->cell) + 2;
  w->nz = (int)(ez / w->cell) + 2;
  ncell = (uint32_t)(w->nx * w->nz);

  w->start = calloc(ncell + 1u, sizeof(uint32_t));
  w->stamp = calloc(w->ntri ? w->ntri : 1u, sizeof(uint32_t));
  fill = calloc(ncell, sizeof(uint32_t));
  if (!w->start || !w->stamp || !fill) {
    free(fill);
    return false;
  }
  for (int pass = 0; pass < 2; pass++) {
    for (uint32_t i = 0; i < w->ntri; i++) {
      const CTri *t = &w->tri[i];
      int x0 = cell_x(w, t->lo[0] / 256.0f), x1 = cell_x(w, t->hi[0] / 256.0f);
      int z0 = cell_z(w, t->lo[2] / 256.0f), z1 = cell_z(w, t->hi[2] / 256.0f);
      for (int z = z0; z <= z1; z++)
        for (int x = x0; x <= x1; x++) {
          uint32_t c = (uint32_t)(z * w->nx + x);
          if (pass)
            w->list[w->start[c] + fill[c]++] = i;
          else
            w->start[c + 1u]++;
        }
    }
    if (pass)
      break;
    for (uint32_t c = 0; c < ncell; c++)
      w->start[c + 1u] += w->start[c];
    total = w->start[ncell];
    w->list = malloc((total ? total : 1u) * sizeof(uint32_t));
    if (!w->list) {
      free(fill);
      return false;
    }
  }
  free(fill);
  w->now = 0;
  return true;
}

static int16_t lo8(float v) {
  float f = floorf(v * 256.0f);
  return (int16_t)clampf(f, -32767.0f, 32767.0f);
}

static int16_t hi8(float v) {
  float f = ceilf(v * 256.0f);
  return (int16_t)clampf(f, -32767.0f, 32767.0f);
}

static void stamp_next(World *w) {
  if (++w->now == 0u) {
    memset(w->stamp, 0, w->ntri * sizeof(uint32_t));
    w->now = 1u;
  }
}

/* Adds the triangles of the cells under the box lo..hi (1/256 m) whose
 * bounds meet it and that this query has not taken yet. */
static uint32_t gather(World *w, const int16_t lo[3], const int16_t hi[3],
                       uint32_t n) {
  int cx0 = cell_x(w, lo[0] / 256.0f), cx1 = cell_x(w, hi[0] / 256.0f);
  int cz0 = cell_z(w, lo[2] / 256.0f), cz1 = cell_z(w, hi[2] / 256.0f);
  for (int z = cz0; z <= cz1; z++)
    for (int x = cx0; x <= cx1; x++) {
      uint32_t c = (uint32_t)(z * w->nx + x);
      for (uint32_t k = w->start[c]; k < w->start[c + 1u]; k++) {
        uint32_t i = w->list[k];
        const CTri *t = &w->tri[i];
        /* Stamped once taken, so a later box of the same query (the ray's
         * pieces) can still take one this box missed. */
        if (w->stamp[i] == w->now || t->hi[0] < lo[0] || t->lo[0] > hi[0] ||
            t->hi[1] < lo[1] || t->lo[1] > hi[1] || t->hi[2] < lo[2] ||
            t->lo[2] > hi[2])
          continue;
        w->stamp[i] = w->now;
        if (n < CAND_MAX)
          cand[n++] = i;
      }
    }
  return n;
}

static bool walkable(const CTri *t) { return fabsf(t->n.y) >= WALKABLE; }

/* The height of t's plane over (x, z), if (x, z) is inside t seen from
 * above. */
static bool height_at(const CTri *t, float x, float z, float *y) {
  float d = (t->v1.z - t->v2.z) * (t->v0.x - t->v2.x) +
            (t->v2.x - t->v1.x) * (t->v0.z - t->v2.z);
  float a, b, c;
  if (fabsf(d) < 1e-9f)
    return false;
  a = ((t->v1.z - t->v2.z) * (x - t->v2.x) +
       (t->v2.x - t->v1.x) * (z - t->v2.z)) / d;
  b = ((t->v2.z - t->v0.z) * (x - t->v2.x) +
       (t->v0.x - t->v2.x) * (z - t->v2.z)) / d;
  c = 1.0f - a - b;
  if (a < -1e-4f || b < -1e-4f || c < -1e-4f)
    return false;
  *y = a * t->v0.y + b * t->v1.y + c * t->v2.y;
  return true;
}

float world_ground(World *w, V3 feet, float radius, float up, float down) {
  const float r = radius * 0.5f;
  const float sx[5] = {0.0f, r, -r, 0.0f, 0.0f}, sz[5] = {0.0f, 0.0f, 0.0f, r, -r};
  float top = feet.y + up, bottom = feet.y - down, best = -1e30f;
  int16_t lo[3] = {lo8(feet.x - r), lo8(bottom), lo8(feet.z - r)};
  int16_t hi[3] = {hi8(feet.x + r), hi8(top), hi8(feet.z + r)};
  uint32_t n;

  stamp_next(w);
  n = gather(w, lo, hi, 0);
  for (uint32_t k = 0; k < n; k++) {
    const CTri *t = &w->tri[cand[k]];
    if (!walkable(t))
      continue;
    for (int s = 0; s < 5; s++) {
      float y;
      if (height_at(t, feet.x + sx[s], feet.z + sz[s], &y) && y <= top &&
          y >= bottom && y > best)
        best = y;
    }
  }
  return best;
}

/* Ericson, Real-Time Collision Detection, 5.1.5. */
static V3 closest_on_tri(V3 p, const CTri *t) {
  V3 a = t->v0, b = t->v1, c = t->v2;
  V3 ab = v3_sub(b, a), ac = v3_sub(c, a), ap = v3_sub(p, a), bp, cp;
  float d1 = v3_dot(ab, ap), d2 = v3_dot(ac, ap), d3, d4, d5, d6, va, vb, vc,
        den;

  if (d1 <= 0.0f && d2 <= 0.0f)
    return a;
  bp = v3_sub(p, b);
  d3 = v3_dot(ab, bp);
  d4 = v3_dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3)
    return b;
  vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    return v3_add(a, v3_scale(ab, d1 / (d1 - d3)));
  cp = v3_sub(p, c);
  d5 = v3_dot(ab, cp);
  d6 = v3_dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6)
    return c;
  vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    return v3_add(a, v3_scale(ac, d2 / (d2 - d6)));
  va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    return v3_add(b, v3_scale(v3_sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
  den = 1.0f / (va + vb + vc);
  return v3_add(a, v3_add(v3_scale(ab, vb * den), v3_scale(ac, vc * den)));
}

/* Sphere centres up the capsule, no further apart than the radius. */
static int spheres(float radius, float height, float step, float *y) {
  float y0 = step + radius, y1 = height - radius;
  int n;
  if (y1 < y0)
    y1 = y0;
  n = (int)((y1 - y0) / radius) + 2;
  if (n > 8)
    n = 8;
  for (int i = 0; i < n; i++)
    y[i] = y0 + (y1 - y0) * (float)i / (float)(n - 1);
  return n;
}

static uint32_t body_cands(World *w, V3 feet, float radius, float height,
                           float step) {
  float m = radius + 0.05f;
  int16_t lo[3] = {lo8(feet.x - m), lo8(feet.y + step), lo8(feet.z - m)};
  int16_t hi[3] = {hi8(feet.x + m), hi8(feet.y + height), hi8(feet.z + m)};
  stamp_next(w);
  return gather(w, lo, hi, 0);
}

V3 world_push(World *w, V3 feet, float radius, float height, float step) {
  float ys[8], r2 = radius * radius;
  int ns = spheres(radius, height, step, ys);

  for (int iter = 0; iter < 3; iter++) {
    uint32_t n = body_cands(w, feet, radius, height, step);
    bool moved = false;
    for (uint32_t k = 0; k < n; k++) {
      const CTri *t = &w->tri[cand[k]];
      for (int s = 0; s < ns; s++) {
        V3 c = v3(feet.x, feet.y + ys[s], feet.z), d;
        float dist2, dist, hl, push, plane;
        int16_t ylo = lo8(c.y - radius), yhi = hi8(c.y + radius);
        if (t->hi[1] < ylo || t->lo[1] > yhi)
          continue;
        /* Off the triangle's plane by more than the radius: no contact,
         * found for a tenth of the closest point's cost. */
        plane = v3_dot(v3_sub(c, t->v0), t->n);
        if (plane > radius || plane < -radius)
          continue;
        d = v3_sub(c, closest_on_tri(c, t));
        dist2 = v3_dot(d, d);
        if (dist2 >= r2 || dist2 < 1e-12f)
          continue;
        dist = sqrtf(dist2);
        /* Pushed sideways only: a contact from straight above or below is
         * a floor or a ceiling, not a wall. */
        hl = sqrtf(d.x * d.x + d.z * d.z) / dist;
        if (hl < 0.25f)
          continue;
        push = (radius - dist) / hl;
        if (push > radius)
          push = radius;
        push /= dist * hl;
        feet.x += d.x * push;
        feet.z += d.z * push;
        moved = true;
      }
    }
    if (!moved)
      break;
  }
  return feet;
}

bool world_free(World *w, V3 feet, float radius, float height, float step) {
  float ys[8], r2 = radius * radius;
  int ns = spheres(radius, height, step, ys);
  uint32_t n = body_cands(w, feet, radius, height, step);
  for (uint32_t k = 0; k < n; k++)
    for (int s = 0; s < ns; s++) {
      const CTri *t = &w->tri[cand[k]];
      V3 c = v3(feet.x, feet.y + ys[s], feet.z), d;
      float plane = v3_dot(v3_sub(c, t->v0), t->n);
      if (plane > radius || plane < -radius)
        continue;
      d = v3_sub(c, closest_on_tri(c, t));
      if (v3_dot(d, d) < r2)
        return false;
    }
  return true;
}

/* Moller and Trumbore, either side facing; t along from->to. */
static bool ray_tri(V3 o, V3 dir, const CTri *t, float *at) {
  V3 e1 = v3_sub(t->v1, t->v0), e2 = v3_sub(t->v2, t->v0);
  V3 pv = v3_cross(dir, e2), tv, qv;
  float det = v3_dot(e1, pv), inv, u, v;
  if (fabsf(det) < 1e-12f)
    return false;
  inv = 1.0f / det;
  tv = v3_sub(o, t->v0);
  u = v3_dot(tv, pv) * inv;
  if (u < 0.0f || u > 1.0f)
    return false;
  qv = v3_cross(tv, e1);
  v = v3_dot(dir, qv) * inv;
  if (v < 0.0f || u + v > 1.0f)
    return false;
  *at = v3_dot(e2, qv) * inv;
  return *at >= 0.0f;
}

float world_ray(World *w, V3 from, V3 to) {
  V3 dir = v3_sub(to, from);
  float len = v3_len(dir), best = 1.0f;
  int steps = (int)(len / (w->cell * 0.5f)) + 1;

  stamp_next(w);
  /* In pieces of half a cell, so each gathers only triangles near the
   * line; the stamps keep one test per triangle. */
  for (int s = 0; s < steps; s++) {
    float t0 = (float)s / steps, t1 = (float)(s + 1) / steps;
    V3 a = v3_add(from, v3_scale(dir, t0)), b = v3_add(from, v3_scale(dir, t1));
    int16_t lo[3] = {lo8(fminf(a.x, b.x)), lo8(fminf(a.y, b.y)),
                     lo8(fminf(a.z, b.z))};
    int16_t hi[3] = {hi8(fmaxf(a.x, b.x)), hi8(fmaxf(a.y, b.y)),
                     hi8(fmaxf(a.z, b.z))};
    uint32_t n;
    if (t0 >= best)
      break;
    n = gather(w, lo, hi, 0);
    for (uint32_t k = 0; k < n; k++) {
      float at;
      if (ray_tri(from, dir, &w->tri[cand[k]], &at) && at < best)
        best = at;
    }
  }
  return best;
}
