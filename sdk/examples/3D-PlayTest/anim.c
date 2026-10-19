/* The player's pose each frame and the skinning that follows it.
 *
 * A pose comes from the file's clips when it has a walk (blended with its
 * idle by speed), otherwise from a procedural walk cycle over the bones
 * player.c found by name. Rotations turn a bone about the model's own axes
 * (Node.ax), so the same angles suit any rig's bone orientations.
 *
 * Skinning is fixed point: positions in 1/4096 m, matrices in 1/2^20 for
 * positions and 1/16384 for normals, 64-bit products. In soft float the
 * same work would take several milliseconds a frame on the ARM9. */
#include "vmath.h"

#include <string.h>

typedef struct {
  V3 t[MAX_NODES];
  Quat r[MAX_NODES];
  V3 s[MAX_NODES];
} Pose;

static void pose_rest(const Player *p, Pose *o) {
  for (int i = 0; i < p->nnodes; i++) {
    o->t[i] = p->node[i].t;
    o->r[i] = p->node[i].r;
    o->s[i] = p->node[i].s;
  }
}

/* The key at or before `t` and how far towards the next one. */
static uint32_t key_at(const Chan *c, float t, float *frac) {
  uint32_t lo = 0, hi = c->nkeys - 1u;
  *frac = 0.0f;
  if (c->nkeys < 2u || t <= c->time[0])
    return 0;
  if (t >= c->time[hi])
    return hi;
  while (hi - lo > 1u) {
    uint32_t mid = (lo + hi) / 2u;
    if (c->time[mid] <= t)
      lo = mid;
    else
      hi = mid;
  }
  {
    float span = c->time[lo + 1u] - c->time[lo];
    *frac = span > 0.0f ? (t - c->time[lo]) / span : 0.0f;
  }
  return lo;
}

static void sample(const Chan *c, float t, float out[4]) {
  const uint32_t comps = c->path == PATH_R ? 4u : 3u;
  float f;
  uint32_t k = key_at(c, t, &f);

  if (c->interp == INTERP_CUBIC) {
    /* Keys hold in-tangent, value, out-tangent (glTF Appendix C). */
    const float *v0 = c->val + (k * 3u + 1u) * comps;
    if (f <= 0.0f || k + 1u >= c->nkeys) {
      memcpy(out, v0, comps * sizeof(float));
    } else {
      const float *b0 = c->val + (k * 3u + 2u) * comps;
      const float *a1 = c->val + ((k + 1u) * 3u) * comps;
      const float *v1 = c->val + ((k + 1u) * 3u + 1u) * comps;
      float dt = c->time[k + 1u] - c->time[k], f2 = f * f, f3 = f2 * f;
      float h00 = 2 * f3 - 3 * f2 + 1, h10 = (f3 - 2 * f2 + f) * dt;
      float h01 = -2 * f3 + 3 * f2, h11 = (f3 - f2) * dt;
      for (uint32_t i = 0; i < comps; i++)
        out[i] = h00 * v0[i] + h10 * b0[i] + h01 * v1[i] + h11 * a1[i];
    }
  } else {
    const float *v0 = c->val + k * comps;
    if (c->interp == INTERP_STEP || f <= 0.0f || k + 1u >= c->nkeys) {
      memcpy(out, v0, comps * sizeof(float));
    } else {
      const float *v1 = v0 + comps;
      if (c->path == PATH_R) {
        Quat a = {v0[0], v0[1], v0[2], v0[3]}, b = {v1[0], v1[1], v1[2], v1[3]};
        Quat q = q_nlerp(a, b, f);
        out[0] = q.x, out[1] = q.y, out[2] = q.z, out[3] = q.w;
        return;
      }
      for (uint32_t i = 0; i < comps; i++)
        out[i] = v0[i] + (v1[i] - v0[i]) * f;
    }
  }
  if (c->path == PATH_R) {
    Quat q = {out[0], out[1], out[2], out[3]};
    q = q_norm(q);
    out[0] = q.x, out[1] = q.y, out[2] = q.z, out[3] = q.w;
  }
}

static void pose_clip(const Player *p, const Clip *c, float t, Pose *o) {
  pose_rest(p, o);
  for (uint32_t i = 0; i < c->nchan; i++) {
    const Chan *ch = &c->chan[i];
    float v[4];
    if (ch->node < 0 || ch->node >= p->nnodes)
      continue;
    sample(ch, t, v);
    if (ch->path == PATH_T) {
      o->t[ch->node] = v3(v[0], v[1], v[2]);
    } else if (ch->path == PATH_S) {
      o->s[ch->node] = v3(v[0], v[1], v[2]);
    } else {
      Quat q = {v[0], v[1], v[2], v[3]};
      o->r[ch->node] = q;
    }
  }
}

static void pose_blend(Pose *a, const Pose *b, float w, int n) {
  for (int i = 0; i < n; i++) {
    a->t[i] = v3_lerp(a->t[i], b->t[i], w);
    a->s[i] = v3_lerp(a->s[i], b->s[i], w);
    a->r[i] = q_nlerp(a->r[i], b->r[i], w);
  }
}

static float wrap01(float x) { return x - floorf(x); }

/* Turns rig bone `rig` about the model's axis `axis` (0 X, 1 Y, 2 Z). */
static void turn(Pose *o, const Player *p, int rig, int axis, float angle) {
  int i = p->rig[rig];
  if (i < 0 || angle == 0.0f)
    return;
  o->r[i] = q_mul(q_axis(p->node[i].ax[axis], angle), o->r[i]);
}

static void lift(Pose *o, const Player *p, int rig, float metres) {
  int i = p->rig[rig];
  if (i < 0)
    return;
  o->t[i] = v3_add(o->t[i], v3_scale(p->node[i].ax[1], metres / p->node[i].unit));
}

/* Signs, for a model facing +Z with up +Y and its left on +X: a positive
 * turn about X swings a hanging limb backwards, so a forward step is
 * negative and a knee bends positive; about Y it carries the left side
 * backwards; about Z it lifts the left arm outwards. */
static void walk_cycle(const Player *p, const AnimState *a, Pose *o) {
  float s = sinf(a->phase), c = cosf(a->phase), w = a->walk;
  float sp = clampf(a->speed, 0.0f, 6.0f);
  float leg = (0.32f + 0.11f * sp) * w, arm = (0.20f + 0.09f * sp) * w;
  float knee = (0.50f + 0.22f * sp) * w, bob = (0.012f + 0.009f * sp) * w;
  float lean = 0.035f * sp * w, breathe = sinf(a->t * 2.2f) * (1.0f - w);
  float out = 0.07f + 0.02f * breathe;

  turn(o, p, RIG_THIGH_L, 0, -leg * s);
  turn(o, p, RIG_THIGH_R, 0, leg * s);
  /* A knee folds while its foot swings forwards, and is nearly straight
   * where the stride is longest. */
  turn(o, p, RIG_SHIN_L, 0, 0.08f * w + knee * fmaxf(0.0f, c));
  turn(o, p, RIG_SHIN_R, 0, 0.08f * w + knee * fmaxf(0.0f, -c));

  turn(o, p, RIG_UARM_L, 0, arm * s);
  turn(o, p, RIG_UARM_R, 0, -arm * s);
  turn(o, p, RIG_UARM_L, 2, out);
  turn(o, p, RIG_UARM_R, 2, -out);
  turn(o, p, RIG_FARM_L, 0, -(0.15f * w + 0.6f * arm * fmaxf(0.0f, -s)));
  turn(o, p, RIG_FARM_R, 0, -(0.15f * w + 0.6f * arm * fmaxf(0.0f, s)));

  /* Highest as the legs pass, lowest at the longest stride: twice a cycle. */
  lift(o, p, RIG_HIPS, bob * cosf(2.0f * a->phase));
  turn(o, p, RIG_HIPS, 1, -0.10f * w * s);
  turn(o, p, RIG_SPINE, 1, 0.16f * w * s);
  turn(o, p, RIG_SPINE, 0, lean + 0.015f * breathe);
  turn(o, p, RIG_HEAD, 1, -0.07f * w * s);
  turn(o, p, RIG_HEAD, 0, -0.7f * lean);
}

static void air_pose(const Player *p, float air, Pose *o) {
  if (air <= 0.0f)
    return;
  turn(o, p, RIG_THIGH_L, 0, -0.55f * air);
  turn(o, p, RIG_THIGH_R, 0, -0.15f * air);
  turn(o, p, RIG_SHIN_L, 0, 0.85f * air);
  turn(o, p, RIG_SHIN_R, 0, 0.45f * air);
  turn(o, p, RIG_UARM_L, 2, 0.55f * air);
  turn(o, p, RIG_UARM_R, 2, -0.55f * air);
  turn(o, p, RIG_UARM_L, 0, -0.25f * air);
  turn(o, p, RIG_UARM_R, 0, -0.25f * air);
  turn(o, p, RIG_FARM_L, 0, -0.35f * air);
  turn(o, p, RIG_FARM_R, 0, -0.35f * air);
}

void anim_pose(const Player *p, const AnimState *a, Aff *jm) {
  static Pose pose, walk;
  static Aff g[MAX_NODES];

  if (p->clip_walk >= 0) {
    const Clip *cw = &p->clip[p->clip_walk];
    if (p->clip_idle >= 0) {
      const Clip *ci = &p->clip[p->clip_idle];
      pose_clip(p, ci, wrap01(a->t / ci->length) * ci->length, &pose);
    } else {
      pose_rest(p, &pose);
    }
    if (a->walk > 0.0f) {
      /* The cycle follows the distance walked, as the procedural one does. */
      pose_clip(p, cw, wrap01(a->phase / (2.0f * PI_F)) * cw->length, &walk);
      pose_blend(&pose, &walk, a->walk, p->nnodes);
    }
  } else {
    pose_rest(p, &pose);
    if (p->nrig)
      walk_cycle(p, a, &pose);
  }
  air_pose(p, a->air, &pose);

  for (int k = 0; k < p->nnodes; k++) {
    int i = p->order[k], up = p->node[i].parent;
    Aff l;
    aff_trs(&l, pose.t[i], pose.r[i], pose.s[i]);
    if (up >= 0)
      aff_mul(&g[i], &g[up], &l);
    else
      g[i] = l;
  }
  for (int s = 0; s < p->njoints; s++) {
    aff_mul(&jm[s], &g[p->joint[s]], &p->ibm[s]);
    aff_mul(&jm[s], &p->fix, &jm[s]);
  }
}

const char *anim_describe(const Player *p) {
  if (p->clip_walk >= 0)
    return p->clip[p->clip_walk].name;
  if (p->nrig >= 4)
    return "procedural walk";
  if (p->nrig)
    return "procedural, few bones";
  return "none (no bones found)";
}

static int32_t q(float v, float one) {
  float f = v * one;
  f = clampf(f, -2147483520.0f, 2147483520.0f);
  return (int32_t)(f < 0.0f ? f - 0.5f : f + 0.5f);
}

void skin_apply(Player *p, const Aff *jm, V3 key, V3 fill, float alpha) {
  static int32_t mp[MAX_JOINTS][12], mn[MAX_JOINTS][9];
  static float tab[4097];
  static bool tab_ready;
  const float a12 = clampf(alpha, 0.0f, 1.0f) / 4095.0f;
  const int32_t amb = (int32_t)(LIGHT_AMBIENT * 16384.0f);
  const int32_t kk = (int32_t)(LIGHT_KEY * 16384.0f);
  const int32_t kf = (int32_t)(LIGHT_FILL * 16384.0f);
  int32_t lk[3] = {q(key.x, 16384.0f), q(key.y, 16384.0f), q(key.z, 16384.0f)};
  int32_t lf[3] = {q(fill.x, 16384.0f), q(fill.y, 16384.0f),
                   q(fill.z, 16384.0f)};

  if (!tab_ready) {
    for (int i = 0; i <= 4096; i++)
      tab[i] = srgb(i / 4096.0f);
    tab_ready = true;
  }

  for (int s = 0; s < p->njoints; s++) {
    float nm[3][3], len = 0.0f;
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++)
        mp[s][r * 4 + c] = q(jm[s].m[r][c], 1048576.0f);
      mp[s][r * 4 + 3] = q(jm[s].m[r][3], 4096.0f);
    }
    /* Normals use the inverse transpose, brought back to unit size. */
    aff_normal_matrix(nm, &jm[s]);
    for (int c = 0; c < 3; c++)
      len += sqrtf(nm[0][c] * nm[0][c] + nm[1][c] * nm[1][c] +
                   nm[2][c] * nm[2][c]);
    len = len > 1e-12f ? 3.0f / len : 0.0f;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        mn[s][r * 3 + c] = q(nm[r][c] * len, 16384.0f);
  }

  for (uint32_t v = 0; v < p->mesh.nvtx; v++) {
    const int32_t *ps = &p->pos[v * 3];
    const int16_t *ns = &p->nrm[v * 3];
    const uint8_t *jw = &p->jw[v * 8];
    const uint16_t *rgb = &p->rgba[v * 4];
    int32_t x = 0, y = 0, z = 0, nx = 0, ny = 0, nz = 0, dk, df, l;
    Vtx *o = &p->mesh.vtx[v];

    for (int k = 0; k < 4; k++) {
      int32_t w = jw[4 + k];
      const int32_t *m, *n;
      int32_t tx, ty, tz, tnx, tny, tnz;
      if (!w)
        continue;
      m = mp[jw[k]];
      n = mn[jw[k]];
      tx = (int32_t)(((int64_t)m[0] * ps[0] + (int64_t)m[1] * ps[1] +
                      (int64_t)m[2] * ps[2]) >> 20) + m[3];
      ty = (int32_t)(((int64_t)m[4] * ps[0] + (int64_t)m[5] * ps[1] +
                      (int64_t)m[6] * ps[2]) >> 20) + m[7];
      tz = (int32_t)(((int64_t)m[8] * ps[0] + (int64_t)m[9] * ps[1] +
                      (int64_t)m[10] * ps[2]) >> 20) + m[11];
      tnx = (n[0] * ns[0] + n[1] * ns[1] + n[2] * ns[2]) >> 14;
      tny = (n[3] * ns[0] + n[4] * ns[1] + n[5] * ns[2]) >> 14;
      tnz = (n[6] * ns[0] + n[7] * ns[1] + n[8] * ns[2]) >> 14;
      if (w == 255) {
        x = tx, y = ty, z = tz, nx = tnx, ny = tny, nz = tnz;
        break;
      }
      /* w * 257 / 65536 is w / 255 to within 1 part in 65000. */
      w *= 257;
      x += (int32_t)(((int64_t)tx * w) >> 16);
      y += (int32_t)(((int64_t)ty * w) >> 16);
      z += (int32_t)(((int64_t)tz * w) >> 16);
      nx += (int32_t)(((int64_t)tnx * w) >> 16);
      ny += (int32_t)(((int64_t)tny * w) >> 16);
      nz += (int32_t)(((int64_t)tnz * w) >> 16);
    }

    dk = (nx * lk[0] + ny * lk[1] + nz * lk[2]) >> 14;
    df = (nx * lf[0] + ny * lf[1] + nz * lf[2]) >> 14;
    l = amb + (dk > 0 ? (kk * dk) >> 14 : 0) + (df > 0 ? (kf * df) >> 14 : 0);
    if (l > 16384)
      l = 16384;

    o->pos[0] = fix2f(x, 12);
    o->pos[1] = fix2f(y, 12);
    o->pos[2] = fix2f(z, 12);
    o->col[0] = tab[(rgb[0] * l) >> 14];
    o->col[1] = tab[(rgb[1] * l) >> 14];
    o->col[2] = tab[(rgb[2] * l) >> 14];
    o->col[3] = (float)rgb[3] * a12;
  }
}
