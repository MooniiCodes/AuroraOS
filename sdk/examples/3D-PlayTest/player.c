/* The player, from a .glb through cgltf: every node as the skeleton (rest
 * pose, parents first), the skinning input in fixed point, and any
 * animation clips. A skinned mesh follows its skin's joints; a plain mesh
 * follows its own node, so models without bones work too. */
#include "loader.h"
#include "vmath.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void node_rest(const cgltf_node *n, Node *o) {
  if (n->has_matrix) {
    Aff a;
    aff_from_gltf(&a, n->matrix);
    aff_decompose(&a, &o->t, &o->r, &o->s);
    return;
  }
  o->t = n->has_translation
             ? v3(n->translation[0], n->translation[1], n->translation[2])
             : v3(0, 0, 0);
  o->r = q_identity();
  if (n->has_rotation) {
    Quat q = {n->rotation[0], n->rotation[1], n->rotation[2], n->rotation[3]};
    o->r = q_norm(q);
  }
  o->s = n->has_scale ? v3(n->scale[0], n->scale[1], n->scale[2])
                      : v3(1, 1, 1);
}

static void rest_globals(const Player *p, Aff *g) {
  for (int k = 0; k < p->nnodes; k++) {
    int i = p->order[k];
    const Node *n = &p->node[i];
    Aff l;
    aff_trs(&l, n->t, n->r, n->s);
    if (n->parent >= 0)
      aff_mul(&g[i], &g[n->parent], &l);
    else
      g[i] = l;
  }
}

static int slot_add(Player *p, int node, const Aff *ibm) {
  if (p->njoints >= MAX_JOINTS)
    return -1;
  p->joint[p->njoints] = node;
  if (ibm)
    p->ibm[p->njoints] = *ibm;
  else
    aff_identity(&p->ibm[p->njoints]);
  return p->njoints++;
}

/* 1 left, 2 right, 0 neither: "Thigh.L", "thigh_r", "LeftUpLeg", "R_Arm". */
static int side_of(const char *name) {
  char s[64];
  size_t n = 0;
  for (; name && name[n] && n + 1 < sizeof(s); n++)
    s[n] = (char)(name[n] >= 'A' && name[n] <= 'Z' ? name[n] - 'A' + 'a'
                                                    : name[n]);
  s[n] = 0;
  if (strstr(s, "left"))
    return 1;
  if (strstr(s, "right"))
    return 2;
  if (n >= 2 && strchr("._- ", s[n - 2])) {
    if (s[n - 1] == 'l')
      return 1;
    if (s[n - 1] == 'r')
      return 2;
  }
  if (n >= 2 && strchr("._- ", s[1])) {
    if (s[0] == 'l')
      return 1;
    if (s[0] == 'r')
      return 2;
  }
  return 0;
}

static bool has(const char *s, const char *word) { return strstr(s, word) != 0; }

/* Which RIG_* a bone is, by the names Blender, Mixamo and most rigs use
 * (the left one of a pair); -1 for the rest. */
static int rig_kind(const char *name) {
  char s[64];
  load_squash(name, s, sizeof(s));
  if (has(s, "end") || has(s, "tip") || has(s, "twist") || has(s, "ik") ||
      has(s, "pole") || has(s, "target") || has(s, "toe") || has(s, "foot") ||
      has(s, "hand") || has(s, "finger"))
    return -1;
  if (has(s, "hips") || has(s, "pelvis"))
    return RIG_HIPS;
  if (has(s, "thigh") || has(s, "upleg") || has(s, "upperleg"))
    return RIG_THIGH_L;
  if (has(s, "shin") || has(s, "calf") || has(s, "lowerleg") ||
      has(s, "knee") || has(s, "leg"))
    return RIG_SHIN_L;
  if (has(s, "forearm") || has(s, "lowerarm") || has(s, "elbow"))
    return RIG_FARM_L;
  if (has(s, "upperarm") || has(s, "uparm") || has(s, "arm"))
    return RIG_UARM_L;
  if (has(s, "spine") || has(s, "chest") || has(s, "torso"))
    return RIG_SPINE;
  if (has(s, "head"))
    return RIG_HEAD;
  return -1;
}

static bool paired(int kind) {
  return kind == RIG_THIGH_L || kind == RIG_SHIN_L || kind == RIG_UARM_L ||
         kind == RIG_FARM_L;
}

/* The first match in parent-first order wins, so "Spine" beats "Spine2". */
static void find_rig(Player *p, const cgltf_data *d, const Aff *g) {
  for (int i = 0; i < RIG_COUNT; i++)
    p->rig[i] = -1;
  p->nrig = 0;
  for (int k = 0; k < p->nnodes; k++) {
    int i = p->order[k], kind = rig_kind(d->nodes[i].name);
    if (kind < 0)
      continue;
    if (paired(kind)) {
      int side = side_of(d->nodes[i].name);
      if (!side) /* the model faces +Z, so its left is +X */
        side = g[i].m[0][3] >= 0.0f ? 1 : 2;
      if (side == 2)
        kind++;
    }
    if (p->rig[kind] < 0) {
      p->rig[kind] = i;
      p->nrig++;
    }
  }
}

/* 2 walk, 1 another way of moving, -1 standing, 0 unknown. */
static int clip_kind(const char *name) {
  char s[40];
  load_squash(name, s, sizeof(s));
  if (has(s, "walk"))
    return 2;
  if (has(s, "run") || has(s, "jog") || has(s, "move") || has(s, "locomot"))
    return 1;
  if (has(s, "idle") || has(s, "stand") || has(s, "breath") || has(s, "rest"))
    return -1;
  return 0;
}

static bool load_chan(Chan *o, const cgltf_animation_channel *ch,
                      const cgltf_data *d) {
  const cgltf_animation_sampler *sm = ch->sampler;
  size_t comps, per, need;

  if (!ch->target_node || !sm || !sm->input || !sm->output ||
      !sm->input->count)
    return false;
  if (ch->target_path == cgltf_animation_path_type_translation)
    o->path = PATH_T;
  else if (ch->target_path == cgltf_animation_path_type_rotation)
    o->path = PATH_R;
  else if (ch->target_path == cgltf_animation_path_type_scale)
    o->path = PATH_S;
  else
    return false;
  comps = o->path == PATH_R ? 4 : 3;
  o->interp = sm->interpolation == cgltf_interpolation_type_step ? INTERP_STEP
              : sm->interpolation == cgltf_interpolation_type_cubic_spline
                  ? INTERP_CUBIC
                  : INTERP_LINEAR;
  per = o->interp == INTERP_CUBIC ? 3 : 1;
  o->nkeys = (uint32_t)sm->input->count;
  need = (size_t)o->nkeys * comps * per;
  if (cgltf_num_components(sm->output->type) != comps ||
      sm->output->count * comps != need)
    return false;
  o->node = (int)(ch->target_node - d->nodes);
  o->time = malloc(o->nkeys * sizeof(float));
  o->val = malloc(need * sizeof(float));
  if (!o->time || !o->val) {
    free(o->time);
    free(o->val);
    return false;
  }
  cgltf_accessor_unpack_floats(sm->input, o->time, o->nkeys);
  cgltf_accessor_unpack_floats(sm->output, o->val, need);
  return true;
}

static void load_clips(Player *p, const cgltf_data *d) {
  int best = 0;
  p->clip_walk = p->clip_idle = -1;
  p->nclip = 0;
  if (!d->animations_count)
    return;
  p->clip = calloc(d->animations_count, sizeof(Clip));
  if (!p->clip)
    return;
  for (cgltf_size a = 0; a < d->animations_count; a++) {
    const cgltf_animation *an = &d->animations[a];
    Clip *c = &p->clip[p->nclip];
    int kind;
    c->chan = calloc(an->channels_count ? an->channels_count : 1, sizeof(Chan));
    if (!c->chan)
      continue;
    snprintf(c->name, sizeof(c->name), "%s", an->name ? an->name : "clip");
    for (cgltf_size k = 0; k < an->channels_count; k++)
      if (load_chan(&c->chan[c->nchan], &an->channels[k], d)) {
        Chan *o = &c->chan[c->nchan++];
        if (o->time[o->nkeys - 1] > c->length)
          c->length = o->time[o->nkeys - 1];
      }
    if (!c->nchan || !(c->length > 0.0f)) {
      free(c->chan);
      memset(c, 0, sizeof(*c));
      continue;
    }
    kind = clip_kind(c->name);
    if (kind < 0) {
      if (p->clip_idle < 0)
        p->clip_idle = p->nclip;
    } else if (p->clip_walk < 0 || kind > best) {
      p->clip_walk = p->nclip;
      best = kind;
    }
    p->nclip++;
  }
}

static bool prim_ok(const cgltf_primitive *pr) {
  const cgltf_accessor *pos = load_attr(pr, cgltf_attribute_type_position, 0);
  return load_is_triangles(pr) && pos && pos->count;
}

/* Up to four influences, weights rounded to add up to 255. */
static void weights(uint8_t *jw, const cgltf_accessor *ja,
                    const cgltf_accessor *wa, cgltf_size i,
                    const cgltf_skin *skin, int slot0) {
  cgltf_uint j[4] = {0, 0, 0, 0};
  float w[4] = {0, 0, 0, 0}, sum = 0.0f;
  int q[4], qs = 0, big = 0;

  memset(jw, 0, 8);
  jw[0] = (uint8_t)slot0;
  jw[4] = 255;
  if (!ja || !wa)
    return;
  cgltf_accessor_read_uint(ja, i, j, 4);
  cgltf_accessor_read_float(wa, i, w, 4);
  for (int k = 0; k < 4; k++) {
    if (j[k] >= skin->joints_count || !(w[k] > 0.0f))
      w[k] = 0.0f;
    sum += w[k];
  }
  if (!(sum > 1e-6f))
    return;
  for (int k = 0; k < 4; k++) {
    q[k] = (int)(w[k] / sum * 255.0f + 0.5f);
    qs += q[k];
    if (q[k] > q[big])
      big = k;
  }
  q[big] += 255 - qs;
  for (int k = 0; k < 4; k++) {
    jw[k] = (uint8_t)(slot0 + (q[k] ? (int)j[k] : 0));
    jw[4 + k] = (uint8_t)q[k];
  }
}

/* One primitive in the mesh's own space. `slot0` is the skin's first joint
 * slot, or the plain mesh's node slot when `skin` is NULL. */
static void player_prim(Player *p, const cgltf_primitive *pr,
                        const cgltf_skin *skin, int slot0) {
  const cgltf_accessor *pos = load_attr(pr, cgltf_attribute_type_position, 0);
  const cgltf_accessor *nrm = load_attr(pr, cgltf_attribute_type_normal, 0);
  const cgltf_accessor *col = load_attr(pr, cgltf_attribute_type_color, 0);
  const cgltf_accessor *ja =
      skin ? load_attr(pr, cgltf_attribute_type_joints, 0) : 0;
  const cgltf_accessor *wa =
      skin ? load_attr(pr, cgltf_attribute_type_weights, 0) : 0;
  uint32_t nv = (uint32_t)pos->count, nt, *tri, v0 = p->mesh.nvtx;
  V3 *fn = NULL, ctr = v3(0, 0, 0);
  MatInfo mi;
  Part *part;

  tri = load_tris(pr, nv, &nt);
  if (!tri)
    return;
  load_mat(pr->material, &mi);
  if (!nrm && (fn = calloc(nv, sizeof(V3))) != NULL)
    for (uint32_t t = 0; t < nt; t++) {
      V3 a = load_read3(pos, tri[t * 3], v3(0, 0, 0));
      V3 b = load_read3(pos, tri[t * 3 + 1], v3(0, 0, 0));
      V3 c = load_read3(pos, tri[t * 3 + 2], v3(0, 0, 0));
      V3 f = v3_cross(v3_sub(b, a), v3_sub(c, a));
      for (int k = 0; k < 3; k++)
        fn[tri[t * 3 + k]] = v3_add(fn[tri[t * 3 + k]], f);
    }

  part = &p->mesh.part[p->mesh.npart++];
  part->vfirst = v0;
  part->vcount = nv;
  part->ifirst = p->mesh.nidx;
  part->icount = nt * 3u;
  part->blend = mi.blend;

  for (uint32_t i = 0; i < nv; i++) {
    uint32_t v = v0 + i;
    V3 a = load_read3(pos, i, v3(0, 0, 0));
    V3 n = v3_norm(nrm  ? load_read3(nrm, i, v3(0, 1, 0))
                   : fn ? fn[i]
                        : v3(0, 1, 0));
    float vc[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    Vtx *o = &p->mesh.vtx[v];

    p->pos[v * 3] = (int32_t)lrintf(a.x * 4096.0f);
    p->pos[v * 3 + 1] = (int32_t)lrintf(a.y * 4096.0f);
    p->pos[v * 3 + 2] = (int32_t)lrintf(a.z * 4096.0f);
    p->nrm[v * 3] = (int16_t)lrintf(n.x * 16383.0f);
    p->nrm[v * 3 + 1] = (int16_t)lrintf(n.y * 16383.0f);
    p->nrm[v * 3 + 2] = (int16_t)lrintf(n.z * 16383.0f);
    ctr = v3_add(ctr, a);
    if (col)
      cgltf_accessor_read_float(col, i, vc, cgltf_num_components(col->type));
    for (int k = 0; k < 3; k++)
      p->rgba[v * 4 + k] = (uint16_t)clampf(
          (mi.rgba[k] * vc[k] + mi.emissive[k]) * 4095.0f + 0.5f, 0.0f,
          4095.0f);
    p->rgba[v * 4 + 3] =
        (uint16_t)clampf(mi.rgba[3] * vc[3] * 4095.0f + 0.5f, 0.0f, 4095.0f);
    weights(&p->jw[v * 8u], ja, wa, i, skin, slot0);
    o->pos[0] = o->pos[1] = o->pos[2] = 0.0f;
    o->pos[3] = 1.0f;
    o->uv[0] = o->uv[1] = 0.0f;
    o->col[0] = o->col[1] = o->col[2] = o->col[3] = 1.0f;
  }
  part->centre = v3_scale(ctr, 1.0f / (float)nv);
  for (uint32_t t = 0; t < nt * 3u; t++)
    p->mesh.idx[p->mesh.nidx + t] = (uint16_t)tri[t];
  p->mesh.nvtx += nv;
  p->mesh.nidx += nt * 3u;
  p->ntris += nt;
  free(tri);
  free(fn);
}

/* Feet on the origin and centred on X and Z, from the rest pose; scaled to
 * 1.8 m only when the file's units clearly are not metres. */
static void player_fix(Player *p, const Aff *g) {
  static Aff j[MAX_JOINTS];
  V3 lo = v3(1e30f, 1e30f, 1e30f), hi = v3(-1e30f, -1e30f, -1e30f);
  float h, s, w, dpt;

  for (int k = 0; k < p->njoints; k++)
    aff_mul(&j[k], &g[p->joint[k]], &p->ibm[k]);
  for (uint32_t v = 0; v < p->mesh.nvtx; v++) {
    V3 a = v3(p->pos[v * 3] / 4096.0f, p->pos[v * 3 + 1] / 4096.0f,
              p->pos[v * 3 + 2] / 4096.0f);
    V3 o = v3(0, 0, 0);
    for (int k = 0; k < 4; k++) {
      int wt = p->jw[v * 8 + 4 + k];
      if (wt)
        o = v3_add(o, v3_scale(aff_point(&j[p->jw[v * 8 + k]], a),
                               wt / 255.0f));
    }
    lo = v3(fminf(lo.x, o.x), fminf(lo.y, o.y), fminf(lo.z, o.z));
    hi = v3(fmaxf(hi.x, o.x), fmaxf(hi.y, o.y), fmaxf(hi.z, o.z));
  }
  h = hi.y - lo.y;
  s = (h < 0.3f || h > 5.0f) && h > 1e-6f ? 1.8f / h : 1.0f;
  aff_identity(&p->fix);
  for (int r = 0; r < 3; r++)
    p->fix.m[r][r] = s;
  p->fix.m[0][3] = -s * 0.5f * (lo.x + hi.x);
  p->fix.m[1][3] = -s * lo.y;
  p->fix.m[2][3] = -s * 0.5f * (lo.z + hi.z);
  p->height = h * s;
  w = (hi.x - lo.x) * s;
  dpt = (hi.z - lo.z) * s;
  /* Arms held out make the width no guide, so the height caps it. */
  p->radius = clampf(fminf(0.375f * fmaxf(w, dpt), 0.16f * p->height), 0.15f,
                     0.45f);
}

/* For procedural motion: each node's model axes in its parent's frame, and
 * the parent frame's size in metres. */
static void node_axes(Player *p, const Aff *g) {
  for (int i = 0; i < p->nnodes; i++) {
    Node *n = &p->node[i];
    Quat q = q_identity();
    float us = 1.0f;
    if (n->parent >= 0) {
      V3 t, s;
      aff_decompose(&g[n->parent], &t, &q, &s);
      us = cbrtf(fabsf(s.x * s.y * s.z));
    }
    n->unit = p->fix.m[0][0] * us;
    n->ax[0] = q_rotate(q_conj(q), v3(1, 0, 0));
    n->ax[1] = q_rotate(q_conj(q), v3(0, 1, 0));
    n->ax[2] = q_rotate(q_conj(q), v3(0, 0, 1));
  }
}

/* Gives every mesh node its joint slots and reads its primitives. */
static bool read_meshes(Player *p, const cgltf_data *d, uint32_t nvtx,
                        char *err, size_t errn) {
  static int rigid[MAX_NODES];
  int skin_slot[16];

  for (int i = 0; i < MAX_NODES; i++)
    rigid[i] = -1;
  for (int i = 0; i < 16; i++)
    skin_slot[i] = -1;
  if (d->skins_count > 16)
    return load_fail(err, errn, "it has more than 16 skins");

  for (cgltf_size i = 0; i < d->nodes_count; i++) {
    const cgltf_node *n = &d->nodes[i];
    const cgltf_skin *sk = n->skin;
    int slot0;
    if (!n->mesh || !load_in_scene(d, n))
      continue;
    if (sk) {
      size_t si = (size_t)(sk - d->skins);
      if (skin_slot[si] < 0) {
        skin_slot[si] = p->njoints;
        for (cgltf_size j = 0; j < sk->joints_count; j++) {
          Aff ibm;
          float m[16];
          aff_identity(&ibm);
          if (sk->inverse_bind_matrices &&
              cgltf_accessor_read_float(sk->inverse_bind_matrices, j, m, 16))
            aff_from_gltf(&ibm, m);
          if (slot_add(p, (int)(sk->joints[j] - d->nodes), &ibm) < 0)
            return load_fail(err, errn, "it has more than %d joints",
                             MAX_JOINTS);
        }
      }
      slot0 = skin_slot[si];
    } else {
      /* glTF ignores a skinned mesh's own node; a plain mesh follows it. */
      if (rigid[i] < 0)
        rigid[i] = slot_add(p, (int)i, NULL);
      slot0 = rigid[i];
      if (slot0 < 0)
        return load_fail(err, errn, "it has more than %d joints", MAX_JOINTS);
    }
    if (p->mesh_node < 0 || (sk && !d->nodes[p->mesh_node].skin))
      p->mesh_node = (int)i;
    for (cgltf_size k = 0; k < n->mesh->primitives_count; k++) {
      const cgltf_primitive *pr = &n->mesh->primitives[k];
      if (prim_ok(pr) &&
          p->mesh.nvtx +
                  load_attr(pr, cgltf_attribute_type_position, 0)->count <=
              nvtx)
        player_prim(p, pr, sk, slot0);
    }
  }
  return true;
}

bool player_load(Player *p, const void *glb, size_t size, char *err,
                 size_t errn) {
  static Aff g[MAX_NODES];
  static int depth[MAX_NODES];
  cgltf_data *d;
  uint32_t nvtx = 0, nidx = 0, npart = 0;

  memset(p, 0, sizeof(*p));
  p->mesh_node = -1;
  if (!load_parse(glb, size, &d, err, errn))
    return false;
  if (d->nodes_count > MAX_NODES) {
    cgltf_free(d);
    return load_fail(err, errn, "it has more than %d nodes", MAX_NODES);
  }

  p->nnodes = (int)d->nodes_count;
  for (int i = 0; i < p->nnodes; i++) {
    const cgltf_node *n = &d->nodes[i], *a = n;
    p->node[i].parent = n->parent ? (int)(n->parent - d->nodes) : -1;
    node_rest(n, &p->node[i]);
    for (depth[i] = 0; a->parent; a = a->parent)
      depth[i]++;
  }
  /* Parents before children: by depth, which is enough. */
  for (int k = 0, dd = 0; k < p->nnodes; dd++)
    for (int i = 0; i < p->nnodes; i++)
      if (depth[i] == dd)
        p->order[k++] = i;
  rest_globals(p, g);

  /* One mesh draws from 16-bit indices, so the player stops at 65535
   * vertices. */
  for (cgltf_size i = 0; i < d->nodes_count; i++) {
    const cgltf_node *n = &d->nodes[i];
    if (!n->mesh || !load_in_scene(d, n))
      continue;
    for (cgltf_size k = 0; k < n->mesh->primitives_count; k++) {
      const cgltf_primitive *pr = &n->mesh->primitives[k];
      uint32_t c;
      if (!prim_ok(pr))
        continue;
      c = (uint32_t)load_attr(pr, cgltf_attribute_type_position, 0)->count;
      if (nvtx + c > 65535u)
        continue;
      nvtx += c;
      nidx += load_index_bound(pr, c);
      npart++;
    }
  }
  if (!npart) {
    cgltf_free(d);
    return load_fail(err, errn, "it has no triangles");
  }
  p->pos = malloc(nvtx * 3u * sizeof(int32_t));
  p->nrm = malloc(nvtx * 3u * sizeof(int16_t));
  p->jw = malloc(nvtx * 8u);
  p->rgba = malloc(nvtx * 4u * sizeof(uint16_t));
  if (!p->pos || !p->nrm || !p->jw || !p->rgba ||
      !load_mesh_alloc(&p->mesh, nvtx, nidx, npart)) {
    cgltf_free(d);
    return load_fail(err, errn, "out of memory");
  }
  if (!read_meshes(p, d, nvtx, err, errn)) {
    cgltf_free(d);
    return false;
  }
  if (!p->mesh.nidx) {
    cgltf_free(d);
    return load_fail(err, errn, "it has no triangles");
  }

  player_fix(p, g);
  node_axes(p, g);
  find_rig(p, d, g);
  load_clips(p, d);
  cgltf_free(d);
  return true;
}
