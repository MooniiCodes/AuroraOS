/* The room, from a .glb through cgltf: baked into what the GPU draws (world
 * space, lit vertex colours) and into the triangles the player walks on and
 * bumps into. Also the cgltf helpers player.c shares (loader.h). */
#include "loader.h"
#include "vmath.h"

#include <malloc.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Towards the lights: a key from above, the front and +X, a fill from
 * behind. */
const V3 g_light_key = {0.40f, 0.75f, 0.53f};
const V3 g_light_fill = {-0.50f, 0.25f, -0.83f};

static float srgb_tab[257];
static bool srgb_ready;

float srgb(float l) {
  float f;
  int i;
  if (!srgb_ready) {
    for (i = 0; i <= 256; i++) {
      float x = i / 256.0f;
      srgb_tab[i] = x <= 0.0031308f ? 12.92f * x
                                    : 1.055f * powf(x, 1.0f / 2.4f) - 0.055f;
    }
    srgb_ready = true;
  }
  if (l <= 0.0f)
    return 0.0f;
  if (l >= 1.0f)
    return 1.0f;
  f = l * 256.0f;
  i = (int)f;
  f -= (float)i;
  return srgb_tab[i] + (srgb_tab[i + 1] - srgb_tab[i]) * f;
}

bool load_fail(char *err, size_t n, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(err, n, fmt, ap);
  va_end(ap);
  return false;
}

static const char *result_text(cgltf_result r) {
  switch (r) {
    case cgltf_result_data_too_short: return "the file is cut short";
    case cgltf_result_unknown_format: return "not a glTF or .glb file";
    case cgltf_result_invalid_json:   return "its JSON is broken";
    case cgltf_result_invalid_gltf:   return "it breaks the glTF rules";
    case cgltf_result_file_not_found: return "a buffer file is missing";
    case cgltf_result_io_error:       return "a buffer could not be read";
    case cgltf_result_out_of_memory:  return "out of memory";
    case cgltf_result_legacy_gltf:    return "glTF 1.0 is not supported";
    default:                          return "it could not be read";
  }
}

bool load_parse(const void *glb, size_t size, cgltf_data **out, char *err,
                size_t errn) {
  cgltf_options opt;
  cgltf_result r;

  memset(&opt, 0, sizeof(opt));
  *out = NULL;
  r = cgltf_parse(&opt, glb, size, out);
  /* A .glb's own buffer needs no path. */
  if (r == cgltf_result_success)
    r = cgltf_load_buffers(&opt, *out, "");
  if (r == cgltf_result_success)
    r = cgltf_validate(*out);
  if (r != cgltf_result_success) {
    if (*out)
      cgltf_free(*out);
    *out = NULL;
    return load_fail(err, errn, "%s", result_text(r));
  }
  return true;
}

const cgltf_accessor *load_attr(const cgltf_primitive *p,
                                cgltf_attribute_type t, int index) {
  for (cgltf_size i = 0; i < p->attributes_count; i++)
    if (p->attributes[i].type == t && p->attributes[i].index == index)
      return p->attributes[i].data;
  return NULL;
}

bool load_is_triangles(const cgltf_primitive *p) {
  return p->type == cgltf_primitive_type_triangles ||
         p->type == cgltf_primitive_type_triangle_strip ||
         p->type == cgltf_primitive_type_triangle_fan;
}

uint32_t load_index_bound(const cgltf_primitive *p, uint32_t nverts) {
  uint32_t n = p->indices ? (uint32_t)p->indices->count : nverts;
  return p->type == cgltf_primitive_type_triangles ? n : n * 3u;
}

/* Strips and fans are unrolled; out-of-range or repeated corners dropped. */
uint32_t *load_tris(const cgltf_primitive *p, uint32_t nverts,
                    uint32_t *ntri) {
  uint32_t n = p->indices ? (uint32_t)p->indices->count : nverts;
  uint32_t nt = p->type == cgltf_primitive_type_triangles ? n / 3u
                : n >= 3u                                   ? n - 2u
                                                            : 0u;
  uint32_t *out = nt ? malloc(nt * 3u * sizeof(uint32_t)) : NULL, k = 0;

  *ntri = 0;
  if (!out)
    return NULL;
  for (uint32_t t = 0; t < nt; t++) {
    uint32_t c[3], s[3];
    if (p->type == cgltf_primitive_type_triangles) {
      s[0] = t * 3u, s[1] = t * 3u + 1u, s[2] = t * 3u + 2u;
    } else if (p->type == cgltf_primitive_type_triangle_strip) {
      /* Every other strip triangle is wound the other way. */
      s[0] = t, s[1] = t + 1u + (t & 1u), s[2] = t + 2u - (t & 1u);
    } else {
      s[0] = 0, s[1] = t + 1u, s[2] = t + 2u;
    }
    for (int i = 0; i < 3; i++)
      c[i] = p->indices ? (uint32_t)cgltf_accessor_read_index(p->indices, s[i])
                        : s[i];
    if (c[0] >= nverts || c[1] >= nverts || c[2] >= nverts || c[0] == c[1] ||
        c[1] == c[2] || c[0] == c[2])
      continue;
    out[k++] = c[0];
    out[k++] = c[1];
    out[k++] = c[2];
  }
  *ntri = k / 3u;
  if (!k) {
    free(out);
    return NULL;
  }
  return out;
}

/* Base colour and emission from the factors; textures are not read. */
void load_mat(const cgltf_material *m, MatInfo *o) {
  o->rgba[0] = o->rgba[1] = o->rgba[2] = 0.8f;
  o->rgba[3] = 1.0f;
  o->emissive[0] = o->emissive[1] = o->emissive[2] = 0.0f;
  o->unlit = o->blend = false;
  if (!m)
    return;
  if (m->has_pbr_metallic_roughness)
    for (int i = 0; i < 4; i++)
      o->rgba[i] = m->pbr_metallic_roughness.base_color_factor[i];
  for (int i = 0; i < 3; i++)
    o->emissive[i] =
        m->emissive_factor[i] *
        (m->has_emissive_strength ? m->emissive_strength.emissive_strength
                                  : 1.0f);
  o->unlit = m->unlit != 0;
  o->blend = m->alpha_mode == cgltf_alpha_mode_blend && o->rgba[3] < 0.999f;
}

V3 load_read3(const cgltf_accessor *a, cgltf_size i, V3 dflt) {
  float f[4] = {dflt.x, dflt.y, dflt.z, 0.0f};
  if (a)
    cgltf_accessor_read_float(a, i, f, 4);
  return v3(f[0], f[1], f[2]);
}

void load_squash(const char *in, char *out, size_t n) {
  size_t k = 0;
  for (; in && *in && k + 1 < n; in++) {
    char c = *in;
    if (c >= 'A' && c <= 'Z')
      c = (char)(c - 'A' + 'a');
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))
      out[k++] = c;
  }
  out[k] = '\0';
}

bool load_in_scene(const cgltf_data *d, const cgltf_node *n) {
  const cgltf_scene *sc = d->scene ? d->scene : d->scenes_count ? d->scenes : 0;
  const cgltf_node *root = n;
  if (!sc)
    return true;
  while (root->parent)
    root = root->parent;
  for (cgltf_size i = 0; i < sc->nodes_count; i++)
    if (sc->nodes[i] == root)
      return true;
  return false;
}

bool load_mesh_alloc(Mesh *m, uint32_t nvtx, uint32_t nidx, uint32_t npart) {
  size_t vbytes = (size_t)nvtx * sizeof(Vtx);
  m->base = memalign(16, vbytes + (size_t)nidx * 2u + 16u);
  m->part = calloc(npart ? npart : 1u, sizeof(Part));
  if (!m->base || !m->part)
    return false;
  m->vtx = (Vtx *)m->base;
  m->idx = (uint16_t *)((uint8_t *)m->base + vbytes);
  m->nvtx = m->nidx = m->npart = 0;
  return true;
}

static float light(V3 n) {
  float k = v3_dot(n, g_light_key), f = v3_dot(n, g_light_fill);
  float l = LIGHT_AMBIENT + (k > 0.0f ? LIGHT_KEY * k : 0.0f) +
            (f > 0.0f ? LIGHT_FILL * f : 0.0f);
  return l > 1.0f ? 1.0f : l;
}

static bool is_spawn(const char *name) {
  static const char *const names[] = {"spawn", "playerstart", "start",
                                      "playerspawn", "spawnpoint"};
  char s[40];
  load_squash(name, s, sizeof(s));
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
    if (!strcmp(s, names[i]))
      return true;
  return false;
}

static bool no_collide(const char *name) {
  char s[64];
  load_squash(name, s, sizeof(s));
  return strstr(s, "nocollide") || strstr(s, "nocollision");
}

static void node_world(const cgltf_node *n, Aff *a) {
  float m[16];
  cgltf_node_transform_world(n, m);
  aff_from_gltf(a, m);
}

static int16_t q8_lo(float v) {
  return (int16_t)clampf(floorf(v * 256.0f) - 1.0f, -32767.0f, 32767.0f);
}

static int16_t q8_hi(float v) {
  return (int16_t)clampf(ceilf(v * 256.0f) + 1.0f, -32767.0f, 32767.0f);
}

static void ctri_add(World *w, V3 a, V3 b, V3 c) {
  CTri *t;
  V3 n = v3_cross(v3_sub(b, a), v3_sub(c, a));
  float l = v3_len(n);
  if (l < 1e-8f)
    return;
  t = &w->tri[w->ntri++];
  t->v0 = a;
  t->v1 = b;
  t->v2 = c;
  t->n = v3_scale(n, 1.0f / l);
  t->lo[0] = q8_lo(fminf(a.x, fminf(b.x, c.x)));
  t->lo[1] = q8_lo(fminf(a.y, fminf(b.y, c.y)));
  t->lo[2] = q8_lo(fminf(a.z, fminf(b.z, c.z)));
  t->hi[0] = q8_hi(fmaxf(a.x, fmaxf(b.x, c.x)));
  t->hi[1] = q8_hi(fmaxf(a.y, fmaxf(b.y, c.y)));
  t->hi[2] = q8_hi(fmaxf(a.z, fmaxf(b.z, c.z)));
}

/* A small part this detailed (Blender's Suzanne has 968 triangles) collides
 * as its bounding box: on the ARM9's soft float a jump beside it would
 * otherwise cost tens of milliseconds of closest-point tests. Large ones,
 * such as a room made as one mesh, keep their triangles. */
#define PROXY_TRIS 192u
#define PROXY_SIZE 1.2f

static void part_box(const Vtx *vb, uint32_t nv, V3 *lo, V3 *hi) {
  *lo = v3(1e30f, 1e30f, 1e30f);
  *hi = v3(-1e30f, -1e30f, -1e30f);
  for (uint32_t i = 0; i < nv; i++) {
    const float *p = vb[i].pos;
    *lo = v3(fminf(lo->x, p[0]), fminf(lo->y, p[1]), fminf(lo->z, p[2]));
    *hi = v3(fmaxf(hi->x, p[0]), fmaxf(hi->y, p[1]), fmaxf(hi->z, p[2]));
  }
}

static float part_size(const Vtx *vb, uint32_t nv) {
  V3 lo, hi;
  part_box(vb, nv, &lo, &hi);
  return fmaxf(hi.x - lo.x, fmaxf(hi.y - lo.y, hi.z - lo.z));
}

static void box_add(World *w, const Vtx *vb, uint32_t nv) {
  /* Corner k has bit 0 for +x, bit 1 for +y, bit 2 for +z. */
  static const uint8_t quad[6][4] = {{0, 2, 6, 4}, {1, 5, 7, 3},
                                     {0, 4, 5, 1}, {2, 3, 7, 6},
                                     {0, 1, 3, 2}, {4, 6, 7, 5}};
  V3 lo, hi, c[8];
  part_box(vb, nv, &lo, &hi);
  for (int k = 0; k < 8; k++)
    c[k] = v3(k & 1 ? hi.x : lo.x, k & 2 ? hi.y : lo.y, k & 4 ? hi.z : lo.z);
  for (int f = 0; f < 6; f++) {
    ctri_add(w, c[quad[f][0]], c[quad[f][1]], c[quad[f][2]]);
    ctri_add(w, c[quad[f][0]], c[quad[f][2]], c[quad[f][3]]);
  }
}

static void grow(World *w, V3 p) {
  w->bmin = v3(fminf(w->bmin.x, p.x), fminf(w->bmin.y, p.y),
               fminf(w->bmin.z, p.z));
  w->bmax = v3(fmaxf(w->bmax.x, p.x), fmaxf(w->bmax.y, p.y),
               fmaxf(w->bmax.z, p.z));
}

/* One triangle primitive of a room node: world space, lit colours, and its
 * collision triangles. */
static void room_prim(Room *r, const cgltf_primitive *pr, const Aff *wm,
                      bool collide) {
  const cgltf_accessor *pos = load_attr(pr, cgltf_attribute_type_position, 0);
  const cgltf_accessor *nrm = load_attr(pr, cgltf_attribute_type_normal, 0);
  const cgltf_accessor *col = load_attr(pr, cgltf_attribute_type_color, 0);
  uint32_t nv = (uint32_t)pos->count, nt, *tri;
  float nm[3][3];
  V3 *fn = NULL, ctr = v3(0, 0, 0);
  MatInfo mi;
  Mesh *m = &r->mesh;
  Vtx *vb = &m->vtx[m->nvtx];
  Part *part;

  tri = load_tris(pr, nv, &nt);
  if (!tri)
    return;
  load_mat(pr->material, &mi);
  aff_normal_matrix(nm, wm);
  if (!nrm && (fn = calloc(nv, sizeof(V3))) != NULL)
    for (uint32_t t = 0; t < nt; t++) {
      V3 a = aff_point(wm, load_read3(pos, tri[t * 3], v3(0, 0, 0)));
      V3 b = aff_point(wm, load_read3(pos, tri[t * 3 + 1], v3(0, 0, 0)));
      V3 c = aff_point(wm, load_read3(pos, tri[t * 3 + 2], v3(0, 0, 0)));
      V3 f = v3_cross(v3_sub(b, a), v3_sub(c, a));
      for (int k = 0; k < 3; k++)
        fn[tri[t * 3 + k]] = v3_add(fn[tri[t * 3 + k]], f);
    }

  part = &m->part[m->npart++];
  part->vfirst = m->nvtx;
  part->vcount = nv;
  part->ifirst = m->nidx;
  part->icount = nt * 3u;
  part->blend = mi.blend;

  for (uint32_t i = 0; i < nv; i++) {
    V3 p = aff_point(wm, load_read3(pos, i, v3(0, 0, 0))), n = v3(0, 1, 0);
    float vc[4] = {1.0f, 1.0f, 1.0f, 1.0f}, out[4], l;
    if (nrm) {
      V3 a = load_read3(nrm, i, v3(0, 1, 0));
      n = v3(nm[0][0] * a.x + nm[0][1] * a.y + nm[0][2] * a.z,
             nm[1][0] * a.x + nm[1][1] * a.y + nm[1][2] * a.z,
             nm[2][0] * a.x + nm[2][1] * a.y + nm[2][2] * a.z);
    } else if (fn) {
      n = fn[i];
    }
    n = v3_norm(n);
    if (col)
      cgltf_accessor_read_float(col, i, vc, cgltf_num_components(col->type));
    /* Darker towards the floor: a cheap stand-in for occlusion. */
    l = mi.unlit ? 1.0f
                 : light(n) * (0.82f + 0.18f * clampf(p.y / 1.5f, 0.0f, 1.0f));
    for (int k = 0; k < 3; k++)
      out[k] = srgb(mi.rgba[k] * vc[k] * l + mi.emissive[k]);
    out[3] = mi.rgba[3] * vc[3];
    vb[i].pos[0] = p.x;
    vb[i].pos[1] = p.y;
    vb[i].pos[2] = p.z;
    vb[i].pos[3] = 1.0f;
    vb[i].uv[0] = vb[i].uv[1] = 0.0f;
    for (int k = 0; k < 4; k++)
      vb[i].col[k] = out[k];
    ctr = v3_add(ctr, p);
    grow(&r->world, p);
  }
  part->centre = v3_scale(ctr, 1.0f / (float)nv);

  for (uint32_t t = 0; t < nt * 3u; t++)
    m->idx[m->nidx + t] = (uint16_t)tri[t];
  if (collide && nt > PROXY_TRIS && part_size(vb, nv) < PROXY_SIZE)
    box_add(&r->world, vb, nv);
  else if (collide)
    for (uint32_t t = 0; t < nt; t++) {
      const float *a = vb[tri[t * 3]].pos, *b = vb[tri[t * 3 + 1]].pos,
                  *c = vb[tri[t * 3 + 2]].pos;
      ctri_add(&r->world, v3(a[0], a[1], a[2]), v3(b[0], b[1], b[2]),
               v3(c[0], c[1], c[2]));
    }
  m->nvtx += nv;
  m->nidx += nt * 3u;
  r->ntris += nt;
  free(tri);
  free(fn);
}

static bool room_prim_ok(const cgltf_primitive *pr) {
  const cgltf_accessor *pos = load_attr(pr, cgltf_attribute_type_position, 0);
  return load_is_triangles(pr) && pos && pos->count && pos->count <= 65535u;
}

bool room_load(Room *r, const void *glb, size_t size, char *err, size_t errn) {
  cgltf_data *d;
  uint32_t nvtx = 0, nidx = 0, npart = 0;

  memset(r, 0, sizeof(*r));
  if (!load_parse(glb, size, &d, err, errn))
    return false;

  for (cgltf_size i = 0; i < d->nodes_count; i++) {
    const cgltf_node *n = &d->nodes[i];
    if (!n->mesh || !load_in_scene(d, n))
      continue;
    for (cgltf_size k = 0; k < n->mesh->primitives_count; k++) {
      const cgltf_primitive *pr = &n->mesh->primitives[k];
      if (!room_prim_ok(pr)) {
        r->skipped++;
        continue;
      }
      nvtx += (uint32_t)load_attr(pr, cgltf_attribute_type_position, 0)->count;
      nidx += load_index_bound(
          pr, (uint32_t)load_attr(pr, cgltf_attribute_type_position, 0)->count);
      npart++;
    }
  }
  if (!npart) {
    cgltf_free(d);
    return load_fail(err, errn, "it has no triangles");
  }
  r->world.tri = malloc((size_t)(nidx / 3u + 1u) * sizeof(CTri));
  if (!r->world.tri || !load_mesh_alloc(&r->mesh, nvtx, nidx, npart)) {
    cgltf_free(d);
    return load_fail(err, errn, "out of memory");
  }
  r->world.bmin = v3(1e30f, 1e30f, 1e30f);
  r->world.bmax = v3(-1e30f, -1e30f, -1e30f);

  for (cgltf_size i = 0; i < d->nodes_count; i++) {
    const cgltf_node *n = &d->nodes[i];
    Aff wm;
    if (!load_in_scene(d, n))
      continue;
    node_world(n, &wm);
    if (!r->has_spawn && is_spawn(n->name)) {
      r->has_spawn = true;
      r->spawn = v3(wm.m[0][3], wm.m[1][3], wm.m[2][3]);
      r->spawn_yaw = atan2f(wm.m[0][2], wm.m[2][2]);
      continue;
    }
    if (!n->mesh)
      continue;
    for (cgltf_size k = 0; k < n->mesh->primitives_count; k++)
      if (room_prim_ok(&n->mesh->primitives[k]))
        room_prim(r, &n->mesh->primitives[k], &wm, !no_collide(n->name));
  }
  cgltf_free(d);
  if (!r->mesh.nidx)
    return load_fail(err, errn, "it has no triangles");
  if (!world_build(&r->world))
    return load_fail(err, errn, "out of memory");
  return true;
}
