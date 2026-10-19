/* The PICA200 draws each frame from a command list built here with the OS's
 * P3d9.c and run by its ARM11 core (gpu_p3d): clear, draw, then copy the
 * frame out in the framebuffer layout. The setup follows the OS's 3D Model
 * screen (src/os/Model3D.c), which runs on hardware: the same shader and
 * vertex inputs, the immediate-mode backdrop first, then vertex buffers
 * drawn from 16-bit indices. See docs/stereo3d.md for P3D itself. */
#include "aurora.h"
#include "gpu.h"
#include "p3d.h"
#include "stereo.h"

#include "render.h"
#include "vmath.h"

#include "playtest_shbin.h" /* made from playtest.v.pica by build.py */

#include <malloc.h>
#include <string.h>

extern void os_cache_sync(void);

#define LIST_WORDS 0x8000u /* 128 KB */
#define NEAR       0.08f
#define FAR        80.0f
#define SHADOW_N   16

static u32 *list;
static u8 *right_eye;
static const char *why = "";
static Mesh shadow;
static u16 order[256];

/* Top to bottom: sky, horizon, below. */
static const float sky[3][4] = {{0.10f, 0.13f, 0.22f, 1.0f},
                                {0.30f, 0.36f, 0.46f, 1.0f},
                                {0.07f, 0.07f, 0.09f, 1.0f}};

/* A flat disc, fading from the centre: radius 1 on the XZ plane. */
static bool shadow_init(void) {
  Vtx *v;
  shadow.base = memalign(16, (SHADOW_N + 1) * sizeof(Vtx) + SHADOW_N * 6u + 16u);
  if (!shadow.base)
    return false;
  v = shadow.vtx = (Vtx *)shadow.base;
  shadow.idx = (u16 *)(v + SHADOW_N + 1);
  memset(v, 0, (SHADOW_N + 1) * sizeof(Vtx));
  for (int i = 0; i <= SHADOW_N; i++) {
    float a = 2.0f * PI_F * (float)(i - 1) / SHADOW_N;
    v[i].pos[0] = i ? cosf(a) : 0.0f;
    v[i].pos[2] = i ? sinf(a) : 0.0f;
    v[i].pos[3] = 1.0f;
  }
  for (int i = 0; i < SHADOW_N; i++) {
    shadow.idx[i * 3] = 0;
    shadow.idx[i * 3 + 1] = (u16)(1 + i);
    shadow.idx[i * 3 + 2] = (u16)(1 + (i + 1) % SHADOW_N);
  }
  shadow.nvtx = SHADOW_N + 1;
  shadow.nidx = SHADOW_N * 3;
  return true;
}

bool render_init(void) {
  if (!gpu_alive())
    return false;
  list = memalign(16, LIST_WORDS * 4u);
  right_eye = memalign(16, TOP_FB_SIZE);
  return list && right_eye && shadow_init();
}

uint32_t render_top_back(void) { return (u32)VRAM_TOP_BACK; }
uint32_t render_right(void) { return (u32)right_eye; }

static void to_p3d(P3dMtx *o, const Aff *a) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 4; c++)
      o->m[r][c] = a->m[r][c];
  o->m[3][0] = o->m[3][1] = o->m[3][2] = 0.0f;
  o->m[3][3] = 1.0f;
}

static void view_matrix(P3dMtx *m, const View *v) {
  const V3 *rows[3] = {&v->right, &v->up, &v->back};
  for (int r = 0; r < 3; r++) {
    m->m[r][0] = rows[r]->x;
    m->m[r][1] = rows[r]->y;
    m->m[r][2] = rows[r]->z;
    m->m[r][3] = -v3_dot(*rows[r], v->eye);
  }
  m->m[3][0] = m->m[3][1] = m->m[3][2] = 0.0f;
  m->m[3][3] = 1.0f;
}

static void backdrop(P3dList *l) {
  P3dMtx ortho;
  const float rows[3] = {0.0f, TOP_SCREEN_HEIGHT * 0.55f, TOP_SCREEN_HEIGHT};
  const float uv[4] = {0.0f, 0.0f, 0.0f, 1.0f};

  p3d_ortho_tilt(&ortho, 0.0f, TOP_SCREEN_WIDTH, TOP_SCREEN_HEIGHT, 0.0f, 1.0f,
                 -1.0f);
  p3d_depth(l, 0);
  p3d_texture_off(l);
  p3d_stage0(l, 0, P3D_ALPHA_ONE);
  p3d_uniform(l, PLAYTEST_SHADER_UNIF_MVP, &ortho);
  p3d_draw_begin(l, P3D_TRIANGLES);
  for (int b = 0; b < 2; b++) {
    const float x0 = 0.0f, x1 = TOP_SCREEN_WIDTH;
    const float a[4] = {x0, rows[b], 0.0f, 1.0f}, c[4] = {x1, rows[b], 0.0f, 1.0f};
    const float d[4] = {x1, rows[b + 1], 0.0f, 1.0f},
                e[4] = {x0, rows[b + 1], 0.0f, 1.0f};
    p3d_vertex3(l, a, uv, sky[b]);
    p3d_vertex3(l, c, uv, sky[b]);
    p3d_vertex3(l, d, uv, sky[b + 1]);
    p3d_vertex3(l, a, uv, sky[b]);
    p3d_vertex3(l, d, uv, sky[b + 1]);
    p3d_vertex3(l, e, uv, sky[b + 1]);
  }
  p3d_draw_end(l);
}

static void draw_part(P3dList *l, const Mesh *m, const Part *p) {
  p3d_buffer0(l, p->vfirst * sizeof(Vtx), sizeof(Vtx), 3);
  p3d_draw_elements(l, (u32)((u8 *)&m->idx[p->ifirst] - (u8 *)m->base),
                    p->icount);
}

static void draw_mesh(P3dList *l, const Mesh *m, const P3dMtx *mvp,
                      bool blend) {
  p3d_uniform(l, PLAYTEST_SHADER_UNIF_MVP, mvp);
  p3d_buffers(l, (u32)m->base);
  for (u32 i = 0; i < m->npart; i++)
    if (m->part[i].blend == blend && m->part[i].icount)
      draw_part(l, m, &m->part[i]);
}

/* Translucent parts far to near; view z grows towards the viewer. */
static void draw_blended(P3dList *l, const Mesh *m, const P3dMtx *mvp,
                         const P3dMtx *view) {
  u32 n = 0;
  for (u32 i = 0; i < m->npart && n < 256u; i++) {
    const V3 c = m->part[i].centre;
    float z;
    u32 j;
    if (!m->part[i].blend || !m->part[i].icount)
      continue;
    z = view->m[2][0] * c.x + view->m[2][1] * c.y + view->m[2][2] * c.z;
    for (j = n; j > 0; j--) {
      const V3 d = m->part[order[j - 1]].centre;
      if (view->m[2][0] * d.x + view->m[2][1] * d.y + view->m[2][2] * d.z <= z)
        break;
      order[j] = order[j - 1];
    }
    order[j] = (u16)i;
    n++;
  }
  if (!n)
    return;
  p3d_uniform(l, PLAYTEST_SHADER_UNIF_MVP, mvp);
  p3d_buffers(l, (u32)m->base);
  for (u32 k = 0; k < n; k++)
    draw_part(l, m, &m->part[order[k]]);
}

bool render_eye(const Room *room, const Player *pl, const View *v, float eye,
                float conv, uint32_t dst) {
  static const u8 inputs[3] = {P3D_FMT_FLOAT4, P3D_FMT_FLOAT2, P3D_FMT_FLOAT4};
  P3dList l;
  P3dMtx proj, view, vp, mvp, m4;
  Aff sh;
  u32 bytes;

  p3d_persp_stereo_tilt(&proj, v->tan_half,
                        (float)TOP_SCREEN_WIDTH / TOP_SCREEN_HEIGHT, NEAR, FAR,
                        eye, conv);
  view_matrix(&view, v);
  p3d_mtx_mul(&vp, &proj, &view);

  p3d_begin(&l, list, LIST_WORDS);
  p3d_target(&l);
  p3d_shader(&l, &playtest_shader);
  p3d_inputs(&l, inputs, 3);
  p3d_vertex_colour(&l);
  backdrop(&l);

  p3d_depth(&l, 1);
  p3d_texture_off(&l);
  p3d_stage0(&l, 0, P3D_ALPHA_ONE);
  p3d_fragment(&l, 1, 1, -1);
  draw_mesh(&l, &room->mesh, &vp, false);

  if (v->shadow_a > 0.0f) {
    shadow.vtx[0].col[3] = v->shadow_a;
    aff_identity(&sh);
    sh.m[0][0] = sh.m[2][2] = v->shadow_r;
    sh.m[0][3] = v->shadow.x;
    sh.m[1][3] = v->shadow.y;
    sh.m[2][3] = v->shadow.z;
    to_p3d(&m4, &sh);
    p3d_mtx_mul(&mvp, &vp, &m4);
    p3d_stage0(&l, 0, P3D_ALPHA_VERTEX);
    p3d_fragment(&l, 1, 0, -1);
    p3d_uniform(&l, PLAYTEST_SHADER_UNIF_MVP, &mvp);
    p3d_buffers(&l, (u32)shadow.base);
    p3d_buffer0(&l, 0, sizeof(Vtx), 3);
    p3d_draw_elements(&l, (u32)((u8 *)shadow.idx - (u8 *)shadow.base),
                      shadow.nidx);
  }

  to_p3d(&m4, &v->player);
  p3d_mtx_mul(&mvp, &vp, &m4);
  /* A faded player is drawn after the room, over it, nearest part first
   * where its depth writes keep its own back parts out. */
  if (v->player_alpha >= 0.999f) {
    p3d_stage0(&l, 0, P3D_ALPHA_ONE);
    p3d_fragment(&l, 1, 1, -1);
    draw_mesh(&l, &pl->mesh, &mvp, false);
  }

  p3d_stage0(&l, 0, P3D_ALPHA_VERTEX);
  p3d_fragment(&l, 1, 0, -1);
  draw_blended(&l, &room->mesh, &vp, &view);
  if (v->player_alpha > 0.0f) {
    if (v->player_alpha < 0.999f) {
      p3d_fragment(&l, 1, 1, -1);
      draw_mesh(&l, &pl->mesh, &mvp, false);
      p3d_fragment(&l, 1, 0, -1);
    }
    draw_blended(&l, &pl->mesh, &mvp, &view);
  }

  p3d_flush(&l);
  bytes = p3d_end(&l);
  if (!bytes) {
    why = "the command list is too long";
    return false;
  }
  /* The GPU writes `dst`: nothing of it may stay in the cache, either to be
   * written back over the frame or read instead of it. */
  os_cache_sync();
  if (!gpu_p3d((u32)list, bytes, dst, 0x000000FFu)) {
    u32 stereo, writes, state, runs, waits, stat;
    gpu_3d_info(&stereo, &writes, &state, &runs, &waits, &stat);
    why = state == GPU_P3D_CLEAR_TIMEOUT  ? "the GPU's clear timed out"
          : state == GPU_P3D_LIST_TIMEOUT ? "the command list never finished"
          : state == GPU_P3D_COPY_TIMEOUT ? "the copy to the screen timed out"
                                          : "the GPU did not answer";
    return false;
  }
  return true;
}

const char *render_error(void) { return why; }

bool render_stereo_capable(void) { return stereo_capable() != 0; }

int render_slider(void) { return stereo_slider(stereo_slider_raw()); }

bool render_stereo(bool on) {
  gpu_wait_idle();
  return stereo_enable(on ? 1 : 0, STEREO_POS_FIXED) != 0;
}

void render_present_stereo(bool bottom) {
  gpu_present_st_async((u32)VRAM_TOP_BACK, (u32)right_eye,
                       bottom ? (u32)VRAM_BOT_BACK : 0);
}

void render_wait(void) { gpu_wait_idle(); }
