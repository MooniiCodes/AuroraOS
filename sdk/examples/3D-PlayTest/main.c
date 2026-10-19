// 3D PlayTest: walk a rigged player around a room, drawn by the PICA200.
//
// Circle Pad moves (B runs, A jumps). The camera turns with the C-stick on a
// New 3DS and with the D-pad on any model; L puts it behind the player, Y
// changes its distance and X turns its up and down around. The 3D slider
// gives stereoscopic 3D. SELECT respawns, START or HOME leaves.
//
// The room and the player are .glb files built into the app; player.glb and
// room.glb in the app's data folder (SD:/Aurora/Apps/C/3D-PlayTest) are used
// instead when present. Build with build.py; see info.md.
#include <aurora_app.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cstick.h"
#include "game.h"
#include "render.h"
#include "vmath.h"

/* build/apps/3D-PlayTest/gen/assets.s */
extern const uint8_t playtest_player_glb[], playtest_player_glb_end[];
extern const uint8_t playtest_room_glb[], playtest_room_glb_end[];

#define WALK_SPEED   2.4f /* m/s, circle pad all the way */
#define RUN_SPEED    4.6f
#define ACCEL_GROUND 12.0f
#define ACCEL_AIR    2.5f
#define TURN_RATE    11.0f /* rad/s */
#define GRAVITY      16.0f
#define JUMP_SPEED   5.0f /* about 0.8 m up */
#define STEP         0.32f /* climbed without jumping */
#define FALL_LIMIT   8.0f /* below the room, back to the start */
/* Walking stops at an edge with nothing this far below it; jumping off
 * still falls. */
#define LEDGE_DROP   2.5f

#define CAM_YAW_RATE   2.6f /* rad/s at full deflection */
#define CAM_PITCH_RATE 1.5f
#define CAM_PITCH_MIN  (-0.20f)
#define CAM_PITCH_MAX  1.20f
#define CAM_PAD        0.25f /* kept between the camera and a wall */
#define CAM_MIN        0.15f
/* Nearer than FADE_FAR the player fades, and from FADE_NEAR it is hidden,
 * so a camera pushed in by a wall still shows the room. */
#define FADE_FAR       1.0f
#define FADE_NEAR      0.4f
#define TAN_HALF       0.55f
/* The eyes' separation with the slider at the top, as a fraction of the
 * distance to the player, who sits on the screen's surface. */
#define EYE_FRAC 0.035f

#define HUD_US 250000u

static const float cam_dists[3] = {2.3f, 3.4f, 5.2f};

typedef struct {
  V3 pos, vel;
  float facing; /* radians; 0 faces +Z */
  float speed;  /* over the ground, after collisions */
  bool ground;
  AnimState anim;
} Body;

typedef struct {
  float yaw, pitch, dist;
  int zoom;
  bool invert;
  float behind; /* seconds left of turning behind the player */
  V3 target;
} Cam;

static Room room;
static Player player;
static Aff joints[MAX_JOINTS];
static Body body;
static Cam cam;
static CStick cs;
static bool n3ds, stereo;
static V3 spawn;
static float spawn_yaw;
static char room_src[16], player_src[16];

static struct {
  uint32_t frames, frame_us, cpu_us, gpu_us;
  int fps10, frame_ms10, cpu_ms10, gpu_ms10;
  uint32_t fails;
  int slider;
} st;

static float wrap_pi(float a) {
  while (a > PI_F)
    a -= 2.0f * PI_F;
  while (a < -PI_F)
    a += 2.0f * PI_F;
  return a;
}

static void message(const char *title, const char *text) {
  gfx_clear(SCREEN_TOP, RGB(16, 18, 28));
  gfx_print(SCREEN_TOP, 16, 16, FONT_TITLE, COLOR_WHITE, "%s", title);
  gfx_print(SCREEN_TOP, 16, 50, FONT_REGULAR, RGB(200, 205, 220), "%s", text);
  gfx_clear(SCREEN_BOTTOM, RGB(28, 28, 34));
  gfx_print(SCREEN_BOTTOM, 16, 16, FONT_REGULAR, RGB(200, 205, 220),
            "START or HOME: back to the Home Menu");
  gfx_present();
}

static void fatal(const char *title, const char *text) {
  message(title, text);
  hid_wait(KEY_START);
  exit(0);
}

/* The data folder's copy when there is one, else the built-in file. */
static const void *asset(const char *const *names, const uint8_t *in,
                         const uint8_t *in_end, size_t *size, char *src) {
  char path[300];
  for (; *names; names++) {
    void *d;
    snprintf(path, sizeof(path), "%s/%s", app_dir(), *names);
    if (fs_exists(path) && (d = fs_read_file(path, size)) != NULL) {
      snprintf(src, 16, "SD card");
      return d;
    }
  }
  snprintf(src, 16, "built in");
  *size = (size_t)(in_end - in);
  return in;
}

static void load(void) {
  static const char *const room_names[] = {"room.glb", "test_room.glb", 0};
  static const char *const player_names[] = {"player.glb", 0};
  char err[96], text[160];
  size_t n;
  const void *d;

  message("3D PlayTest", "Loading the room...");
  d = asset(room_names, playtest_room_glb, playtest_room_glb_end, &n, room_src);
  if (!room_load(&room, d, n, err, sizeof(err))) {
    snprintf(text, sizeof(text), "The room (%s) could not be loaded:\n%s",
             room_src, err);
    fatal("3D PlayTest", text);
  }
  if (d != playtest_room_glb)
    free((void *)d);

  message("3D PlayTest", "Loading the player...");
  d = asset(player_names, playtest_player_glb, playtest_player_glb_end, &n,
            player_src);
  if (!player_load(&player, d, n, err, sizeof(err))) {
    snprintf(text, sizeof(text), "The player (%s) could not be loaded:\n%s",
             player_src, err);
    fatal("3D PlayTest", text);
  }
  if (d != playtest_player_glb)
    free((void *)d);
}

/* The room's spawn node, or a point towards the room's open front; then the
 * nearest place there where the player stands on something and fits. */
static void find_spawn(void) {
  World *w = &room.world;
  float r = player.radius, h = player.height;
  V3 c;

  if (room.has_spawn) {
    c = room.spawn;
    spawn_yaw = room.spawn_yaw;
  } else {
    c = v3(0.5f * (w->bmin.x + w->bmax.x), w->bmin.y,
           0.5f * (w->bmin.z + w->bmax.z) + 0.2f * (w->bmax.z - w->bmin.z));
    spawn_yaw = PI_F; /* facing -Z, into the room */
  }
  for (int ring = 0; ring <= 12; ring++) {
    int around = ring ? ring * 8 : 1;
    for (int k = 0; k < around; k++) {
      float a = 2.0f * PI_F * (float)k / (float)around;
      float x = c.x + 0.25f * ring * cosf(a), z = c.z + 0.25f * ring * sinf(a);
      float g = world_ground(w, v3(x, c.y + 1.0f, z), r, 0.0f,
                             w->bmax.y - w->bmin.y + 2.0f);
      if (g > -1e29f && world_free(w, v3(x, g, z), r, h, STEP)) {
        spawn = v3(x, g, z);
        return;
      }
    }
  }
  spawn = c;
}

static void respawn(void) {
  memset(&body, 0, sizeof(body));
  body.pos = spawn;
  body.facing = spawn_yaw;
  body.ground = true;
  cam.yaw = wrap_pi(spawn_yaw + PI_F);
  cam.pitch = 0.32f;
  cam.dist = cam_dists[cam.zoom];
  cam.target = v3(spawn.x, spawn.y + player.height * 0.78f, spawn.z);
  cam.behind = 0.0f;
}

static void leave_3d(void) {
  if (stereo)
    render_stereo(false);
  stereo = false;
}

static void move_camera(float dt, uint32_t held, uint32_t down) {
  float cx = 0.0f, cy = 0.0f;

  if (n3ds && cs.misses < CSTICK_GIVE_UP) {
    cstick_read(&cs);
    cx = cs.x / 127.0f;
    cy = cs.y / 127.0f;
  }
  if (held & KEY_LEFT)
    cx -= 1.0f;
  if (held & KEY_RIGHT)
    cx += 1.0f;
  if (held & KEY_UP)
    cy += 1.0f;
  if (held & KEY_DOWN)
    cy -= 1.0f;
  cx = clampf(cx, -1.0f, 1.0f);
  cy = clampf(cy, -1.0f, 1.0f);
  if (down & KEY_X)
    cam.invert = !cam.invert;
  if (down & KEY_Y)
    cam.zoom = (cam.zoom + 1) % 3;
  if (down & KEY_L)
    cam.behind = 0.35f;

  /* Right turns the view right; up raises the camera unless inverted. */
  cam.yaw = wrap_pi(cam.yaw - cx * CAM_YAW_RATE * dt);
  cam.pitch += (cam.invert ? -cy : cy) * CAM_PITCH_RATE * dt;
  cam.pitch = clampf(cam.pitch, CAM_PITCH_MIN, CAM_PITCH_MAX);
  if (cam.behind > 0.0f) {
    float want = wrap_pi(body.facing + PI_F);
    float k = clampf(dt / cam.behind, 0.0f, 1.0f);
    cam.yaw = wrap_pi(cam.yaw + wrap_pi(want - cam.yaw) * k);
    cam.behind -= dt;
  }
}

static bool floor_below(float x, float z, float y) {
  return world_ground(&room.world, v3(x, y, z), player.radius, STEP,
                      LEDGE_DROP) > -1e29f;
}

static void move_body(float dt, uint32_t held, uint32_t down) {
  World *w = &room.world;
  int px, py;
  float mx, my, mag, top, k, sy = sinf(cam.yaw), cyaw = cosf(cam.yaw);
  V3 wish, np, old = body.pos;

  hid_cpad(&px, &py);
  mx = px / 127.0f;
  my = py / 127.0f;
  mag = sqrtf(mx * mx + my * my);
  if (mag > 1.0f) {
    mx /= mag;
    my /= mag;
    mag = 1.0f;
  }
  /* Up on the pad is away from the camera. */
  wish = v3(-sy * my + cyaw * mx, 0.0f, -cyaw * my - sy * mx);
  top = (held & KEY_B) ? RUN_SPEED : WALK_SPEED;
  k = clampf((body.ground ? ACCEL_GROUND : ACCEL_AIR) * dt, 0.0f, 1.0f);
  body.vel.x += (wish.x * top - body.vel.x) * k;
  body.vel.z += (wish.z * top - body.vel.z) * k;

  if (mag > 0.1f) {
    float want = atan2f(wish.x, wish.z);
    float turn = wrap_pi(want - body.facing), max = TURN_RATE * dt;
    body.facing = wrap_pi(body.facing + clampf(turn, -max, max));
  }
  if ((down & KEY_A) && body.ground) {
    body.vel.y = JUMP_SPEED;
    body.ground = false;
  }
  if (!body.ground)
    body.vel.y -= GRAVITY * dt;

  np = v3(old.x + body.vel.x * dt, old.y, old.z + body.vel.z * dt);
  /* The room does not move, so standing still cannot run into it. */
  if (!body.ground || body.vel.x != 0.0f || body.vel.z != 0.0f)
    np = world_push(w, np, player.radius, player.height, STEP);
  np.y = old.y + body.vel.y * dt;

  if (body.ground) {
    /* A step up or down keeps the player on the ground and a deeper drop is
     * walked off, but with nothing below at all the edge stops the player,
     * who slides along it if one axis still has a floor underfoot. */
    float g = world_ground(w, v3(np.x, old.y, np.z), player.radius, STEP,
                           LEDGE_DROP);
    if (g <= -1e29f) {
      if (floor_below(np.x, old.z, old.y))
        np.z = old.z, body.vel.z = 0.0f;
      else if (floor_below(old.x, np.z, old.y))
        np.x = old.x, body.vel.x = 0.0f;
      else
        np.x = old.x, np.z = old.z, body.vel.x = body.vel.z = 0.0f;
      g = world_ground(w, v3(np.x, old.y, np.z), player.radius, STEP,
                       LEDGE_DROP);
    }
    if (g > -1e29f && g >= old.y - STEP) {
      np.y = g;
      body.vel.y = 0.0f;
    } else {
      np.y = old.y;
      body.ground = false;
    }
  } else if (body.vel.y <= 0.0f) {
    /* Landing takes a ledge up to a step above the feet, as walking does. */
    float g = world_ground(w, v3(np.x, old.y, np.z), player.radius, STEP,
                           old.y - np.y + 0.05f);
    if (g > -1e29f) {
      np.y = g;
      body.vel.y = 0.0f;
      body.ground = true;
    }
  }
  body.speed = sqrtf((np.x - old.x) * (np.x - old.x) +
                     (np.z - old.z) * (np.z - old.z)) / dt;
  /* After a wall, the velocity is what was left of it: along the wall. */
  if (body.speed <
      0.98f * sqrtf(body.vel.x * body.vel.x + body.vel.z * body.vel.z)) {
    body.vel.x = (np.x - old.x) / dt;
    body.vel.z = (np.z - old.z) / dt;
  }
  body.pos = np;
  if (body.pos.y < w->bmin.y - FALL_LIMIT || (down & KEY_SELECT))
    respawn();
}

static void animate(float dt) {
  AnimState *a = &body.anim;
  float sp = clampf(body.speed, 0.0f, RUN_SPEED * 1.2f);
  float walk = body.ground ? clampf(sp / 1.0f, 0.0f, 1.0f) : 0.0f;
  float cycle = 1.2f + 0.4f * sp; /* metres a full stride cycle covers */

  a->t += dt;
  a->speed += (sp - a->speed) * clampf(dt * 10.0f, 0.0f, 1.0f);
  a->walk += (walk - a->walk) * clampf(dt * 8.0f, 0.0f, 1.0f);
  a->air += ((body.ground ? 0.0f : 1.0f) - a->air) * clampf(dt * 10.0f, 0.0f, 1.0f);
  a->phase += 2.0f * PI_F * sp / cycle * dt;
  if (a->phase > 2.0f * PI_F)
    a->phase -= 2.0f * PI_F;
}

static void follow(float dt) {
  float want = cam_dists[cam.zoom], reach, hit, allowed;
  float cp = cosf(cam.pitch);
  V3 tgt = v3(body.pos.x, body.pos.y + player.height * 0.78f, body.pos.z);
  V3 back = v3(cp * sinf(cam.yaw), sinf(cam.pitch), cp * cosf(cam.yaw));

  cam.target = v3_lerp(cam.target, tgt, clampf(dt * 12.0f, 0.0f, 1.0f));
  /* Walls between the player and the camera pull it in at once; it backs
   * out again slowly. */
  reach = want + CAM_PAD;
  hit = world_ray(&room.world, cam.target,
                  v3_add(cam.target, v3_scale(back, reach)));
  allowed = fmaxf(CAM_MIN, hit * reach - CAM_PAD);
  if (allowed > want)
    allowed = want;
  if (allowed < cam.dist)
    cam.dist = allowed;
  else
    cam.dist += (allowed - cam.dist) * clampf(dt * 3.0f, 0.0f, 1.0f);
}

static float player_fade(void) {
  return clampf((cam.dist - FADE_NEAR) / (FADE_FAR - FADE_NEAR), 0.0f, 1.0f);
}

static void make_view(View *v) {
  float sy = sinf(cam.yaw), cy = cosf(cam.yaw);
  float sp = sinf(cam.pitch), cp = cosf(cam.pitch);
  float g, h;

  v->back = v3(cp * sy, sp, cp * cy);
  v->right = v3(cy, 0.0f, -sy);
  v->up = v3(-sp * sy, cp, -sp * cy);
  v->eye = v3_add(cam.target, v3_scale(v->back, cam.dist));
  v->tan_half = TAN_HALF;
  aff_yaw(&v->player, body.pos, body.facing);
  v->player_alpha = player_fade();

  g = world_ground(&room.world, body.pos, player.radius, 0.1f, 4.0f);
  v->shadow_a = 0.0f;
  if (g > -1e29f) {
    h = body.pos.y - g;
    v->shadow = v3(body.pos.x, g + 0.012f, body.pos.z);
    v->shadow_a = 0.42f * clampf(1.0f - h / 2.5f, 0.0f, 1.0f);
    v->shadow_r = player.radius * (1.3f + 0.15f * h);
  }
}

/* The lights in the player's own space: the world's turned back by the
 * player's facing. */
static void skin(void) {
  float c = cosf(body.facing), s = sinf(body.facing);
  V3 k = g_light_key, f = g_light_fill;
  V3 km = v3(k.x * c - k.z * s, k.y, k.x * s + k.z * c);
  V3 fm = v3(f.x * c - f.z * s, f.y, f.x * s + f.z * c);
  anim_pose(&player, &body.anim, joints);
  skin_apply(&player, joints, km, fm, player_fade());
}

static int tenths(uint32_t us_sum, uint32_t n) {
  return n ? (int)((us_sum / n + 50u) / 100u) : 0;
}

static void draw_bottom(void) {
  const Screen B = SCREEN_BOTTOM;
  const uint32_t dim = RGB(170, 178, 196), hi = RGB(236, 240, 248);
  int y = 40;

  gfx_gradient(B, 0, 0, 320, 240, RGB(30, 34, 46), RGB(14, 15, 20));
  gfx_print(B, 12, 8, FONT_TITLE, COLOR_WHITE, "3D PlayTest");
  gfx_print(B, 150, 12, FONT_REGULAR, COLOR_AURORA, "%d.%d fps", st.fps10 / 10,
            st.fps10 % 10);
  gfx_print(B, 238, 12, FONT_SMALL, dim, "%s", stereo ? "3D on" : "2D");

  gfx_round_rect(B, 8, y - 4, 304, 126, 8, RGB(40, 45, 60));
  gfx_print(B, 16, y, FONT_SMALL, dim,
            "frame %d.%d ms   update %d.%d ms   GPU %d.%d ms",
            st.frame_ms10 / 10, st.frame_ms10 % 10, st.cpu_ms10 / 10,
            st.cpu_ms10 % 10, st.gpu_ms10 / 10, st.gpu_ms10 % 10);
  y += 15;
  if (n3ds)
    gfx_print(B, 16, y, FONT_SMALL, cs.ok ? hi : COLOR_ORANGE,
              "New 3DS: camera on the C-stick%s   %d %d%s%s",
              cs.misses >= CSTICK_GIVE_UP ? " (silent: D-pad)"
              : cs.ok                     ? ""
                                          : " (no answer)",
              cs.x, cs.y, cs.zl ? " ZL" : "", cs.zr ? " ZR" : "");
  else
    gfx_print(B, 16, y, FONT_SMALL, hi, "Old 3DS: camera on the D-pad");
  y += 15;
  if (n3ds)
    gfx_print(B, 16, y, FONT_SMALL, dim,
              "C-stick bytes %02X %02X %02X %02X %02X %02X %02X  %u/%u",
              cs.raw[0], cs.raw[1], cs.raw[2], cs.raw[3], cs.raw[4], cs.raw[5],
              cs.raw[6], (unsigned)cs.good, (unsigned)cs.reads);
  else
    gfx_print(B, 16, y, FONT_SMALL, dim, "The D-pad turns the camera on any model");
  y += 15;
  if (!render_stereo_capable())
    gfx_print(B, 16, y, FONT_SMALL, dim, "3D: this model has no 3D screen");
  else
    gfx_print(B, 16, y, FONT_SMALL, dim, "3D slider %d%%%s",
              st.slider * 100 / 256, stereo ? ", stereoscopic" : "");
  y += 15;
  gfx_print(B, 16, y, FONT_SMALL, dim, "Room: %u triangles (%s)",
            (unsigned)room.ntris, room_src);
  y += 15;
  gfx_print(B, 16, y, FONT_SMALL, dim, "Player: %u triangles, %d bones (%s)",
            (unsigned)player.ntris, player.njoints, player_src);
  y += 15;
  gfx_print(B, 16, y, FONT_SMALL, dim, "Animation: %s%s", anim_describe(&player),
            body.ground ? "" : ", in the air");
  y += 15;
  gfx_print(B, 16, y, FONT_SMALL, dim, "Position %d, %d, %d cm   %d cm/s",
            (int)lrintf(body.pos.x * 100.0f), (int)lrintf(body.pos.y * 100.0f),
            (int)lrintf(body.pos.z * 100.0f), (int)lrintf(body.speed * 100.0f));
  if (st.fails)
    gfx_print(B, 16, 170, FONT_SMALL, COLOR_ORANGE, "GPU: %s (%u frames)",
              render_error(), (unsigned)st.fails);

  gfx_print(B, 12, 186, FONT_SMALL, hi, "Circle Pad: move    B: run    A: jump");
  gfx_print(B, 12, 200, FONT_SMALL, hi, "%s: camera    L: behind    Y: zoom",
            n3ds ? "C-stick / D-pad" : "D-pad");
  gfx_print(B, 12, 214, FONT_SMALL, hi, "X: invert camera%s    SELECT: respawn",
            cam.invert ? " (on)" : "");
  gfx_print(B, 12, 226, FONT_SMALL, dim, "START or HOME: back to the Home Menu");
}

static bool draw_top(float conv, int level) {
  View v;
  bool ok;
  float eye = EYE_FRAC * conv * 0.5f * (float)level / 256.0f;

  make_view(&v);
  if (stereo) {
    render_wait();
    ok = render_eye(&room, &player, &v, -eye, conv, render_top_back()) &&
         render_eye(&room, &player, &v, eye, conv, render_right());
  } else {
    uint8_t *fb = gfx_framebuffer(SCREEN_TOP);
    ok = render_eye(&room, &player, &v, 0.0f, conv, (uint32_t)fb);
    if (ok)
      gfx_print(SCREEN_TOP, 6, 4, FONT_SMALL, RGB(220, 228, 240), "%d fps",
                (st.fps10 + 5) / 10);
  }
  if (!ok) {
    st.fails++;
    if (!stereo) {
      gfx_clear(SCREEN_TOP, RGB(40, 16, 20));
      gfx_print(SCREEN_TOP, 16, 16, FONT_REGULAR, COLOR_WHITE,
                "The GPU did not draw the frame:\n%s", render_error());
    }
  }
  return ok;
}

int main(void) {
  uint32_t t_last, t_hud;
  bool hud_due = true;

  if (!app_from_home() || !render_init())
    fatal("3D PlayTest",
          "The 3D view needs the GPU, which the OS's ARM11 core runs.\n"
          "Start the app from the Home Menu, Files or the Terminal.");
  n3ds = sys_is_new3ds();
  atexit(leave_3d);
  cam.zoom = 1;

  load();
  find_spawn();
  respawn();
  animate(0.0f);

  t_last = t_hud = (uint32_t)sys_micros();
  for (;;) {
    uint32_t now, dt_us, t0, t1, t2, held, down;
    float dt;
    int level;
    bool want3d, bottom = false;

    if (stereo)
      hid_scan();
    else if (!app_loop())
      break;
    held = hid_keys_held();
    down = hid_keys_down();
    if (down & KEY_START)
      break;

    now = (uint32_t)sys_micros();
    dt_us = now - t_last;
    t_last = now;
    if (dt_us > 50000u)
      dt_us = 50000u;
    if (dt_us < 1000u)
      dt_us = 1000u;
    dt = dt_us / 1000000.0f;
    st.frames++;
    st.frame_us += dt_us;

    t0 = (uint32_t)sys_micros();
    move_camera(dt, held, down);
    move_body(dt, held, down);
    animate(dt);
    follow(dt);
    skin();

    level = render_stereo_capable() ? render_slider() : 0;
    st.slider = level;
    want3d = level > 0;
    if (want3d != stereo && render_stereo(want3d)) {
      stereo = want3d;
      hud_due = true;
    }
    t1 = (uint32_t)sys_micros();
    draw_top(cam.dist, level);
    t2 = (uint32_t)sys_micros();
    st.cpu_us += t1 - t0;
    st.gpu_us += t2 - t1;

    if (now - t_hud >= HUD_US || hud_due) {
      if (st.frames && now != t_hud) {
        st.fps10 = (int)((uint64_t)st.frames * 10000000u / (now - t_hud));
        st.frame_ms10 = tenths(st.frame_us, st.frames);
        st.cpu_ms10 = tenths(st.cpu_us, st.frames);
        st.gpu_ms10 = tenths(st.gpu_us, st.frames);
      }
      st.frames = st.frame_us = st.cpu_us = st.gpu_us = 0;
      t_hud = now;
      draw_bottom();
      bottom = true;
    }
    /* In 2D the next app_loop() presents; stereo frames go out here. */
    if (stereo)
      render_present_stereo(bottom);
    hud_due = false;
  }
  return 0;
}
