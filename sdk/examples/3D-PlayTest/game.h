/* Shared by the 3D PlayTest sources. Units are metres, +Y is up, and models
 * face +Z, glTF's forward. */
#ifndef PLAYTEST_GAME_H
#define PLAYTEST_GAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  float x, y, z;
} V3;

typedef struct {
  float x, y, z, w;
} Quat;

/* Row-major 3x4 affine transform: columns 0-2 rotate and scale, column 3
 * translates. */
typedef struct {
  float m[3][4];
} Aff;

/* What the GPU reads per vertex, laid out as the OS's GlbVertex so the same
 * shader inputs apply: position (w = 1), texture coordinate, and the colour
 * with the lighting baked in. */
typedef struct {
  float pos[4];
  float uv[2];
  float col[4];
} Vtx;

typedef struct {
  uint32_t vfirst, vcount;
  uint32_t ifirst, icount; /* into the mesh's index array */
  bool blend;              /* drawn after the opaque parts, far to near */
  V3 centre;
} Part;

/* Vertices, then 16-bit indices, in one block the GPU reads: `base` is the
 * attribute base the draw offsets count from. */
typedef struct {
  void *base;
  Vtx *vtx;
  uint32_t nvtx;
  uint16_t *idx;
  uint32_t nidx;
  Part *part;
  uint32_t npart;
} Mesh;

/* Collision. Triangles in world space; `lo`/`hi` bound each one in 1/256 m,
 * rounded outwards, so most are rejected with integer compares. */
typedef struct {
  V3 v0, v1, v2, n;
  int16_t lo[3], hi[3];
} CTri;

typedef struct {
  CTri *tri;
  uint32_t ntri;
  /* A grid over X and Z: cell c lists triangles list[start[c]..start[c+1]). */
  float x0, z0, cell;
  int nx, nz;
  uint32_t *start, *list;
  uint32_t *stamp, now; /* one visit per triangle per query */
  V3 bmin, bmax;
} World;

typedef struct {
  Mesh mesh;
  World world;
  uint32_t ntris;
  uint32_t skipped; /* primitives that are not triangles */
  bool has_spawn;
  V3 spawn;
  float spawn_yaw;
} Room;

#define MAX_NODES  96
#define MAX_JOINTS 64

/* The joints the procedural walk moves, found by name. */
enum {
  RIG_HIPS,
  RIG_SPINE,
  RIG_HEAD,
  RIG_THIGH_L,
  RIG_THIGH_R,
  RIG_SHIN_L,
  RIG_SHIN_R,
  RIG_UARM_L,
  RIG_UARM_R,
  RIG_FARM_L,
  RIG_FARM_R,
  RIG_COUNT
};

typedef struct {
  int parent;
  V3 t, s; /* rest pose */
  Quat r;
  /* The model's X, Y and Z axes in the parent's frame at rest: procedural
   * rotations turn a joint about these, whatever the rig's own axes. */
  V3 ax[3];
  float unit; /* metres per unit of the parent's frame */
} Node;

enum { PATH_T, PATH_R, PATH_S };
enum { INTERP_LINEAR, INTERP_STEP, INTERP_CUBIC };

typedef struct {
  int node, path, interp;
  uint32_t nkeys;
  float *time, *val; /* 3 or 4 floats a key, three times that for cubic */
} Chan;

typedef struct {
  char name[32];
  float length;
  Chan *chan;
  uint32_t nchan;
} Clip;

typedef struct {
  /* Every node of the player's file, `order` listing parents first. */
  Node node[MAX_NODES];
  int nnodes;
  int order[MAX_NODES];
  int mesh_node;
  int njoints; /* 0: a mesh without a skin, moved as one piece */
  int joint[MAX_JOINTS];
  Aff ibm[MAX_JOINTS];
  int rig[RIG_COUNT]; /* node, or -1 */
  int nrig;
  Clip *clip;
  int nclip;
  int clip_walk, clip_idle; /* -1 for none */
  /* The file's space to the player's: feet on the origin, in metres. */
  Aff fix;
  float height, radius;
  /* Skinning input, per vertex of `mesh`. */
  int32_t *pos; /* x, y, z in 1/4096 m, the file's space */
  int16_t *nrm; /* x, y, z in 1/16384 */
  uint8_t *jw;  /* joints 0-3, then weights 0-3 adding up to 255 */
  uint16_t *rgba; /* base colour, linear, and alpha, 0..4095 each */
  Mesh mesh;
  uint32_t ntris;
} Player;

typedef struct {
  float t;     /* seconds */
  float phase; /* of the walk cycle, radians */
  float walk;  /* 0 standing .. 1 moving */
  float speed; /* m/s over the ground */
  float air;   /* 0 on the ground .. 1 in the air */
} AnimState;

/* Each reads a .glb held in memory. False with a reason in `err`. */
bool room_load(Room *r, const void *glb, size_t size, char *err, size_t errn);
bool player_load(Player *p, const void *glb, size_t size, char *err,
                 size_t errn);

void anim_pose(const Player *p, const AnimState *a, Aff *joint_mtx);
/* Skins into p->mesh. The lights are unit directions in the player's own
 * space, towards the light; `alpha` scales the model's own. */
void skin_apply(Player *p, const Aff *joint_mtx, V3 key, V3 fill, float alpha);
const char *anim_describe(const Player *p);

bool world_build(World *w);
/* The highest walkable surface under `feet` (sampled over `radius`) no
 * higher than feet.y + up and no lower than feet.y - down, or -1e30. */
float world_ground(World *w, V3 feet, float radius, float up, float down);
/* `feet` moved out of walls, keeping it upright; bodies are capsules of
 * `radius` from `step` above the feet to `height`. */
V3 world_push(World *w, V3 feet, float radius, float height, float step);
bool world_free(World *w, V3 feet, float radius, float height, float step);
/* The fraction of from->to before the first triangle, 1 when clear. */
float world_ray(World *w, V3 from, V3 to);

/* Lighting shared by the room bake and the skinning: linear colour times
 * this, then to sRGB. */
#define LIGHT_AMBIENT 0.38f
#define LIGHT_KEY     0.62f
#define LIGHT_FILL    0.22f
extern const V3 g_light_key, g_light_fill;
/* Linear 0..1 to sRGB 0..1. */
float srgb(float linear);

#endif
