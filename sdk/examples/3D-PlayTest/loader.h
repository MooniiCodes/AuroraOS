/* cgltf helpers shared by scene.c (the room) and player.c. */
#ifndef PLAYTEST_LOADER_H
#define PLAYTEST_LOADER_H

#include "game.h"

#include "cgltf.h"

typedef struct {
  float rgba[4]; /* linear */
  float emissive[3];
  bool unlit, blend;
} MatInfo;

/* Writes the reason and returns false. */
bool load_fail(char *err, size_t n, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
/* Parses, loads the buffers and validates; cgltf_free() the result. */
bool load_parse(const void *glb, size_t size, cgltf_data **out, char *err,
                size_t errn);
const cgltf_accessor *load_attr(const cgltf_primitive *p,
                                cgltf_attribute_type t, int index);
bool load_is_triangles(const cgltf_primitive *p);
/* The primitive's triangles as vertex numbers, three a triangle (free() it),
 * or NULL when it has none. */
uint32_t *load_tris(const cgltf_primitive *p, uint32_t nverts, uint32_t *ntri);
/* Index or vertex count an upper bound on load_tris()'s output needs. */
uint32_t load_index_bound(const cgltf_primitive *p, uint32_t nverts);
void load_mat(const cgltf_material *m, MatInfo *o);
V3 load_read3(const cgltf_accessor *a, cgltf_size i, V3 dflt);
/* Lower case, letters and digits only: "Thigh.L" becomes "thighl". */
void load_squash(const char *in, char *out, size_t n);
bool load_in_scene(const cgltf_data *d, const cgltf_node *n);
bool load_mesh_alloc(Mesh *m, uint32_t nvtx, uint32_t nidx, uint32_t npart);

#endif
