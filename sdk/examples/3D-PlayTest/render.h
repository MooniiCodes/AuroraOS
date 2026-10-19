/* Drawing with the PICA200 through the OS's P3D command lists, and the top
 * screen's stereoscopic 3D. Only render.c includes the OS headers. */
#ifndef PLAYTEST_RENDER_H
#define PLAYTEST_RENDER_H

#include "game.h"

typedef struct {
  /* The camera: its position and basis, `back` pointing from the scene
   * towards the viewer. */
  V3 eye, right, up, back;
  float tan_half; /* of the vertical field of view */
  Aff player;     /* the player's space to the world */
  float player_alpha; /* 1 opaque, below that faded, 0 not drawn */
  V3 shadow;      /* the blob shadow's centre on the ground */
  float shadow_r, shadow_a;
} View;

/* False without the GPU (the app was not started from the Home Menu). */
bool render_init(void);
/* One eye's frame into `dst`, a 400x240 frame in the framebuffer layout.
 * `eye` is the eye's sideways offset in metres (negative for the left, 0
 * for 2D) and `conv` the distance at which both eyes' pictures meet. */
bool render_eye(const Room *room, const Player *pl, const View *v, float eye,
                float conv, uint32_t dst);
/* Why the last frame failed. */
const char *render_error(void);
/* The SDK's top-screen backbuffer and the right eye's frame. */
uint32_t render_top_back(void);
uint32_t render_right(void);

bool render_stereo_capable(void);
/* The 3D slider, 0 (off) .. 256 (top). */
int render_slider(void);
bool render_stereo(bool on);
/* Both eyes, and the bottom screen when `bottom`, at the next blank. */
void render_present_stereo(bool bottom);
/* Until the GPU has finished whatever was posted last. */
void render_wait(void);

#endif
