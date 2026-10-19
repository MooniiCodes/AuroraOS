# 3D PlayTest

A third-person 3D test for AuroraOS, written in C with the native app SDK.
The player model walks around a room model, both drawn by the PICA200 GPU,
with the 3D slider giving stereoscopic 3D. The player is skinned on its
bones and animates as it moves.

| | |
|--|--|
| Code | `sdk/examples/3D-PlayTest` |
| Build | `sdk/build/apps/3D-PlayTest/3D-PlayTest.bin` (about 400 KB) |
| Install | copy `3D-PlayTest.bin` to `SD:/Aurora/Apps/C/` |
| Data folder | `SD:/Aurora/Apps/C/3D-PlayTest` (made by the OS at launch) |

## Controls

| Input | |
|-------|--|
| Circle Pad | move, relative to the camera; the further it is pushed, the faster |
| B (held) | run |
| A | jump |
| C-stick (New 3DS) | turn the camera around the player, and up and down |
| D-pad | the camera too: the only camera control on an Old 3DS, and a second one on a New 3DS |
| L | swing the camera behind the player |
| Y | camera distance: near, middle, far |
| X | invert the camera's up and down |
| 3D slider | stereoscopic 3D (3DS, 3DS XL, New 3DS, New 3DS XL) |
| SELECT | back to the start point |
| START or HOME | back to the Home Menu |

The bottom screen shows the frame rate and timings, which camera control the
console uses, the C-stick's raw report, the 3D slider, the models' sizes, the
animation in use and the player's position.

## Building

```
py sdk/examples/3D-PlayTest/build.py        # add -v to see each command
```

It needs devkitARM (as for any SDK app), devkitPro's `picasso` (the 3DS
tools, `devkitPro/tools/bin`) and Windows Python (`py`). `build.py`:

1. assembles `playtest.v.pica` with picasso and turns it into
   `gen/playtest_shbin.h` with the OS's `tools/shbin2c.py`;
2. writes `gen/assets.s`, which brings `assets/player.glb` and
   `assets/test_room.glb` into the app with `.incbin`;
3. runs `sdk/aurcc.py` on this folder plus the OS's `src/os/P3d9.c` and
   `src/os/Stereo9.c`, with `include/` and `gen/` on the include path.

Everything generated goes to `sdk/build/apps/3D-PlayTest` (git-ignored).

## Libraries used, and why

### cgltf 1.15 (ported)

`lib/cgltf/cgltf.h`, by Johannes Kuhlmann, MIT licence (`lib/cgltf/LICENSE`),
from <https://github.com/jkuhlmann/cgltf> tag v1.15.

**Why.** Both models are glTF 2.0 binaries (`.glb`). The player needs more
than geometry: its node tree (13 nodes), a skin (11 joints with inverse bind
matrices) and per-vertex joints and weights. Animation clips should work too,
for models that have them. cgltf reads all of that: the GLB chunks, the JSON,
accessors of every component type (normalised integers, sparse accessors),
node transforms, skins, animations and materials. It also validates
the file, so a broken model gives a message rather than a crash. It is one C99
header that needs only `malloc` and the string functions, which newlib has.

**The port.** `cgltf.h` is upstream's file unchanged (same SHA-1 as the tag).
The port is in `cgltf_port.c`:

* `CGLTF_ATOF` is replaced by `cgltf_port_atof`, a float-precision decimal
  parser. Blender writes numbers like `0.15000000596046448`, and newlib's
  exact `atof` converts each one with double-precision big-number arithmetic,
  in software, since the ARM9 has no FPU. cgltf stores floats, so nine digits
  and one rounding are all it can use. On the PC it matched `strtod` to
  within 3e-7 (relative) for 300,000 numbers of every shape.
* The `-Wall -Wextra` warnings cgltf raises (sign comparisons, unused
  parameters, case fall-through) are silenced around the include only.
* cgltf's file loading is not used: the `.glb` is in memory, either built
  into the app or read from the data folder with `fs_read_file()`. Its
  memory comes from the SDK's 16 MB `malloc()` heap.

### AuroraOS's own code

The SDK does not expose 3D yet, so the app links the OS's own 3D code,
which the OS's 3D Model screen already runs on hardware:

* **`src/os/P3d9.c`**: builds PICA200 command lists (register writes, the
  vertex shader upload, vertex buffers, DrawElements, the combiner and the
  depth and blend state). It follows citro3d's register order.
* **`src/os/Gpu9.c`** (already in the SDK runtime): `gpu_p3d()` has the
  ARM11 core clear the render target, run the list and copy the frame out
  in the framebuffer layout. Also `gpu_present_st_async()` (both eyes and
  the bottom screen at one blank) and `gpu_stereo()`.
* **`src/os/Stereo9.c`**: the console model, the 3D slider (MCU 0x08), and
  switching the top screen and its parallax barrier into and out of 3D.
* **`src/i2c.c`** (in the runtime): `I2C_init()` sets every I2C bus to wait
  while a device holds SCL low, which the C-stick needs.
* **The shader**: `playtest.v.pica` is the 3D Model screen's
  `src/os/model.v.pica` again, and picasso's output is byte for byte the same.
  The vertex layout is also that screen's (`GlbVertex`). So the parts of
  the GPU path the OS has already shown working on a console are reused as
  they are.
* **The SDK runtime and newlib**: `malloc`/`memalign`, stdio, and libm
  (`sinf`, `cosf`, `atan2f`, `sqrtf`, `powf`, `cbrtf`, `floorf`).

### Tools

* **picasso** (devkitPro): assembles the vertex shader.
* **tools/shbin2c.py** (AuroraOS): turns picasso's `.shbin` into the
  `P3dShader` header P3d9.c uploads.
* **sdk/aurcc.py** (AuroraOS): compiles and packs the app as `AURC`.

### Considered and not used

* **citro3d / libctru**: these work only under Horizon (they go through
  the GSP service and its linear heap); AuroraOS is bare metal. P3d9.c
  stands in for citro3d.
* **The OS's `Glb.c`** (the 3D Model screen's loader): it puts its buffers
  at fixed addresses, including `0x24000000`, where an app's heap lives. It
  also bakes every node into one static mesh, with no skins or animation.
* **A maths library** (cglm, HandmadeMath, linmath): on an ARM9 without an
  FPU the per-vertex work has to be fixed point, which none of them do. The
  few float matrices a frame need about 200 lines (`vmath.c`).
* **A physics library** (Bullet, ODE, ...): far too large and float-heavy
  for a 134 MHz CPU with no FPU. A grid of triangles with sphere and ray
  tests (`collide.c`) is all the playtest needs.
* **An image library**: the models use colours, not textures.

### Algorithms from the literature

Closest point on a triangle (Ericson, *Real-Time Collision Detection*,
5.1.5); ray against triangle (Moller and Trumbore); matrix to quaternion
(Shepperd); cubic-spline keys (glTF 2.0 specification, appendix C); the sRGB
curve (IEC 61966-2-1). The C-stick's protocol comes from GBATEK ("3DS I2C
New3DS C-Stick and ZL/ZR Buttons") and 3dbrew ("I2C Registers", device 17).

## How it works

**Loading** (`scene.c`, `player.c`). The room is baked once into world
space. Each vertex gets its colour with the light already in it: ambient,
a key light and a fill light, a little darker near the floor, then converted
from glTF's linear colour to sRGB so it looks as it does in Blender. Its
triangles also become the collision mesh. The player keeps every node as
its skeleton, with parents first. Its vertices are stored in fixed point
(1/4096 m) with up to four joints each, weights adding up to 255. A plain
mesh without a skin follows its own node the same way, through one rigid
joint slot. The player is moved so its feet are on the origin. It is scaled
to 1.8 m only if its height is clearly not in metres.

**Animation** (`anim.c`). If the player file has a clip named like *walk*
or *run*, it plays in step with the distance walked, blended with an *idle*
clip (or the rest pose) by speed. The player file supplied has bones but no
clips, so a procedural cycle drives it instead: thighs and arms swing in
opposition, knees fold during the swing, the hips bob twice a stride and
twist against the spine, and the body leans with speed. Standing still, it
breathes. In the air, the legs tuck and the arms rise. Bones are found by name
(`Hips`, `Spine`, `Head`, `Thigh.L`, `Shin.L`, `UpperArm.L`, `Forearm.L`
and their `.R` twins, or Mixamo's `LeftUpLeg`, `LeftLeg`, `LeftArm`,
`LeftForeArm` and so on). They turn about the model's own axes, so the same
angles suit any rig's bone orientations.

**Skinning** (`anim.c`). It runs on the ARM9 every frame in fixed point:
joint matrices in 1/2^20, normals in 1/16384, 64-bit products, then each
vertex is lit for the player's facing. In soft float the same work would
take several milliseconds a frame.

**Drawing** (`render.c`). Each eye is one command list:

1. the sky, drawn behind everything;
2. the room's opaque parts;
3. a soft blob shadow under the player;
4. the player;
5. anything translucent, far to near.

The ARM11 core clears the GPU's render target, runs the list and copies the
frame into the SDK's top-screen backbuffer. In 2D the SDK's `app_loop()`
presents it. With the 3D slider up, both eyes are drawn (the player sits on
the screen surface) and presented together with the bottom screen.

**Movement and collision** (`collide.c`, `main.c`). The player is a
capsule of spheres above a 32 cm step height. A grid over the floor plan
gives each test only nearby triangles, and integer bounds reject most of
those before any float maths. Walls push the player sideways, and the
velocity keeps whatever runs along the wall. Floors are found by
sampling heights under the feet: the player steps up and down ledges up to
32 cm, lands on surfaces when falling, and stops at an edge with nothing
under it (jumping off still falls, and falling far brings the player back
to the start). Small, detailed parts (over 192 triangles and under 1.2 m,
like Suzanne, the sphere and the torus) collide as their bounding box,
which keeps a jump beside them from costing tens of milliseconds.

**Camera.** The camera orbits a point at chest height. A ray from there
pulls the camera in front of any wall behind it at once, and it eases back
out. When it gets close, the player fades, then disappears, so the room
stays in view.

**The C-stick** (`cstick.c`). On a New 3DS it is a small microcontroller
on the third I2C bus (`0x10148000`), at device address 0x54. It has no
registers: a read returns its report from byte 0 (status, ZL/ZR, X and Y,
then finer X and Y at bytes 5 and 6). The app reads seven bytes a frame,
since reading past byte 0x1A can hang the chip. Every wait is bounded, and
after 120 failed reads in a row the app stops trying (the D-pad still
turns the camera).

## Using other models

Put `player.glb` and/or `room.glb` (or `test_room.glb`) in
`SD:/Aurora/Apps/C/3D-PlayTest`: they replace the built-in ones, no rebuild
needed. The bottom screen says which are in use.

* glTF 2.0 binary, triangles. Colours come from base colour factors,
  vertex colours and emission; textures are not read.
* The player faces +Z, which is what Blender's glTF exporter writes for a
  model facing Blender's -Y. Up is +Y and units are metres.
* For the procedural walk, name the bones as above, or include clips named
  *walk* (or *run*) and *idle*.
* In the room, an empty named `Spawn` (or `PlayerStart`) sets where the
  player starts and which way it faces. A node whose name contains
  `NoCollide` is drawn but not solid.
* Limits: the player up to 96 nodes, 64 joints and 65,535 vertices; each
  room primitive up to 65,535 vertices.

## Files

| File | |
|------|--|
| `main.c` | start-up, loading, the frame loop, movement, the camera, the bottom screen |
| `scene.c`, `loader.h` | the room loader, and cgltf helpers shared with `player.c` |
| `player.c` | the player loader: skeleton, joint slots, skin data, clips, bone names |
| `anim.c` | poses (clips or procedural), joint matrices, fixed-point skinning |
| `collide.c` | the collision grid, ground heights, capsule push-out, rays |
| `render.c`, `render.h` | PICA200 command lists, stereo, presents |
| `cstick.c`, `cstick.h` | the New 3DS C-stick over I2C |
| `vmath.c`, `vmath.h` | float vectors, quaternions, affine matrices |
| `game.h` | shared types |
| `cgltf_port.c`, `lib/cgltf/` | cgltf and its port |
| `playtest.v.pica` | the vertex shader |
| `assets/` | the player and room models built into the app |
| `3D-PlayTest.icon` | the Home Menu icon |
| `build.py` | the build |

## Status

Built with devkitARM; not yet run on a console. It was checked on a PC with
a harness built from the app's own sources: `main.c`, the loaders, the
animation, the collision, and `render.c` with the OS's real `P3d9.c` and
`Stereo9.c`. Its stand-in for `gpu_p3d()` decodes the command list the app
builds (register writes, uniform uploads, immediate-mode vertices,
DrawElements from the vertex buffers) and rasterises it. Scripted runs
checked:

* the frames (room, skinned player, walk cycle, jump pose, shadow, stereo
  pair);
* walking into furniture and walls, jumping onto the table and off it;
* stopping at the open front of the floor;
* the camera against walls, the C-stick and D-pad cameras, zoom,
  recentring;
* switching into and out of 3D.

What only a console can show:

* the frame rate (the bottom screen shows frame, update and GPU times);
* the C-stick: the bottom screen shows its raw bytes and reads, so if the
  camera does not turn with it, report those;
* stereo depth and the barrier on an Old 3DS.
