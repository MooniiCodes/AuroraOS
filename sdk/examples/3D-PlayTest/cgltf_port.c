/* cgltf 1.15 (lib/cgltf/cgltf.h, MIT licence, Johannes Kuhlmann), built for
 * AuroraOS's ARM9. The header is upstream's file unchanged; what the port
 * needs is configured here, before it is included. See info.md.
 *
 * newlib gives cgltf malloc (the SDK's 16 MB heap), atoi, atoll and the
 * string functions. Its numbers go through cgltf_port_atof below instead of
 * newlib's atof: Blender writes 16 and 17 digit decimals, which newlib's
 * exact strtod converts with double-precision big-number arithmetic, all in
 * software on an ARM9 without an FPU. cgltf stores floats, so nine digits
 * and one rounding are all it can use. */
#include <stdbool.h>
#include <stdint.h>

float cgltf_port_atof(const char *s);

#define CGLTF_ATOF(str) cgltf_port_atof(str)
#define CGLTF_IMPLEMENTATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#include "cgltf.h"
#pragma GCC diagnostic pop

static const float p10[39] = {
    1e0f,  1e1f,  1e2f,  1e3f,  1e4f,  1e5f,  1e6f,  1e7f,  1e8f,  1e9f,
    1e10f, 1e11f, 1e12f, 1e13f, 1e14f, 1e15f, 1e16f, 1e17f, 1e18f, 1e19f,
    1e20f, 1e21f, 1e22f, 1e23f, 1e24f, 1e25f, 1e26f, 1e27f, 1e28f, 1e29f,
    1e30f, 1e31f, 1e32f, 1e33f, 1e34f, 1e35f, 1e36f, 1e37f, 1e38f};

/* A JSON number: sign, digits, fraction, exponent. Digits past the ninth
 * significant one only move the decimal point. */
float cgltf_port_atof(const char *s) {
  uint32_t m = 0;
  int e = 0, digits = 0;
  bool neg = false;
  float v;

  if (*s == '-') {
    neg = true;
    s++;
  } else if (*s == '+') {
    s++;
  }
  for (; *s >= '0' && *s <= '9'; s++) {
    if (digits < 9) {
      m = m * 10u + (uint32_t)(*s - '0');
      if (m)
        digits++;
    } else {
      e++;
    }
  }
  if (*s == '.') {
    for (s++; *s >= '0' && *s <= '9'; s++) {
      if (digits < 9) {
        m = m * 10u + (uint32_t)(*s - '0');
        if (m)
          digits++;
        e--;
      }
    }
  }
  if (*s == 'e' || *s == 'E') {
    bool eneg = false;
    int x = 0;
    s++;
    if (*s == '-') {
      eneg = true;
      s++;
    } else if (*s == '+') {
      s++;
    }
    for (; *s >= '0' && *s <= '9'; s++)
      if (x < 1000)
        x = x * 10 + (*s - '0');
    e += eneg ? -x : x;
  }

  v = (float)m;
  if (!m) {
    v = 0.0f;
  } else if (e > 0) {
    v = e > 38 ? 3.4e38f : v * p10[e];
  } else if (e < 0) {
    if (e < -38) {
      v /= 1e38f;
      e += 38;
      v = e < -38 ? 0.0f : v / p10[-e];
    } else {
      v /= p10[-e];
    }
  }
  return neg ? -v : v;
}
