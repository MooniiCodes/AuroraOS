#ifndef PLAYTEST_CSTICK_H
#define PLAYTEST_CSTICK_H

#include <stdbool.h>
#include <stdint.h>

#define CSTICK_RAW 7

typedef struct {
  bool ok;          /* the last read answered with a valid status */
  int x, y;         /* -127..127, +x right, +y up, 0 in the dead zone */
  bool zl, zr;
  uint8_t raw[CSTICK_RAW]; /* the last report, for the info screen */
  uint32_t reads, good;
  uint32_t misses; /* failed reads in a row */
} CStick;

/* After this many failed reads in a row the stick is left alone: a chip
 * holding the bus would otherwise cost each frame a timeout. */
#define CSTICK_GIVE_UP 120u

/* New 3DS family only. */
void cstick_read(CStick *c);

#endif
