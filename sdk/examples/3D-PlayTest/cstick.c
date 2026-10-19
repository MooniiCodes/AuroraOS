/* The New 3DS C-stick and ZL/ZR: a microcontroller on the third I2C bus
 * (0x10148000), device address 0x54. It has no registers; a read returns
 * its report from byte 0 (GBATEK "3DS I2C New3DS C-Stick and ZL/ZR Buttons";
 * 3dbrew "I2C Registers", device 17):
 *
 *   0     status, 0x80..0x83 (0xFF when it is not answering properly)
 *   1     buttons: ZL 0x04, ZR 0x02
 *   2, 3  X and Y, signed, 0 at rest, +X right and +Y up
 *   5, 6  X and Y at a finer resolution, about -0x7F..0x7F
 *
 * Reading past byte 0x1A can hang the chip, so only seven bytes are read.
 * The chip holds SCL low while it gets ready, which the bus waits for since
 * the SDK's I2C_init() (I2C_CNTEX bit 1). Waits are bounded: an Old 3DS has
 * no such device, and a hang here would freeze the app. */
#include "cstick.h"

#include <string.h>

#define BUS_ADDR 0x10148000u
#define DEV_READ 0x55u /* 0x54, read */

#define CNT_STOP   0x01u
#define CNT_START  0x02u
#define CNT_ERROR  0x04u
#define CNT_ACK    0x10u
#define CNT_READ   0x20u
#define CNT_ENABLE 0x80u /* set to start a byte; clears when it is done */

#define WAIT_POLLS 200000u

#define DEAD_HI  10 /* of the fine bytes */
#define FULL_HI  96
#define DEAD_LO  3 /* of the plain bytes */
#define FULL_LO  24

typedef struct {
  volatile uint8_t data;
  volatile uint8_t cnt;
  volatile uint16_t cntex;
  volatile uint16_t scl;
} I2cBus;

static bool idle(I2cBus *b) {
  for (uint32_t i = 0; i < WAIT_POLLS; i++)
    if (!(b->cnt & CNT_ENABLE))
      return true;
  return false;
}

static bool read_report(uint8_t *out, int n) {
  I2cBus *b = (I2cBus *)BUS_ADDR;
  if (!idle(b))
    return false;
  b->data = DEV_READ;
  b->cnt = CNT_ENABLE | CNT_START;
  if (!idle(b))
    return false;
  if (!(b->cnt & CNT_ACK)) {
    b->cnt = CNT_ENABLE | CNT_ERROR | CNT_STOP;
    idle(b);
    return false;
  }
  /* Each byte but the last is acknowledged, asking for the next. */
  for (int i = 0; i < n; i++) {
    b->cnt = CNT_ENABLE | CNT_READ | (i + 1 < n ? CNT_ACK : CNT_STOP);
    if (!idle(b))
      return false;
    out[i] = b->data;
  }
  return true;
}

static int axis(int v, int dead, int full) {
  int a = v < 0 ? -v : v;
  if (a <= dead)
    return 0;
  a = (a - dead) * 127 / (full - dead);
  if (a > 127)
    a = 127;
  return v < 0 ? -a : a;
}

/* The finer bytes unless only the plain ones moved. */
static int pick(int fine, int plain) {
  int a = axis(fine, DEAD_HI, FULL_HI), b = axis(plain, DEAD_LO, FULL_LO);
  int ma = a < 0 ? -a : a, mb = b < 0 ? -b : b;
  return ma >= mb ? a : b;
}

void cstick_read(CStick *c) {
  uint8_t r[CSTICK_RAW];
  bool got = read_report(r, CSTICK_RAW);

  c->reads++;
  if (got)
    memcpy(c->raw, r, sizeof(r));
  c->ok = got && r[0] != 0xFF && (r[0] & 0xFCu) == 0x80u;
  if (!c->ok) {
    c->x = c->y = 0;
    c->zl = c->zr = false;
    c->misses++;
    return;
  }
  c->good++;
  c->misses = 0;
  c->x = pick((int8_t)r[5], (int8_t)r[2]);
  c->y = pick((int8_t)r[6], (int8_t)r[3]);
  c->zl = (r[1] & 0x04u) != 0;
  c->zr = (r[1] & 0x02u) != 0;
}
