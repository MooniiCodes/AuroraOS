/* Reads SD:\Aurora\assets.pak and RLE-decodes every entry into a fixed arena.
 * All-or-nothing: any inconsistency leaves assets_ok() false. Layout in
 * docs/assets.md. */

#include "assets.h"
#include "aurora.h"
#include "ff.h"

#define PAK_MAGIC   0x4B415041u /* "APAK" */
#define PAK_VERSION 1u
#define PAK_PATH    "0:/Aurora/assets.pak"
#define ASSETS_LOAD_ATTEMPTS 3u
#define ASSETS_RETRY_DELAY   60000u

typedef struct {
  uint32_t magic, version, count, data_off, arena, reserved[3];
} PakHead;

typedef struct {
  uint32_t id;
  uint16_t type, w, h, pad;
  uint32_t off, csize, usize;
} PakEntry;

/* The pack is written by a Python script packing raw little-endian fields, so
 * any padding the compiler adds here would silently misread every entry. */
typedef char assets_layout_check[
    (sizeof(PakHead) == 32 && sizeof(PakEntry) == 24 &&
     sizeof(AssetFontHead) == 16 && sizeof(AssetGlyph) == 10 &&
     ASSET_ARENA_NEEDED <= ASSETS_ARENA_MAX)
        ? 1
        : -1];

static Asset g_assets[ASSET_COUNT];
static int g_ok;
static const char *g_err = "not loaded";

static PakEntry g_table[ASSET_COUNT];
static uint8_t g_scratch[ASSET_MAX_CSIZE];

/* 0x01..0x7F: run of n copies of the next byte. 0x81..0xFF: n literals. */
static int rle_decode(const uint8_t *src, uint32_t csize, uint8_t *dst,
                      uint32_t usize) {
  const uint8_t *send = src + csize;
  uint8_t *out = dst, *oend = dst + usize;
  while (src < send) {
    uint8_t c = *src++;
    if (c & 0x80) {
      uint32_t n = c & 0x7Fu;
      if (!n || n > (uint32_t)(oend - out) || n > (uint32_t)(send - src))
        return 0;
      while (n--)
        *out++ = *src++;
    } else {
      uint32_t n = c;
      uint8_t v;
      if (!n || n > (uint32_t)(oend - out) || src >= send)
        return 0;
      v = *src++;
      while (n--)
        *out++ = v;
    }
  }
  return out == oend;
}

int assets_ok(void) { return g_ok; }
const char *assets_error(void) { return g_err; }

const Asset *asset_get(uint32_t id) {
  if (!g_ok || id >= ASSET_COUNT || !g_assets[id].data)
    return 0;
  return &g_assets[id];
}

int asset_font(uint32_t id, Font *out) {
  const Asset *a = asset_get(id);
  const AssetFontHead *h;
  if (!a || !out || a->type != ASSET_TYPE_FONT)
    return 0;
  h = (const AssetFontHead *)a->data;
  out->head = h;
  out->glyphs = (const AssetGlyph *)(a->data + sizeof(AssetFontHead));
  out->atlas = a->data + sizeof(AssetFontHead) + h->count * sizeof(AssetGlyph);
  return 1;
}

static int fail(const char *why) {
  g_err = why;
  g_ok = 0;
  return 0;
}

int assets_load(void) {
  static FATFS fs;
  int mounted, ok = 0;

  if (g_ok)
    return 1;
  for (unsigned attempt = 0; attempt < ASSETS_LOAD_ATTEMPTS; attempt++) {
    mounted = f_mount(&fs, "", 1) == FR_OK;
    if (mounted)
      ok = assets_read();
    else
      fail("no SD card");
    f_mount(NULL, "", 0);
    if (ok)
      break;
    if (attempt + 1u < ASSETS_LOAD_ATTEMPTS)
      delay(ASSETS_RETRY_DELAY);
  }
  return ok;
}

int assets_read(void) {
  static FIL f;
  PakHead head;
  UINT br;
  uint32_t i, used = 0;
  uint8_t seen[ASSET_COUNT] = {0};
  uint32_t expected_data_off;
  FSIZE_t file_size;
  uint8_t *arena = (uint8_t *)ASSETS_ARENA_ADDR;

  if (g_ok)
    return 1;
  if (f_open(&f, PAK_PATH, FA_READ) != FR_OK)
    return fail("assets.pak not found");

  if (f_read(&f, &head, sizeof(head), &br) != FR_OK || br != sizeof(head)) {
    f_close(&f);
    return fail("pack header unreadable");
  }
  if (head.magic != PAK_MAGIC || head.version != PAK_VERSION) {
    f_close(&f);
    return fail("pack magic/version mismatch");
  }
  if (head.count != ASSET_COUNT) {
    f_close(&f);
    return fail("pack does not match this build");
  }
  if (head.arena > ASSETS_ARENA_MAX) {
    f_close(&f);
    return fail("pack too large for the arena");
  }

  expected_data_off = sizeof(PakHead) + sizeof(PakEntry) * head.count;
  file_size = f_size(&f);
  if (head.data_off != expected_data_off || head.data_off > file_size) {
    f_close(&f);
    return fail("pack index offset invalid");
  }

  if (f_read(&f, g_table, sizeof(PakEntry) * head.count, &br) != FR_OK ||
      br != sizeof(PakEntry) * head.count) {
    f_close(&f);
    return fail("pack index unreadable");
  }

  for (i = 0; i < head.count; i++) {
    const PakEntry *e = &g_table[i];
    if (e->id >= ASSET_COUNT || seen[e->id] || !e->usize || !e->csize ||
        e->csize > ASSET_MAX_CSIZE || used > ASSETS_ARENA_MAX ||
        e->usize > ASSETS_ARENA_MAX - used || e->off < head.data_off ||
        e->off > file_size || e->csize > file_size - e->off) {
      f_close(&f);
      return fail("pack entry out of range");
    }
    seen[e->id] = 1;
    if (f_lseek(&f, e->off) != FR_OK ||
        f_read(&f, g_scratch, e->csize, &br) != FR_OK || br != e->csize) {
      f_close(&f);
      return fail("pack payload unreadable");
    }
    if (!rle_decode(g_scratch, e->csize, arena + used, e->usize)) {
      f_close(&f);
      return fail("pack payload malformed");
    }
    if (e->type == ASSET_TYPE_ALPHA8 || e->type == ASSET_TYPE_RGBA) {
      uint64_t expected = (uint64_t)e->w * e->h;
      if (e->type == ASSET_TYPE_RGBA)
        expected *= 4u;
      if (!e->w || !e->h || expected != e->usize) {
        f_close(&f);
        return fail("pack image size invalid");
      }
    } else if (e->type == ASSET_TYPE_FONT) {
      const AssetFontHead *font = (const AssetFontHead *)(arena + used);
      uint64_t expected = sizeof(AssetFontHead) +
                          (uint64_t)font->count * sizeof(AssetGlyph) +
                          (uint64_t)font->atlas_w * font->atlas_h;
      if (!font->count || !font->atlas_w || !font->atlas_h ||
          expected != e->usize) {
        f_close(&f);
        return fail("pack font size invalid");
      }
    } else {
      f_close(&f);
      return fail("pack asset type invalid");
    }
    g_assets[e->id].type = e->type;
    g_assets[e->id].w = e->w;
    g_assets[e->id].h = e->h;
    g_assets[e->id].data = arena + used;
    used += (e->usize + 3u) & ~3u;
  }

  if (used != head.arena) {
    f_close(&f);
    return fail("pack arena size mismatch");
  }
  if (g_assets[ASSET_FONT_SMALL].type != ASSET_TYPE_FONT ||
      g_assets[ASSET_FONT_UI].type != ASSET_TYPE_FONT ||
      g_assets[ASSET_FONT_BOLD].type != ASSET_TYPE_FONT ||
      g_assets[ASSET_FONT_TITLE].type != ASSET_TYPE_FONT) {
    f_close(&f);
    return fail("required font missing");
  }

  f_close(&f);
  g_err = "";
  g_ok = 1;
  return 1;
}

/* Returns 0 when the asset is missing, so callers can fall back. Lives here
 * rather than in screen.c because Auric apps link screen.c without the loader. */
int draw_asset_boxed(volatile u8 *fb, int bx, int by, int bw, int bh,
                     int screen_height, uint32_t id, Color tint) {
  const Asset *a = asset_get(id);
  if (!a)
    return 0;
  draw_asset(fb, bx + (bw - (int)a->w) / 2, by + (bh - (int)a->h) / 2,
             screen_height, a, tint);
  return 1;
}
