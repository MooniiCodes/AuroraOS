#ifndef AURORA_SYSTEM_MENU_H
#define AURORA_SYSTEM_MENU_H

#include <stddef.h>
#include <stdint.h>

/* Shared ARM9/ARM11 control block, each field group in its own cache line:
 * the ARM9 writes app_active and app_caps, the ARM11 menu_open and action,
 * which the ARM9 only clears. */
#define AURORA_SYSTEM_MENU_ADDR 0x233A8000u
#define AURORA_SYSTEM_MENU_NONE 0u
#define AURORA_SYSTEM_MENU_HOME 1u

/* app_caps: set by an app's runtime at start. The core cannot take the ARM9
 * out of an app, so Return to Home works only in apps that say they act on
 * it; os_install_return clears the field for every launch. */
#define AURORA_SYSTEM_MENU_CAP_HOME 1u

typedef struct __attribute__((aligned(32))) {
  volatile uint32_t app_active;
  volatile uint32_t app_caps;
  uint32_t _app_pad[6];
  volatile uint32_t menu_open;
  uint32_t _open_pad[7];
  volatile uint32_t action;
  uint32_t _action_pad[7];
} AuroraSystemMenuShared;

_Static_assert(offsetof(AuroraSystemMenuShared, menu_open) == 32,
               "ARM11 menu visibility must have its own cache line");
_Static_assert(offsetof(AuroraSystemMenuShared, action) == 64,
               "ARM11 menu action must have its own cache line");

static inline volatile AuroraSystemMenuShared *aurora_system_menu_shared(void) {
  return (volatile AuroraSystemMenuShared *)AURORA_SYSTEM_MENU_ADDR;
}

#endif
