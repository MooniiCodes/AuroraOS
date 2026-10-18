#ifndef AURORA_SYSTEM_MENU_H
#define AURORA_SYSTEM_MENU_H

#include <stddef.h>
#include <stdint.h>

/* Shared ARM9/ARM11 control block. Each core owns a separate cache line:
 * ARM9 writes app_active, while ARM11 writes action. */
#define AURORA_SYSTEM_MENU_ADDR 0x233A8000u
#define AURORA_SYSTEM_MENU_NONE 0u
#define AURORA_SYSTEM_MENU_HOME 1u

typedef struct __attribute__((aligned(32))) {
  volatile uint32_t app_active;
  uint32_t _app_pad[7];
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
