// Exercises the resident system menu while Down and Select remain app input.
#include <aurora_app.h>

int main(void) {
  int down_presses = 0;
  int left_presses = 0;
  int select_presses = 0;

  while (app_loop()) {
    uint32_t keys = hid_keys_down();
    if (keys & KEY_DOWN)
      down_presses++;
    if (keys & KEY_LEFT)
      left_presses++;
    if (keys & KEY_SELECT)
      select_presses++;

    gfx_gradient(SCREEN_TOP, 0, 0, SCREEN_TOP_WIDTH, SCREEN_HEIGHT,
                 RGB(18, 24, 42), RGB(8, 9, 16));
    gfx_print(SCREEN_TOP, 18, 18, FONT_TITLE, COLOR_WHITE,
              "System menu test - C");
    gfx_print(SCREEN_TOP, 18, 58, FONT_REGULAR, COLOR_AURORA,
              "Press L + Down + Select");
    gfx_print(SCREEN_TOP, 18, 82, FONT_REGULAR, COLOR_WHITE,
              "The app also uses Down and Select.");
    gfx_print(SCREEN_TOP, 18, 130, FONT_REGULAR, COLOR_GRAY,
              "Down presses: %d", down_presses);
    gfx_print(SCREEN_TOP, 18, 154, FONT_REGULAR, COLOR_GRAY,
              "Left presses: %d", left_presses);
    gfx_print(SCREEN_TOP, 18, 178, FONT_REGULAR, COLOR_GRAY,
              "Select presses: %d", select_presses);
    gfx_clear(SCREEN_BOTTOM, RGB(20, 22, 30));
    gfx_print(SCREEN_BOTTOM, 20, 42, FONT_BOLD, COLOR_AURORA,
              "AuroraOS C SDK");
    gfx_print(SCREEN_BOTTOM, 20, 76, FONT_REGULAR, COLOR_WHITE,
              "L + Down + Select opens the system menu.");
    gfx_print(SCREEN_BOTTOM, 20, 102, FONT_REGULAR, COLOR_GRAY,
              "Cancel returns here; Return to Home exits.");
    gfx_print(SCREEN_BOTTOM, 20, 150, FONT_SMALL, COLOR_GRAY,
              "HOME or B returns to Aurora Home.");
    if (keys & KEY_B)
      break;
  }
  return 0;
}
