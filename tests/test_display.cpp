#include "camera.h"
#include "pch.hpp"
#include <assert.h>

int main() {
  int SCR_W = 1000;
  int SCR_H = 400;
  int WRLD_W = 400;
  double SCALE = (double)SCR_W / WRLD_W;

  Camera c = Camera(NULL, WRLD_W, Vector2d(SCR_W, SCR_H));
  Vector2d world_pos(0.0, 0.0);
  Vector2d screen_pos = c.world_to_screen(world_pos);
  assert(screen_pos == Vector2d(SCR_W / 2.0, SCR_H / 2.0));

  world_pos << WRLD_W / 4.0, 0.0;
  screen_pos = c.world_to_screen(world_pos);
  assert(screen_pos == Vector2d(3 / 4.0 * SCR_W, SCR_H / 2.0));

  world_pos << 0, 0;
  c.set_center(Vector2d(WRLD_W / 4.0, 0));
  screen_pos = c.world_to_screen(world_pos);
  assert(screen_pos == Vector2d(SCR_W / 4.0, SCR_H / 2.0));

  c.set_center(Vector2d(0, 0));
  MatrixX2d world_pos_list(3, 2);
  world_pos_list << 0, 0, 100, 0, 0, 20;
  MatrixX2d screen_pos_list = c.world_to_screen(world_pos_list);
  MatrixX2d expected(3, 2);
  expected << SCR_W / 2.0, SCR_H / 2.0, SCR_W / 2.0 + 100 / SCALE, SCR_H / 2.0,
      SCR_W / 2.0, SCR_H / 2.0 - 20 / SCALE;

  std::cout << "Success!" << std::endl;
}
