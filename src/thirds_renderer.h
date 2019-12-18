#pragma once

#include <pixel_engine/program.h>

class ThirdsRenderer {
 public:
  static void RenderLines();

 private:
  static void Init();

  static std::shared_ptr<pxl::Program> prog_;
  static GLuint vao_;
};