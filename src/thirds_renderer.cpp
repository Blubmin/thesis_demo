#include "thirds_renderer.h"

namespace {
boost::filesystem::path GetShaderPath() {
  return boost::filesystem::path(__FILE__).parent_path() / "resources" /
         "shaders";
}
}  // namespace

std::shared_ptr<pxl::Program> ThirdsRenderer::prog_(nullptr);

GLuint ThirdsRenderer::vao_(0);

void ThirdsRenderer::Init() {
  prog_ = std::make_shared<pxl::Program>(GetShaderPath() / "thirds.vert",
                                         GetShaderPath() / "thirds.frag");

  glGenVertexArrays(1, &vao_);
  glBindVertexArray(vao_);

  GLuint buffer;
  glGenBuffers(1, &buffer);

  std::vector<float> pts = {0,   .33, 1,   .33, 0,   .66, 1,   .66,
                            .33, 0,   .33, 1,   .66, 0,   .66, 1};

  glBindBuffer(GL_ARRAY_BUFFER, buffer);
  glBufferData(GL_ARRAY_BUFFER, pts.size() * sizeof(pts[0]), pts.data(),
               GL_STATIC_DRAW);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ThirdsRenderer::RenderLines() {
  if (prog_ == nullptr) {
    Init();
  }

  prog_->Bind();
  glBindVertexArray(vao_);
  glLineWidth(2);
  glDrawArrays(GL_LINES, 0, 8);
  glLineWidth(1);
  glBindVertexArray(0);
  prog_->UnBind();
}
