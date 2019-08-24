#include "planar_movement_component.h"

#include <glfw/glfw3.h>
#include <imgui/imgui.h>
#include <pixel_engine/entity.h>
#include <Eigen/Core>

namespace {
const float kSpeed = 5;
}

PlanarMovementComponent::PlanarMovementComponent() {}

void PlanarMovementComponent::Update(float time_elapsed) {
  Eigen::Vector3f movement_vec(0, 0, 0);
  if (ImGui::IsKeyDown(GLFW_KEY_UP)) {
    movement_vec.z() += 1;
  }
  if (ImGui::IsKeyDown(GLFW_KEY_DOWN)) {
    movement_vec.z() -= 1;
  }
  if (ImGui::IsKeyDown(GLFW_KEY_LEFT)) {
    movement_vec.x() -= 1;
  }
  if (ImGui::IsKeyDown(GLFW_KEY_RIGHT)) {
    movement_vec.x() += 1;
  }
  owner.lock()->position += movement_vec * kSpeed * time_elapsed;
}