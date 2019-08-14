#include "aesthetic_camera_component.h"

#include <glog/logging.h>
#include <ceres/ceres.h>

AestheticCameraComponent::AestheticCameraComponent() {}

void AestheticCameraComponent::Update(float time_elapsed) {
  LOG(INFO) << "Updating the camera";
}