#pragma once
#include <pixel_engine/component.h>

#include <functional>
#include <vector>

#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

#include "ai_manager.h"

class AestheticCameraComponent : public pxl::Component {
 public:
  AestheticCameraComponent();
  void Update(float time_elapsed) override;

  std::function<void()> RunSolver();
  std::function<void()> UpdateTransform(std::array<double, 5> parameters,
                                        float time_elapsed);

  void SetTarget(std::weak_ptr<pxl::Entity> target);
  void SetPlayer(std::weak_ptr<pxl::Entity> player);
  void SetManager(std::weak_ptr<AiManager> manager);

  std::vector<bool> constant_residuals;
  std::vector<bool> constant_parameters;

 private:
  std::weak_ptr<pxl::Entity> player_;
  std::weak_ptr<pxl::Entity> target_;
  std::weak_ptr<AiManager> manager_;
  float time_elapsed_;
  boost::circular_buffer<float> average_framerate_;
  Eigen::Vector2f player_target;
  Eigen::Vector2f enemy_target;

  boost::optional<Eigen::Vector3f> prev_player_pos_;
};