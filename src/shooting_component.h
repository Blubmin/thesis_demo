#pragma once

#include <pixel_engine/component.h>

#include <memory>

#include <pixel_engine/scene.h>

class Beachball : public pxl::MeshEntity {
 public:
  Beachball();

  bool spent;
};

class ShootingComponent : public pxl::Component {
 public:
  ShootingComponent();
  ShootingComponent(std::shared_ptr<pxl::Scene> scene);

  void Update(float time_elapsed) override;

 private:
  std::shared_ptr<pxl::Entity> CreateBall();

  std::shared_ptr<pxl::Scene> scene_;
};