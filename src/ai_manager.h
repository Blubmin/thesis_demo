#pragma once

#include <memory>

#include <pixel_engine/entity.h>
#include <pixel_engine/scene.h>

class AiManager {
 public:
  AiManager(std::shared_ptr<pxl::Scene> scene,
            std::shared_ptr<pxl::Entity> player);

  void Update(float time_elapsed);

  float separation_distance;
  float separation;
  float clustering_distance;
  float clustering;
  float max_speed;

 private:
  Eigen::Vector3f Seek(const std::shared_ptr<pxl::Entity> unit,
                       const Eigen::Vector3f to_seek) const;
  Eigen::Vector3f ComputeSeparation(
      const std::shared_ptr<pxl::Entity> unit) const;
  Eigen::Vector3f ComputeClustering(
      const std::shared_ptr<pxl::Entity> unit) const;

  std::shared_ptr<pxl::Scene> scene_;
  std::shared_ptr<pxl::Entity> player_;
  std::vector<std::shared_ptr<pxl::Entity>> red_team_;
  std::shared_ptr<pxl::Entity> red_leader_;
};