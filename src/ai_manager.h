#pragma once

#include <memory>

#include <pixel_engine/entity.h>
#include <pixel_engine/scene.h>

class Enemy : public pxl::MeshEntity {
 public:
   Enemy();

   float speed;
   float weight;
   bool disable;
};

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
  std::shared_ptr<Enemy> CreateEnemy(const Eigen::Vector3f& pos = Eigen::Vector3f::Zero());
  Eigen::Vector3f Seek(const std::shared_ptr<pxl::Entity> unit,
                       const Eigen::Vector3f to_seek) const;
  Eigen::Vector3f ComputeSeparation(
      const std::shared_ptr<pxl::Entity> unit) const;
  Eigen::Vector3f ComputeClustering(
      const std::shared_ptr<pxl::Entity> unit) const;

  std::shared_ptr<pxl::Scene> scene_;
  std::shared_ptr<pxl::Entity> player_;

 public:
  void Add();
  void Remove();
  std::vector<std::weak_ptr<Enemy>> red_team_;
  bool halt;
};