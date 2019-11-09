#include "ai_manager.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <pixel_engine/collider_component.h>
#include <pixel_engine/mesh_loader.h>
#include <pixel_engine/ogl_mesh.h>
#include <pixel_engine/physics_component.h>
#include <boost/filesystem.hpp>

namespace {
const int32_t kTeamSize = 5;

boost::filesystem::path GetResourcePath() {
  return boost::filesystem::path(__FILE__).parent_path() / "resources";
}

boost::filesystem::path GetMeshPath(const std::string& mesh_file) {
  return GetResourcePath() / "meshes" / mesh_file;
}
}  // namespace

class BoidComponent : public pxl::Component {
 public:
  BoidComponent()
      : velocity(Eigen::Vector3f::Zero()),
        acceleration(Eigen::Vector3f::Zero()) {}

  void Update(float time_elapsed) override {}

  Eigen::Vector3f velocity;
  Eigen::Vector3f acceleration;
};

AiManager::AiManager(std::shared_ptr<pxl::Scene> scene,
                     std::shared_ptr<pxl::Entity> player)
    : separation_distance(1.2),
      separation(5),
      clustering_distance(20),
      clustering(1),
      max_speed(1),
      scene_(scene),
      player_(player),
      red_team_(kTeamSize) {
  for (int32_t i = 0; i < kTeamSize; ++i) {
    std::shared_ptr<pxl::MeshEntity> mesh =
        pxl::MeshLoader::LoadMeshEntity<pxl::OglMesh>(
            GetMeshPath("army_man_standing_scaled.obj"));
    mesh->Bind();
    mesh->position = Eigen::Vector3f(-i, 0, -5);
    // mesh->mesh->materials[0]->diffuse = Eigen::Vector3f(1, i == 0 ? .15 : 0,
    // 0);
    mesh->AddComponent(std::make_shared<BoidComponent>());
    mesh->AddComponent(std::make_shared<pxl::PhysicsComponent>());
    mesh->AddComponent(std::make_shared<pxl::CapsuleCollider>(
        .35f, 1.8f, pxl::ColliderComponent::kDynamic));
    red_team_[i] = mesh;
    scene_->entities.push_back(red_team_[i]);
    if (i == 0) {
      red_leader_ = mesh;
    }
  }
}

Eigen::Vector3f AiManager::Seek(const std::shared_ptr<pxl::Entity> unit,
                                const Eigen::Vector3f target) const {
  auto boid = unit->GetComponent<BoidComponent>();

  Eigen::Vector3f desired = target - unit->position;
  desired.normalize();
  desired *= 1;
  Eigen::Vector3f steer = desired - boid->velocity;
  return steer.normalized();
}

Eigen::Vector3f AiManager::ComputeSeparation(
    const std::shared_ptr<pxl::Entity> unit) const {
  auto boid = unit->GetComponent<BoidComponent>();
  Eigen::Vector3f separation_vec = Eigen::Vector3f::Zero();
  int32_t count = 0;

  for (auto other_unit : red_team_) {
    if (unit == other_unit) {
      continue;
    }
    auto diff = other_unit->position - unit->position;
    auto dist = diff.norm();
    if (dist < separation_distance) {
      separation_vec -= diff.normalized() / dist;
      ++count;
    }
  }

  // Average
  separation_vec /= (count != 0 ? count : 1);

  if (separation_vec.norm() != 0) {
    separation_vec.normalize();
    separation_vec *= 1;
    separation_vec -= boid->velocity;
  }

  return separation_vec;
}

Eigen::Vector3f AiManager::ComputeClustering(
    const std::shared_ptr<pxl::Entity> unit) const {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  int32_t count = 0;

  for (auto other_unit : red_team_) {
    if (unit == other_unit) {
      continue;
    }
    auto diff = other_unit->position - unit->position;
    auto dist = diff.norm();
    if (dist < clustering_distance) {
      position += other_unit->position;
      ++count;
    }
  }

  if (count > 0) {
    return Seek(unit, position / count);
  }
  return Eigen::Vector3f::Zero();
}

void AiManager::Update(float time_elapsed) {
  for (auto unit : red_team_) {
    auto boid = unit->GetComponent<BoidComponent>();

    // Follow the leader
    if (unit != red_leader_) {
      boid->acceleration += Seek(unit, red_leader_->position);
    } else {
      boid->acceleration += Seek(unit, player_->position) * 10.0;
    }
    boid->acceleration += ComputeClustering(unit) * clustering;
    boid->acceleration += ComputeSeparation(unit) * separation;
    boid->velocity += boid->acceleration * time_elapsed;
    boid->velocity.y() = 0;
    unit->rotation.y() =
        atan(boid->velocity.x() / boid->velocity.z()) * 180 / M_PI;
    if (boid->velocity.z() < 0) {
      unit->rotation.y() += 180;
    }
    if (boid->velocity.norm() > max_speed) {
      boid->velocity = boid->velocity.normalized() * max_speed;
    }
  }

  for (auto unit : red_team_) {
    auto boid = unit->GetComponent<BoidComponent>();
    unit->position += boid->velocity * time_elapsed;
    boid->acceleration *= 0;
  }
}