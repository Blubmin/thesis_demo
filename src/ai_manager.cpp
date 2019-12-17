#include "ai_manager.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <pixel_engine/collider_component.h>
#include <pixel_engine/mesh_loader.h>
#include <pixel_engine/ogl_mesh.h>
#include <pixel_engine/physics_component.h>
#include <boost/filesystem.hpp>

#include "shooting_component.h"

namespace {
const int32_t kTeamSize = 5;

boost::filesystem::path GetResourcePath() {
  return boost::filesystem::path(__FILE__).parent_path() / "resources";
}

boost::filesystem::path GetMeshPath(const std::string& mesh_file) {
  return GetResourcePath() / "meshes" / mesh_file;
}

class DeathCollisionResponse : public pxl::CollisionResponse {
 public:
  void Respond(btManifoldPoint& pt, const btCollisionObject* owner,
               const btCollisionObject* other) override {
    auto collider = (pxl::ColliderComponent*)other->getUserPointer();
    auto bb = std::dynamic_pointer_cast<Beachball>(collider->GetOwner());
    if (bb != nullptr && !bb->spent) {
      GetOwner()->RemoveFromScene();
      bb->spent = true;
    }
  }
};
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

Enemy::Enemy()
    : pxl::MeshEntity(pxl::MeshLoader::LoadMesh<pxl::OglMesh>(
          GetMeshPath("army_man_standing_scaled.obj"))),
      speed(2),
      weight(1),
      disable(false) {}

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
    auto enemy = CreateEnemy(Eigen::Vector3f(-i, 0, 5));
    red_team_[i] = enemy;
    scene_->AddEntity(enemy);
    if (i == 0) {
      // red_leader_ = mesh;
    }
  }
}

std::shared_ptr<Enemy> AiManager::CreateEnemy(const Eigen::Vector3f& pos) {
  std::shared_ptr<Enemy> enemy = std::make_shared<Enemy>();
  enemy->position = pos;
  enemy->Bind();
  // mesh->mesh->materials[0]->diffuse = Eigen::Vector3f(1, i == 0 ? .15 : 0,
  // 0);
  enemy->AddComponent(std::make_shared<BoidComponent>());
  enemy->AddComponent(std::make_shared<pxl::PhysicsComponent>());
  enemy->AddComponent(std::make_shared<pxl::CapsuleCollider>(
      .35f, 1.8f, pxl::ColliderComponent::kDynamic));
  enemy->AddComponent(std::make_shared<DeathCollisionResponse>());
  return enemy;
}

Eigen::Vector3f AiManager::Seek(const std::shared_ptr<pxl::Entity> unit,
                                const Eigen::Vector3f target) const {
  auto enemy = std::dynamic_pointer_cast<Enemy>(unit);
  auto boid = unit->GetComponent<BoidComponent>();

  Eigen::Vector3f desired = target - unit->position;
  desired.normalize();
  desired *= enemy->speed;
  Eigen::Vector3f steer = desired - boid->velocity;
  return steer.normalized();
}

Eigen::Vector3f AiManager::ComputeSeparation(
    const std::shared_ptr<pxl::Entity> unit) const {
  auto boid = unit->GetComponent<BoidComponent>();
  Eigen::Vector3f separation_vec = Eigen::Vector3f::Zero();
  int32_t count = 0;

  for (auto tmp : red_team_) {
    if (tmp.expired()) {
      continue;
    }
    auto other_unit = tmp.lock();
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

  for (auto tmp : red_team_) {
    if (tmp.expired()) {
      continue;
    }
    auto other_unit = tmp.lock();
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
  red_team_.erase(std::remove_if(red_team_.begin(), red_team_.end(),
                                 [](auto a) { return a.expired(); }),
                  red_team_.end());

  for (auto tmp : red_team_) {
    if (tmp.expired()) {
      continue;
    }
    auto unit = tmp.lock();
    auto boid = unit->GetComponent<BoidComponent>();

    // Follow the leader
    /*if (unit != red_leader_) {
      boid->acceleration += Seek(unit, red_leader_->position);
    } else {*/
    boid->acceleration += Seek(unit, player_->position) * 10.0;
    // }
    boid->acceleration += ComputeClustering(unit) * clustering;
    boid->acceleration += ComputeSeparation(unit) * separation;
    boid->velocity += boid->acceleration * time_elapsed;
    boid->velocity.y() = 0;
    unit->rotation.y() =
        atan(boid->velocity.x() / boid->velocity.z()) * 180 / M_PI;
    if (boid->velocity.z() < 0) {
      unit->rotation.y() += 180;
    }
    if (boid->velocity.norm() > unit->speed) {
      boid->velocity = boid->velocity.normalized() * unit->speed;
    }
  }

  for (auto tmp : red_team_) {
    if (tmp.expired()) {
      continue;
    }
    auto unit = tmp.lock();
    auto boid = unit->GetComponent<BoidComponent>();
    unit->position += boid->velocity * time_elapsed;
    boid->acceleration *= 0;
  }
}

void AiManager::Add() {
  auto enemy = CreateEnemy();
  scene_->AddEntity(enemy);
  red_team_.push_back(enemy);
}

void AiManager::Remove() {
  scene_->RemoveEntity(red_team_.back().lock());
  red_team_.pop_back();
}
