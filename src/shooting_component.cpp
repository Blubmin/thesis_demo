#include "shooting_component.h"

#define _USE_MATH_DEFINES ;
#include <math.h>

#include <chrono>
#include <random>

#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include <pixel_engine/collider_component.h>
#include <pixel_engine/eigen_utilities.h>
#include <pixel_engine/mesh_loader.h>
#include <pixel_engine/ogl_mesh.h>
#include <pixel_engine/physics_component.h>

namespace {
boost::filesystem::path GetResourcePath() {
  return boost::filesystem::path(__FILE__).parent_path() / "resources";
}
boost::filesystem::path GetMeshPath(const std::string& mesh_file) {
  return GetResourcePath() / "meshes" / mesh_file;
}

float kFiringSpeed = 15;

class RandomRotatingComponent : public pxl::Component {
 public:
  RandomRotatingComponent() {
    std::default_random_engine engine(
        std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> distribution(0, 1);
    auto rand = std::bind(distribution, engine);
    auto theta = rand() * M_PI * 2;
    auto z = rand() * 2 - 1;

    axis_ = Eigen::Vector3f(std::sqrt(1 - z * z) * std::cos(theta),
                            std::sqrt(1 - z * z) * std::sin(theta), z);
    rotation_speed_ = rand() * M_PI * 4;
  }

  void Update(float time_elapsed) override {
    auto owner_ptr = owner.lock();
    auto rot = Eigen::GetRotation(owner_ptr->GetTransform());
    rot = Eigen::AngleAxisf(rotation_speed_ * time_elapsed, axis_) * rot;

    auto eulers = rot.eulerAngles(1, 0, 2);
    std::swap(eulers.x(), eulers.y());
    owner_ptr->rotation = eulers * 180 / M_PI;
  }

 private:
  Eigen::Vector3f axis_;
  float rotation_speed_;
};

// If the ball collides with anything, remove it from the game after a fixed set
// of time
class BallPopCollisionResponse : public pxl::CollisionResponse {
  const float kTimerMax = .5;

 public:
  BallPopCollisionResponse() : has_collided(false), collision_timer(0) {}

  void Update(float time_elapsed) override {
    if (has_collided) {
      collision_timer += time_elapsed;
    }
    if (collision_timer > kTimerMax) {
      GetOwner()->RemoveFromScene();
    }
  }

  void Respond(btManifoldPoint& pt, const btCollisionObject* this_object,
               const btCollisionObject* other_object) override {
    has_collided = true;
  }

  bool has_collided;
  float collision_timer;
};
}  // namespace

Beachball::Beachball()
    : pxl::MeshEntity(pxl::MeshLoader::LoadMesh<pxl::OglMesh>(
          GetMeshPath("beachball.obj"))), spent(false) {
}

ShootingComponent::ShootingComponent() : ShootingComponent(nullptr) {}

ShootingComponent::ShootingComponent(std::shared_ptr<pxl::Scene> scene)
    : scene_(scene) {}

std::shared_ptr<pxl::Entity> ShootingComponent::CreateBall() {
  auto camera = owner.lock()->GetChild<pxl::Camera>();
  auto forward = Eigen::GetZAxis(camera->GetTransform());
  forward *= -1;

  auto ball = std::make_shared<Beachball>();
  ball->Bind();
  ball->position = Eigen::GetPosition(camera->GetTransform());
  ball->scale = Eigen::Vector3f::Ones() * .2;

  auto random_angle = std::bind(
      std::uniform_real_distribution<float>(0, 360),
      std::default_random_engine(
          std::chrono::system_clock::now().time_since_epoch().count()));

  ball->rotation =
      Eigen::Vector3f(random_angle(), random_angle(), random_angle());
  ball->AddComponent(
      std::make_shared<pxl::PhysicsComponent>(forward * kFiringSpeed));
  ball->AddComponent(std::make_shared<RandomRotatingComponent>());
  ball->AddComponent(std::make_shared<pxl::SphereCollider>(
      .2, pxl::ColliderComponent::kDynamic));

  ball->AddComponent(std::make_shared<BallPopCollisionResponse>());

  return ball;
}

void ShootingComponent::Update(float time_elapsed) {
  if (ImGui::IsMouseClicked(GLFW_MOUSE_BUTTON_1)) {
    auto ball = CreateBall();
    scene_->AddEntity(ball);
  }
}