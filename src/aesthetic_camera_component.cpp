#include "aesthetic_camera_component.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <array>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <imgui/imgui.h>
#include <pixel_engine/camera.h>
#include <pixel_engine/eigen_utilities.h>
#include <pixel_engine/game.h>
#include <pixel_engine/physics_component.h>

class AestheticCostFunction {
 public:
  AestheticCostFunction() {}

  AestheticCostFunction(Eigen::Matrix4f perspective_transform,
                        Eigen::Matrix4f player_transform,
                        Eigen::Matrix4f target_transform,
                        std::vector<bool> constant_residuals)
      : perspective_transform(perspective_transform),
        player_transform(player_transform),
        target_transform(target_transform),
        constant_residuals_(constant_residuals) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, const T* const z,
                  const T* const pitch, const T* const yaw,
                  T* residuals) const {
    // Get positions
    Eigen::Vector4<T> player_pos = Eigen::Vector4d(0, .7, 0, 1).cast<T>();
    player_pos = player_transform.cast<T>() * player_pos;
    Eigen::Vector4<T> target_pos = Eigen::Vector4d(0, .7, 0, 1).cast<T>();
    target_pos = target_transform.cast<T>() * target_pos;

    Eigen::Vector3<T> camera_pos(*x, *y, *z);
    Eigen::Quaternion<T> rotation =
        Eigen::AngleAxis<T>(*yaw, Eigen::Vector3<T>::UnitY()) *
        Eigen::AngleAxis<T>(*pitch, Eigen::Vector3<T>::UnitX());

    Eigen::Isometry3<T> camera_T_world = Eigen::Isometry3<T>::Identity();
    camera_T_world.translation() = camera_pos;
    camera_T_world.linear() = rotation.toRotationMatrix();

    camera_T_world = camera_T_world.inverse();

    Eigen::Vector4<T> norm_target_pos =
        perspective_transform.cast<T>() * camera_T_world.matrix() * target_pos;
    Eigen::Vector4<T> norm_player_pos =
        perspective_transform.cast<T>() * camera_T_world.matrix() * player_pos;

    Eigen::Vector3<T> position_diff = player_pos.head<3>() - camera_pos;
    residuals[0] = position_diff[0];
    residuals[1] = position_diff[1];
    residuals[2] = position_diff[2];
    residuals[3] = ceres::pow(position_diff.norm() - 7.0, 2);

    residuals[4] = (norm_target_pos[0] / norm_target_pos[3]) - .33;
    residuals[5] = (norm_target_pos[1] / norm_target_pos[3]) - .33;

    residuals[6] = ((norm_player_pos[0] / norm_player_pos[3]) + .33) * 5.0;
    residuals[7] = ((norm_player_pos[1] / norm_player_pos[3]) + .33) * 5.0;

    if (norm_target_pos[2] < 0.0) {
      residuals[4] = T(100.0);
      residuals[5] = T(100.0);
    }

    if (norm_player_pos[2] < 0.0) {
      residuals[6] = T(100.0);
      residuals[7] = T(100.0);
    }

    for (int i = 0; i < constant_residuals_.size(); ++i) {
      if (constant_residuals_[i]) {
        residuals[i] = T(0.0);
      }
    }

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Matrix4f& perspective_transform,
      const Eigen::Matrix4f& player_transform,
      const Eigen::Matrix4f& entity_transform,
      std::vector<bool> constant_parameters) {
    ceres::AutoDiffCostFunction<AestheticCostFunction, 8, 1, 1, 1, 1, 1>*
        cost_function = new ceres::AutoDiffCostFunction<AestheticCostFunction,
                                                        8, 1, 1, 1, 1, 1>(
            new AestheticCostFunction(perspective_transform, player_transform,
                                      entity_transform, constant_parameters));
    return cost_function;
  }

 private:
  Eigen::Matrix4f perspective_transform;
  Eigen::Matrix4f player_transform;
  Eigen::Matrix4f target_transform;

  std::vector<bool> constant_residuals_;
};

class DistanceCostFunction {
 public:
  DistanceCostFunction(Eigen::Vector3f target_position, float target_distance)
      : target_position(target_position), target_distance(target_distance) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, const T* const z,
                  T* residuals) const {
    residuals[0] =
        (Eigen::Vector3<T>(*x, *y, *z) - target_position.cast<T>()).norm() -
        T(target_distance);
    residuals[0] *= residuals[0];

    return true;
  }

  static ceres::CostFunction* Create(Eigen::Vector3f target_position,
                                     float target_distance) {
    ceres::AutoDiffCostFunction<DistanceCostFunction, 1, 1, 1, 1>*
        cost_function =
            new ceres::AutoDiffCostFunction<DistanceCostFunction, 1, 1, 1, 1>(
                new DistanceCostFunction(target_position, target_distance));
    return cost_function;
  }

 private:
  Eigen::Vector3f target_position;
  float target_distance;
};

class TargetCostFunction {
 public:
  TargetCostFunction(Eigen::Matrix4f perspective_transform,
                     Eigen::Matrix4f target_transform,
                     Eigen::Vector2f target_projected, float weight)
      : perspective_transform(perspective_transform),
        target_transform(target_transform),
        target_projected(target_projected),
        weight(weight) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, const T* const z,
                  const T* const pitch, const T* const yaw,
                  T* residuals) const {
    // Get positions
    Eigen::Vector4<T> target_pos =
        target_transform.cast<T>() * Eigen::Vector4f(0, 0, 0, 1).cast<T>();

    Eigen::Vector3<T> camera_pos(*x, *y, *z);
    Eigen::Quaternion<T> rotation =
        Eigen::AngleAxis<T>(*yaw, Eigen::Vector3<T>::UnitY()) *
        Eigen::AngleAxis<T>(*pitch, Eigen::Vector3<T>::UnitX());

    Eigen::Isometry3<T> camera_T_world = Eigen::Isometry3<T>::Identity();
    camera_T_world.translation() = camera_pos;
    camera_T_world.linear() = rotation.toRotationMatrix();

    camera_T_world = camera_T_world.inverse();

    Eigen::Vector4<T> norm_target_pos =
        perspective_transform.cast<T>() * camera_T_world.matrix() * target_pos;

    residuals[0] = ((norm_target_pos[0] / norm_target_pos[3]) -
                    target_projected.cast<T>().x()) *
                   T(weight);
    residuals[1] = ((norm_target_pos[1] / norm_target_pos[3]) -
                    target_projected.cast<T>().y()) *
                   T(weight);

    if (norm_target_pos[2] < 0.0) {
      residuals[0] = T(100.0);
      residuals[1] = T(100.0);
    }

    // for (int i = 0; i < constant_residuals_.size(); ++i) {
    //  if (constant_residuals_[i]) {
    //    residuals[i] = T(0.0);
    //  }
    //}

    return true;
  };

  static ceres::CostFunction* Create(
      const Eigen::Matrix4f& perspective_transform,
      const Eigen::Matrix4f& target_transform,
      Eigen::Vector2f target_projection, float weight) {
    ceres::AutoDiffCostFunction<TargetCostFunction, 2, 1, 1, 1, 1,
                                1>* cost_function =
        new ceres::AutoDiffCostFunction<TargetCostFunction, 2, 1, 1, 1, 1, 1>(
            new TargetCostFunction(perspective_transform, target_transform,
                                   target_projection, weight));
    return cost_function;
  }

 private:
  Eigen::Matrix4f perspective_transform;
  Eigen::Matrix4f target_transform;
  Eigen::Vector2f target_projected;
  float weight;
};

AestheticCameraComponent::AestheticCameraComponent()
    : constant_residuals(8),
      constant_parameters(5),
      average_framerate_(100, 0) {
  for (auto& val : constant_residuals) {
    val = false;
  }

  for (auto& val : constant_parameters) {
    val = false;
  }
}

void AestheticCameraComponent::Update(float time_elapsed) {
  time_elapsed_ = time_elapsed;

  if (ImGui::IsKeyPressed(GLFW_KEY_R)) {
    float mean = 0;
    for (auto val : average_framerate_) {
      mean += val;
    }
    mean /= manager_.lock()->red_team_.size();
    LOG(INFO) << manager_.lock()->red_team_.size() << ", " << mean;
  }
}

void AestheticCameraComponent::SetTarget(std::weak_ptr<pxl::Entity> target) {
  target_ = target;
}

void AestheticCameraComponent::SetPlayer(std::weak_ptr<pxl::Entity> player) {
  player_ = player;
  prev_player_pos_ = player_.lock()->position;
}

void AestheticCameraComponent::SetManager(std::weak_ptr<AiManager> manager) {
  manager_ = manager;
}

std::function<void()> AestheticCameraComponent::RunSolver() {
  Eigen::Matrix4f player_transform = player_.lock()->GetTransform();
  player_transform.block<3, 1>(0, 3) += Eigen::Vector3f(0, .7, 0);
  /*Eigen::Vector3f player_velocity =
      player_.lock()->GetComponent<pxl::PhysicsComponent>()->velocity;
  player_transform.block<3, 1>(0, 3) =
      Eigen::GetPosition(player_transform) + player_velocity * time_elapsed_;*/
  Eigen::Matrix4f camera_transform = owner.lock()->GetTransform();
  Eigen::Matrix4f perspective_transform =
      std::dynamic_pointer_cast<pxl::Camera>(owner.lock())->GetPerspective();
  Eigen::Vector3f camera_rotation = owner.lock()->rotation;
  // Eigen::Matrix4f target_transform = target_.lock()->GetTransform();
  std::vector<Eigen::Matrix4f> target_transforms;
  std::vector<float> weights;
  for (auto tmp : manager_.lock()->red_team_) {
    if (tmp.expired()) {
      continue;
    }
    auto unit = tmp.lock();
    if (unit->disable) {
      continue;
    }
    auto transform = unit->GetTransform();
    transform.block<3, 1>(0, 3) += Eigen::Vector3f(0, .7, 0);
    target_transforms.push_back(transform);
    weights.push_back(unit->weight);
  }

  return [=]() mutable {
    ceres::Problem problem;

    std::array<double, 5> parameters;
    Eigen::AngleAxisf rot(Eigen::GetRotation(camera_transform));
    Eigen::Vector3f axis = rot.axis().normalized();
    axis = axis * rot.angle();

    // Improve camera initialization by adding in player shift
    if (prev_player_pos_) {
      camera_transform.block<3, 1>(0, 3) =
          Eigen::GetPosition(camera_transform) +
          (Eigen::GetPosition(player_transform) - prev_player_pos_.get());
    }

    auto camera_pos = Eigen::GetPosition(camera_transform);

    parameters[0] = camera_pos.x();
    parameters[1] = camera_pos.y();
    parameters[2] = camera_pos.z();
    parameters[3] = camera_rotation.x() / 180 * M_PI;
    parameters[4] = camera_rotation.y() / 180 * M_PI;

    // Adds cost function for each enemy
    for (int i = 0; i < target_transforms.size(); ++i) {
      auto transform = target_transforms.at(i);
      auto cost =
          TargetCostFunction::Create(perspective_transform, transform,
                                     Eigen::Vector2f(.33, .33), weights.at(i));
      problem.AddResidualBlock(cost, NULL, parameters.data(),
                               parameters.data() + 1, parameters.data() + 2,
                               parameters.data() + 3, parameters.data() + 4);
    }

    // Adds player cost function
    auto player_cost = TargetCostFunction::Create(
        perspective_transform, player_transform, Eigen::Vector2f(-.33, -.33), 1);
    problem.AddResidualBlock(player_cost, NULL, parameters.data(),
                             parameters.data() + 1, parameters.data() + 2,
                             parameters.data() + 3, parameters.data() + 4);

    // Adds cost function for distance to player
    auto distance_cost =
        DistanceCostFunction::Create(Eigen::GetPosition(player_transform), 7);
    problem.AddResidualBlock(distance_cost, NULL, parameters.data(),
                             parameters.data() + 1, parameters.data() + 2);

    // Keep rotations from flipping the camera upside-down
    problem.SetParameterLowerBound(parameters.data() + 3, 0, -M_PI_2);
    problem.SetParameterUpperBound(parameters.data() + 3, 0, M_PI_2);

    for (int i = 0; i < constant_parameters.size(); ++i) {
      const auto val = constant_parameters[i];
      if (val) {
        problem.SetParameterBlockConstant(parameters.data() + i);
      }
    }

    ceres::Solver::Options options;
    options.num_threads = 12;
    options.max_num_iterations = 100;
    // options.parameter_tolerance = 1e-6;
    // options.function_tolerance = 1e-6;
    options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;

    ceres::Solver::Summary summary;
    auto time = std::chrono::system_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto duration = std::chrono::system_clock::now() - time;
    float time_elapsed =
        std::chrono::duration_cast<
            std::chrono::duration<float, std::ratio<1, 1>>>(duration)
            .count();

    /* if (summary.termination_type ==
     ceres::TerminationType::FAILURE) { LOG(ERROR) <<
     summary.FullReport(); return;
     }*/

    pxl::Game::RenderingThread.Post(
        UpdateTransform(std::move(parameters), time_elapsed));
  };
}

std::function<void()> AestheticCameraComponent::UpdateTransform(
    std::array<double, 5> parameters, float time_elapsed) {
  return [&, parameters = parameters, time_elapsed = time_elapsed]() {
    auto player_ptr = std::static_pointer_cast<pxl::Entity>(player_.lock());
    auto camera_ptr = std::static_pointer_cast<pxl::Camera>(owner.lock());
    auto target_ptr = std::static_pointer_cast<pxl::Camera>(target_.lock());

    // Set the camera position to the solved position
    camera_ptr->position =
        Eigen::Vector3f(parameters[0], parameters[1], parameters[2]);

    // Set the camera rotation to the solved rotations
    camera_ptr->rotation.x() = parameters[3] / M_PI * 180.f;
    camera_ptr->rotation.y() = parameters[4] / M_PI * 180.f;

    // Compute framerate
    average_framerate_.push_back(time_elapsed);

    // Kick off next solve
    pxl::Game::BackgroundThreadPool.Post(RunSolver());
  };
}
