#include "aesthetic_camera_component.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <array>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <pixel_engine/camera.h>
#include <pixel_engine/eigen_typedefs.h>

class AestheticCostFunction {
 public:
  AestheticCostFunction(std::weak_ptr<pxl::Camera> camera,
                        std::weak_ptr<pxl::Entity> player,
                        std::weak_ptr<pxl::Entity> target,
                        std::vector<bool> constant_residuals)
      : camera(camera),
        player(player),
        target(target),
        constant_residuals_(constant_residuals) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, const T* const z,
                  const T* const pitch, const T* const yaw,
                  T* residuals) const {
    const auto player_ptr = player.lock();
    const auto target_ptr = target.lock();
    if (target_ptr == nullptr || player_ptr == nullptr) {
      return false;
    }
    Eigen::Vector4<T> player_pos = Eigen::Vector4d(0, .5, 0, 1).cast<T>();
    player_pos = player_ptr->GetTransform().cast<T>() * player_pos;
    Eigen::Vector4<T> target_pos = Eigen::Vector4d(0, 0, 0, 1).cast<T>();
    target_pos = target_ptr->GetTransform().cast<T>() * target_pos;

    Eigen::Vector3<T> camera_pos(*x, *y, *z);
    Eigen::Quaternion<T> rotation =
        Eigen::AngleAxis<T>(*yaw, Eigen::Vector3<T>::UnitY()) *
        Eigen::AngleAxis<T>(*pitch, Eigen::Vector3<T>::UnitX());

    Eigen::Isometry3<T> camera_T_world = Eigen::Isometry3<T>::Identity();
    camera_T_world.translation() = camera_pos;
    camera_T_world.linear() = rotation.toRotationMatrix();

    camera_T_world = camera_T_world.inverse();

    const auto camera_ptr = camera.lock();
    if (camera_ptr == nullptr) {
      return false;
    }
    Eigen::Vector4<T> norm_target_pos = camera_ptr->GetPerspective().cast<T>() *
                                        camera_T_world.matrix() * target_pos;
    Eigen::Vector4<T> norm_player_pos = camera_ptr->GetPerspective().cast<T>() *
                                        camera_T_world.matrix() * player_pos;

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

  static ceres::CostFunction* Create(std::weak_ptr<pxl::Camera> camera,
                                     std::weak_ptr<pxl::Entity> player,
                                     std::weak_ptr<pxl::Entity> entity,
                                     std::vector<bool> constant_parameters) {
    ceres::AutoDiffCostFunction<AestheticCostFunction, 8, 1, 1, 1, 1, 1>*
        cost_function =
            new ceres::AutoDiffCostFunction<AestheticCostFunction, 8, 1, 1, 1,
                                            1, 1>(new AestheticCostFunction(
                camera, player, entity, constant_parameters));
    return cost_function;
  }

 private:
  const std::weak_ptr<pxl::Camera> camera;
  const std::weak_ptr<pxl::Entity> player;
  const std::weak_ptr<pxl::Entity> target;

  std::vector<bool> constant_residuals_;
};

AestheticCameraComponent::AestheticCameraComponent()
    : constant_residuals(8), constant_parameters(5) {
  for (auto& val : constant_residuals) {
    val = false;
  }

  for (auto& val : constant_parameters) {
    val = false;
  }
}

void AestheticCameraComponent::Update(float time_elapsed) {
  ceres::Problem problem;

  auto player_ptr = player_.lock();
  auto camera_ptr = std::static_pointer_cast<pxl::Camera>(owner.lock());
  std::array<double, 5> parameters;
  Eigen::AngleAxisf rot(camera_ptr->GetTransform().block<3, 3>(0, 0));
  Eigen::Vector3f axis = rot.axis().normalized();
  axis = axis * rot.angle();

  // Improve camera initialization by adding in player shift
  if (prev_player_pos_) {
    camera_ptr->position += player_ptr->position - prev_player_pos_.get();
  }
  prev_player_pos_ = player_ptr->position;

  parameters[0] = camera_ptr->position.x();
  parameters[1] = camera_ptr->position.y();
  parameters[2] = camera_ptr->position.z();
  parameters[3] = camera_ptr->rotation.x() / 180 * M_PI;
  parameters[4] = camera_ptr->rotation.y() / 180 * M_PI;

  auto cost = AestheticCostFunction::Create(camera_ptr, player_, target_,
                                            constant_residuals);
  problem.AddResidualBlock(cost, NULL, parameters.data(), parameters.data() + 1,
                           parameters.data() + 2, parameters.data() + 3,
                           parameters.data() + 4);
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
  ceres::Solve(options, &problem, &summary);

  if (summary.termination_type == ceres::TerminationType::FAILURE) {
    LOG(ERROR) << summary.FullReport();
    return;
  }

  // Set the camera position to the solved position
  camera_ptr->position =
      Eigen::Vector3f(parameters[0], parameters[1], parameters[2]);

  // Set the camera rotation to the solved rotations
  camera_ptr->rotation.x() = parameters[3] / M_PI * 180.f;
  camera_ptr->rotation.y() = parameters[4] / M_PI * 180.f;
}

void AestheticCameraComponent::SetTarget(std::weak_ptr<pxl::Entity> target) {
  target_ = target;
}

void AestheticCameraComponent::SetPlayer(std::weak_ptr<pxl::Entity> player) {
  player_ = player;
}