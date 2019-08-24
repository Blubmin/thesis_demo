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
                        std::weak_ptr<pxl::Entity> target, bool solve_player)
      : camera(camera),
        player(player),
        target(target),
        solve_player_(solve_player) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const auto player_ptr = player.lock();
    const auto target_ptr = target.lock();
    if (target_ptr == nullptr || player_ptr == nullptr) {
      return false;
    }
    Eigen::Vector4<T> player_pos = Eigen::Vector4d(0, .5, 0, 1).cast<T>();
    player_pos = player_ptr->GetTransform().cast<T>() * player_pos;
    Eigen::Vector4<T> target_pos = Eigen::Vector4d(0, 0, 0, 1).cast<T>();
    target_pos = target_ptr->GetTransform().cast<T>() * target_pos;

    Eigen::Vector3<T> camera_pos(parameters[0], parameters[1], parameters[2]);
    Eigen::Quaternion<T> rotation =
        Eigen::AngleAxis<T>(parameters[4], Eigen::Vector3<T>::UnitY()) *
        Eigen::AngleAxis<T>(parameters[3], Eigen::Vector3<T>::UnitX());
    /*Eigen::Vector3<T> camera_axis(parameters[3], parameters[4],
    parameters[5]); Eigen::AngleAxis<T> camera_rot(camera_axis.norm(),
                                   camera_axis.normalized());*/

    Eigen::Isometry3<T> camera_T_world = Eigen::Isometry3<T>::Identity();
    camera_T_world.translation() = camera_pos;
    camera_T_world.linear() = rotation.toRotationMatrix();
    /*camera_T_world.linear() = camera_rot.toRotationMatrix();
    camera_T_world.linear() = Eigen::Matrix3<T>::Identity();*/
    // LOG(INFO) << std::endl << camera_T_world.linear();

    camera_T_world = camera_T_world.inverse();

    const auto camera_ptr = camera.lock();
    if (camera_ptr == nullptr) {
      return false;
    }
    Eigen::Vector4<T> norm_target_pos = camera_ptr->GetPerspective().cast<T>() *
                                        camera_T_world.matrix() * target_pos;
    Eigen::Vector4<T> norm_player_pos = camera_ptr->GetPerspective().cast<T>() *
                                        camera_T_world.matrix() * player_pos;

    Eigen::Vector3<T> position_diff =
        player_pos.head<3>() -
        Eigen::Vector3<T>(parameters[0], parameters[1], parameters[2]);
    residuals[0] = position_diff[0];
    residuals[1] = position_diff[1];
    residuals[2] = position_diff[2];
    residuals[3] = ceres::pow(position_diff.norm() - 7.0, 2);

    residuals[4] = (norm_target_pos[0] / norm_target_pos[3]) - .33;
    residuals[5] = (norm_target_pos[1] / norm_target_pos[3]) - .33;

    if (solve_player_) {
      residuals[6] = (norm_player_pos[0] / norm_player_pos[3]) + .33;
      residuals[7] = (norm_player_pos[1] / norm_player_pos[3]) + .33;
    } else {
      residuals[6] = T(0.0);
      residuals[7] = T(0.0);
    }

    return true;
  }

  static ceres::CostFunction* Create(std::weak_ptr<pxl::Camera> camera,
                                     std::weak_ptr<pxl::Entity> player,
                                     std::weak_ptr<pxl::Entity> entity,
                                     bool solve_player) {
    ceres::AutoDiffCostFunction<AestheticCostFunction, 8, 5>* cost_function =
        new ceres::AutoDiffCostFunction<AestheticCostFunction, 8, 5>(
            new AestheticCostFunction(camera, player, entity, solve_player));
    return cost_function;
  }

 private:
  const std::weak_ptr<pxl::Camera> camera;
  const std::weak_ptr<pxl::Entity> player;
  const std::weak_ptr<pxl::Entity> target;

  bool solve_player_;
};

AestheticCameraComponent::AestheticCameraComponent() : solve_player(true) {}

void AestheticCameraComponent::Update(float time_elapsed) {
  ceres::Problem problem;

  auto camera_ptr = std::static_pointer_cast<pxl::Camera>(owner.lock());
  std::array<double, 6> parameters;
  Eigen::AngleAxisf rot(camera_ptr->GetTransform().block<3, 3>(0, 0));
  Eigen::Vector3f axis = rot.axis().normalized();
  axis = axis * rot.angle();
  // parameters[0] = axis.x();
  // parameters[1] = axis.y();
  // parameters[2] = axis.z();
  // parameters[3] = camera_ptr->position.x();
  // parameters[4] = camera_ptr->position.y();
  // parameters[5] = camera_ptr->position.z();
  parameters[0] = camera_ptr->position.x();
  parameters[1] = camera_ptr->position.y();
  parameters[2] = camera_ptr->position.z();
  // parameters[3] = axis.x();
  // parameters[4] = axis.y();
  // parameters[5] = axis.z();
  parameters[3] = camera_ptr->rotation.x() / 180 * M_PI;
  parameters[4] = camera_ptr->rotation.y() / 180 * M_PI;

  auto cost =
      AestheticCostFunction::Create(camera_ptr, player_, target_, solve_player);
  problem.AddResidualBlock(cost, NULL, parameters.data());

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
  camera_ptr->position =
      Eigen::Vector3f(parameters[0], parameters[1], parameters[2]);
  // Eigen::Vector3f rotation_axis(parameters[3], parameters[4], parameters[5]);
  // Eigen::AngleAxisf rotation(rotation_axis.norm(),
  // rotation_axis.normalized());
  camera_ptr->rotation.x() = parameters[3] / M_PI * 180.f;
  camera_ptr->rotation.y() = parameters[4] / M_PI * 180.f;
}

void AestheticCameraComponent::SetTarget(std::weak_ptr<pxl::Entity> target) {
  target_ = target;
}

void AestheticCameraComponent::SetPlayer(std::weak_ptr<pxl::Entity> player) {
  player_ = player;
}