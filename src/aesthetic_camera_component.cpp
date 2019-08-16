#include "aesthetic_camera_component.h"

#include <array>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <pixel_engine/camera.h>
#include <pixel_engine/eigen_typedefs.h>

class AestheticCostFunction {
 public:
  AestheticCostFunction(std::weak_ptr<pxl::Camera> camera,
                        std::weak_ptr<pxl::Entity> target)
      : camera(camera), target(target) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const auto target_ptr = target.lock();
    if (target_ptr == nullptr) {
      return false;
    }
    Eigen::Vector4<T> target_pos = Eigen::Vector4d(0, 1, 0, 1).cast<T>();
    target_pos = target_ptr->GetTransform().cast<T>() * target_pos;

    Eigen::Vector3<T> axis(parameters[0], parameters[1], parameters[2]);
    Eigen::AngleAxis<T> angle_axis(axis.norm(), axis.normalized());
    Eigen::Vector3<T> camera_pos(parameters[0], parameters[1], parameters[2]);
    // Eigen::Vector3<T> camera_pos(parameters[3], parameters[4],
    // parameters[5]);

    Eigen::Isometry3<T> camera_T_world = Eigen::Isometry3<T>::Identity();
    camera_T_world.translation() = camera_pos;
    camera_T_world = camera_T_world.inverse();

    const auto camera_ptr = camera.lock();
    if (camera_ptr == nullptr) {
      return false;
    }
    Eigen::Vector4<T> norm_pos = camera_ptr->GetPerspective().cast<T>() *
                                 camera_T_world.matrix() * target_pos;

    Eigen::Vector3<T> position_diff =
        target_pos.head<3>() -
        Eigen::Vector3<T>(parameters[0], parameters[1], parameters[2]);
    residuals[0] = position_diff[0];
    residuals[1] = position_diff[1];
    residuals[2] = position_diff[2];

    residuals[3] = ceres::pow(position_diff.norm() - 7.0, 2);

    residuals[4] = norm_pos[0];
    residuals[5] = norm_pos[1];

    LOG(INFO) << target_pos.head<3>().transpose();
    return true;
  }

  static ceres::CostFunction* Create(std::weak_ptr<pxl::Camera> camera,
                                     std::weak_ptr<pxl::Entity> entity) {
    ceres::AutoDiffCostFunction<AestheticCostFunction, 6, 3>* cost_function =
        new ceres::AutoDiffCostFunction<AestheticCostFunction, 6, 3>(
            new AestheticCostFunction(camera, entity));
    return cost_function;
  }

 private:
  std::weak_ptr<pxl::Camera> camera;
  const std::weak_ptr<pxl::Entity> target;
};

AestheticCameraComponent::AestheticCameraComponent() {}

void AestheticCameraComponent::Update(float time_elapsed) {
  ceres::Problem problem;

  auto camera_ptr = std::static_pointer_cast<pxl::Camera>(owner.lock());
  std::array<double, 6> parameters;
  Eigen::AngleAxisf rot(camera_ptr->GetTransform().block<3, 3>(0, 0));
  Eigen::Vector3f axis = rot.axis().normalized();
  axis *= rot.angle();
  // parameters[0] = axis.x();
  // parameters[1] = axis.y();
  // parameters[2] = axis.z();
  // parameters[3] = camera_ptr->position.x();
  // parameters[4] = camera_ptr->position.y();
  // parameters[5] = camera_ptr->position.z();
  parameters[0] = camera_ptr->position.x();
  parameters[1] = camera_ptr->position.y();
  parameters[2] = camera_ptr->position.z();

  auto cost = AestheticCostFunction::Create(camera_ptr, target_);
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
  LOG(ERROR) << camera_ptr->position.transpose();
  LOG(ERROR) << summary.FullReport();
}

void AestheticCameraComponent::SetTarget(std::weak_ptr<pxl::Entity> target) {
  target_ = target;
}