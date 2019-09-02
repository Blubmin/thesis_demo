#include <pixel_engine/component.h>

#include <vector>

#include <Eigen/Core>
#include <boost/optional.hpp>

class AestheticCameraComponent : public pxl::Component {
 public:
  AestheticCameraComponent();
  void Update(float time_elapsed) override;

  void SetTarget(std::weak_ptr<pxl::Entity> target);
  void SetPlayer(std::weak_ptr<pxl::Entity> player);

  std::vector<bool> constant_residuals;
  std::vector<bool> constant_parameters;

 private:
  std::weak_ptr<pxl::Entity> player_;
  std::weak_ptr<pxl::Entity> target_;

  boost::optional<Eigen::Vector3f> prev_player_pos_;
};