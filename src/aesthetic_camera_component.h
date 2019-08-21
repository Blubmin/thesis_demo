#include <pixel_engine/component.h>

class AestheticCameraComponent : public pxl::Component {
 public:
  AestheticCameraComponent();
  void Update(float time_elapsed) override;

  void SetTarget(std::weak_ptr<pxl::Entity> target);
  void SetPlayer(std::weak_ptr<pxl::Entity> player);

 private:
  std::weak_ptr<pxl::Entity> player_;
  std::weak_ptr<pxl::Entity> target_;
};