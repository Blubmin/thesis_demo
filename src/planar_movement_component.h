#include <pixel_engine/component.h>

class PlanarMovementComponent : public pxl::Component {
 public:
  PlanarMovementComponent();
  void Update(float time_elapsed) override;
};