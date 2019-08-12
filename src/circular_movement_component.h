#include <pixel_engine/component.h>

class CircularMovementComponent : public pxl::Component {
public:
  CircularMovementComponent();
  void Update(float time_elapsed) override;
};