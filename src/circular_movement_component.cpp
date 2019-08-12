#include "circular_movement_component.h"

#include <pixel_engine/entity.h>

CircularMovementComponent::CircularMovementComponent() {}

void CircularMovementComponent::Update(float time_elapsed) {
  auto entity = owner.lock();
  if (entity == nullptr) {
    return;
  }
  entity->rotation.y() += time_elapsed * 90 / 2.f;
}