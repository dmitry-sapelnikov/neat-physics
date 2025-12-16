#pragma once

// Includes
#include "neat_physics/collision/CollisionPoint.h"

namespace nph
{

// 2-element array of Vec2
using Vec2Array2 = std::array<Vec2, 2>;

// 2-element array of Rotation
using RotationArray2 = std::array<Rotation, 2>;

// 2-element array of Mat22
using Mat22Array2 = std::array<Mat22, 2>;

/// Computes collision points between 2 boxes
/// \return Number of collision points found (0-2)
uint32_t getBoxBoxCollision(
	const Vec2Array2& positions,
	const RotationArray2 rotations,
	const Vec2Array2& halfSizes,
	CollisionPointArray& result);

// End of namespace nph
}
