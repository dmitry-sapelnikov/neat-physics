// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Rotation.h"
#include "neat_physics/collision/CollisionPoint.h"

namespace nph
{

/// Computes collision points between two box-shaped geometries
template <uint16_t D>
uint32_t getBoxBoxCollision(
	const std::array<Vec<D>, 2>& positions,
	const std::array<Rotation<D>, 2>& rotations,
	const std::array<Vec<D>, 2>& halfSizes,
	CollisionPointArray<D>& result);

// End of namespace nph
}
