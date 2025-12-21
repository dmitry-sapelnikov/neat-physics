// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/collision/CollisionManifold.h"

namespace nph
{

/// Callback interface for the collision system update
template <uint16_t D>
class CollisionCallback
{
public:
	/// Virtual destructor
	virtual ~CollisionCallback() = default;

	/// Called when a collision manifold is created
	virtual void onCollision(const CollisionManifold<D>& manifold) = 0;
};

} // namespace nph
