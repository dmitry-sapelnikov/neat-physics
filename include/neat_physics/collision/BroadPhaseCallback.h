// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>

namespace nph
{

/// Callback interface for broad-phase collision detection
class BroadPhaseCallback
{
public:
	/// Virtual destructor
	virtual ~BroadPhaseCallback() = default;

	/// Called when a pair of bodies is found to be colliding
	virtual void onCollision(uint32_t bodyIndA, uint32_t bodyIndB) = 0;
};

// namespace nph
}
