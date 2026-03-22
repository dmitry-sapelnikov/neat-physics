// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/AxisAngle.h"
#include "neat_physics/math/Rotation.h"

namespace nph
{

/// Position-rotation pair
template <uint16_t D>
struct Transform
{
	Vec<D> position;
	Rotation<D> rotation;

	Vec<D> operator*(const Vec<D>& localPoint) const noexcept
	{
		return position + rotation.getMat() * localPoint;
	}
};

/// Linear-angular pair (velocity / force)
template <uint16_t D>
struct LinearAngularPair
{
	Vec<D> linear;
	AxisAngle<D> angular;
};

} // namespace nph
