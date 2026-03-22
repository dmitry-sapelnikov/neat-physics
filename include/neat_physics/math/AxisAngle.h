// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec3.h"

namespace nph
{

/// Generic axis-angle trait
template <uint16_t D, typename T>
struct AxisAngleTrait;

/// 2D axis-angle trait specialization
template <typename T>
struct AxisAngleTrait<2, T>
{
	/// Axis-angle representation for 2D
	using Type = T;
};

/// 3D axis-angle trait specialization
template <typename T>
struct AxisAngleTrait<3, T>
{
	/// Axis-angle representation for 3D
	using Type = Vec<3, T>;
};

/// Axis-angle definition via trait
template <uint16_t D, typename T = float>
using AxisAngle = typename AxisAngleTrait<D, T>::Type;

} // namespace nph
