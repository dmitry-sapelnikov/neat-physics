// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>

namespace nph
{

/// Generic axis-angle trait
template <uint16_t D>
struct AxisAngleTrait;

/// 2D axis-angle trait specialization
template <>
struct AxisAngleTrait<2>
{
	using Type = float;
};

/// Axis-angle definition via trait
template <uint16_t D>
using AxisAngle = typename AxisAngleTrait<D>::Type;

} // namespace nph
