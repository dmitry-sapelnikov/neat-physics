// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>

namespace nph
{

/// Generic inertia trait
template <uint16_t D>
struct InertiaTrait;

/// 2D inertia trait specialization
template <>
struct InertiaTrait<2>
{
	using Type = float;
};

/// Inertia definition via trait
template <uint16_t D>
using Inertia = typename InertiaTrait<D>::Type;

} // namespace nph
