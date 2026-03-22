// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/MathEcs.h"
#include "neat_physics/body/MassProperties.h"

namespace nph::ecs
{
// Body-specific components


struct HalfSizeTag {};
template <uint16_t D>
using HalfSizeBlock = Block<HalfSizeTag, Vec<D>>;

struct MassPropertiesTag {};

template <uint16_t D>
using MassPropertiesBlock = Block<MassPropertiesTag, MassProperties<D>>;

struct FrictionTag {};
using FrictionBlock = Block<FrictionTag, float>;

} // namespace nph::esc
