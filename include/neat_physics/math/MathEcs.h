// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/core/Ecs.h"
#include "neat_physics/math/Kinematics.h"

namespace nph::ecs
{

struct TransformTag {};

template <uint16_t D>
using TransformBlock = Block<TransformTag, Transform<D>>;

struct VelocityTag {};

template <uint16_t D>
using VelocityBlock = Block<VelocityTag, LinearAngularPair<D>>;

} // namespace nph::ecs
