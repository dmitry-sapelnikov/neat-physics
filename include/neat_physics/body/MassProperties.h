// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Rotation.h"
#include "neat_physics/body/Inertia.h"

namespace nph
{

/// Mass properties block
template <uint16_t D>
struct MassProperties
{
	float invMass;
	Inertia<D> invLocalInertia;
	Inertia<D> invWorldInertia;

	void updateWorldInertia(const Rotation<D>& rotation) noexcept
	{
		invWorldInertia =
			rotation.getMat() * 
			invLocalInertia *
			rotation.getMat().getTransposed();
	}

	[[nodiscard]] bool isStatic() const noexcept
	{
		return invMass == 0.0f;
	}
};

} // namespace nph