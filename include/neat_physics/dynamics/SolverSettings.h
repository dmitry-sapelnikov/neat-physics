// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cassert>
#include <cstdint>

namespace nph
{

/// Dynamic solver settings
class SolverSettings
{
public:
	/// Constructor
	SolverSettings() = default;

	/// Returns the number of velocity iterations for constraint solvers
	[[nodiscard]] uint32_t getVelocityIterations() const noexcept
	{
		return mVelocityIterations;
	}

	/// Sets the number of velocity solver iterations for constraint solvers
	/// asserts that iterations > 0
	void setVelocityIterations(uint32_t iterations)
	{
		assert(iterations > 0);
		mVelocityIterations = iterations;
	}

	/// Returns the number of position iterations for constraint solvers
	[[nodiscard]] uint32_t getPositionIterations() const noexcept
	{
		return mPositionIterations;
	}

	/// Sets the number of position solver iterations for constraint solvers
	void setPositionIterations(uint32_t iterations)
	{
		mPositionIterations = iterations;
	}

	/// Returns the position correction factor for constraint solvers
	[[nodiscard]] float getPositionCorrectionFactor() const noexcept
	{
		return mPositionCorrectionFactor;
	}

	/// Sets the position correction factor for constraint solvers
	/// asserts that factor is in [0, 1]
	[[nodiscard]] void setPositionCorrectionFactor(float factor)
	{
		assert(0.0f <= factor && factor <= 1.0f);
		mPositionCorrectionFactor = factor;
	}

	/// Returns the allowed penetration between geometries for constraint solvers
	[[nodiscard]] float getAllowedPenetration() const noexcept
	{
		return mAllowedPenetration;
	}

	/// Sets the allowed penetration between geometries for constraint solvers
	/// asserts that penetration >= 0
	[[nodiscard]] void setAllowedPenetration(float penetration)
	{
		assert(penetration >= 0.0f);
		mAllowedPenetration = penetration;
	}

private:
	/// Number of velocity iterations for constraint solvers
	uint32_t mVelocityIterations = 10;

	/// Number of position iterations for constraint solvers
	uint32_t mPositionIterations = 3;

	// Position correction factor
	float mPositionCorrectionFactor = 0.2f;

	// Allowed penetration between geometries
	float mAllowedPenetration = 0.001f;
};

} // namespace nph
