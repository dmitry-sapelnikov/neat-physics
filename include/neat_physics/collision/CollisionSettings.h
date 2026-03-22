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
/// Collision settings
class CollisionSettings
{
public:
	/// Constructor
	CollisionSettings() = default;

	/// Returns the AABB tree expansion factor for fat AABBs
	[[nodiscard]] float getAabbTreeExpansionFactor() const noexcept
	{
		return mAabbTreeExpansionFactor;
	}

	/// Sets the AABB tree expansion factor for fat AABBs
	void setAabbTreeExpansionFactor(float factor)
	{
		assert(factor >= 0.0f);
		mAabbTreeExpansionFactor = factor;
	}

	/// Returns the factor to prefer the minimum penetration axis when
	/// multiple min penetration axes are possible
	[[nodiscard]] float getMinPenetrationAxisPreferenceFactor() const noexcept
	{
		return mMinPenetrationAxisPreferenceFactor;
	}

	/// Sets the factor to prefer the minimum penetration axis when
	/// multiple min penetration axes are possible
	void setMinPenetrationAxisPreferenceFactor(float factor)
	{
		assert(0.0f <= factor && factor <= 1.0f);
		mMinPenetrationAxisPreferenceFactor = factor;
	}

	/// Returns the distance between geometries at which 
	/// they are considered separated
	[[nodiscard]] float getSeparationFactor() const noexcept
	{
		return mSeparationFactor;
	}

	/// Sets the distance between geometries at which
	/// they are considered separated
	void setSeparationFactor(float factor)
	{
		assert(factor >= 0.0f);
		mSeparationFactor = factor;
	}

private:
	/// AABB tree expansion factor for fat AABBs
	float mAabbTreeExpansionFactor = 0.05f;

	/// Factor to stick to the minimum penetration axis 
	/// when multiple min penetration axes are possible
	float mMinPenetrationAxisPreferenceFactor = 1.0f;

	/// Distance between geometries at which they are considered separated
	float mSeparationFactor = 0.001f;
};

} // namespace nph
