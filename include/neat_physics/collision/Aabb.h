// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec.h"

namespace nph
{

/// 2D axis-aligned bounding box
template <uint16_t D, typename T = float>
struct Aabb
{
	/// Minimum corner
	Vec<D, T> min;

	/// Maximum corner
	Vec<D, T> max;

	/// Default constructor (no initialization)
	Aabb() noexcept = default;

	/// Min-max point constructor; asserts that min <= max
	Aabb(
		const Vec<D, T>& inMin,
		const Vec<D, T>& inMax) noexcept :
		min(inMin),
		max(inMax)
	{
		for (uint16_t i = 0; i < D; ++i)
		{
			assert(min[i] <= max[i]);
		}
	}

	/// Returns the AABB center
	Vec<D, T> getCenter() const noexcept
	{
		return 0.5f * (min + max);
	}

	/// Expands the AABB by a specific factor
	Aabb getExpanded(float factor) const noexcept
	{
		return {
			min - Vec<D, T>(factor),
			max + Vec<D, T>(factor)
		};
	}

	/// Checks if the AABB contains another AABB
	bool contains(const Aabb<D, T>& other) const noexcept
	{
		if constexpr (D == 2)
		{
			return
				(min.x <= other.min.x) &&
				(min.y <= other.min.y) &&
				(max.x >= other.max.x) &&
				(max.y >= other.max.y);
		}
		else
		{
			return
				(min.x <= other.min.x) &&
				(min.y <= other.min.y) &&
				(min.z <= other.min.z) &&
				(max.x >= other.max.x) &&
				(max.y >= other.max.y) &&
				(max.z >= other.max.z);
		}
	}

	/// Checks if the AABB overlaps with another AABB
	bool overlaps(const Aabb<D, T>& other) const noexcept
	{
		if constexpr (D == 2)
		{
			return 
				(min.x <= other.max.x) &&
				(min.y <= other.max.y) &&
				(max.x >= other.min.x) &&
				(max.y >= other.min.y);
		}
		else
		{
			return 
				(min.x <= other.max.x) &&
				(min.y <= other.max.y) &&
				(min.z <= other.max.z) &&
				(max.x >= other.min.x) &&
				(max.y >= other.min.y) &&
				(max.z >= other.min.z);
		}
	}
};

/// Returns the surface area of a 2D AABB
template <typename T>
inline float getSurfaceArea(const Aabb<2, T>& aabb) noexcept
{
	const Vec2 extents = aabb.max - aabb.min;
	return T(2) * (extents.x + extents.y);
}

/// Returns the surface area of a 3D AABB
template <typename T>
inline float getSurfaceArea(const Aabb<3, T>& aabb) noexcept
{
	const Vec3 extents = aabb.max - aabb.min;
	return T(2) * (
		extents.x * extents.y +
		extents.y * extents.z +
		extents.z * extents.x);
}

/// Combines 2 2D AABBs
template <typename T>
inline Aabb<2, T> combineAabbs(const Aabb<2, T>& a, const Aabb<2, T>& b) noexcept
{
	return {
		Vec2(
			std::min(a.min.x, b.min.x),
			std::min(a.min.y, b.min.y)),
		Vec2(
			std::max(a.max.x, b.max.x),
			std::max(a.max.y, b.max.y))
	};
}

/// Combines 2 3D AABBs
template <typename T>
inline Aabb<3, T> combineAabbs(const Aabb<3, T>& a, const Aabb<3, T>& b) noexcept
{
	return {
		Vec<3, T>(
			std::min(a.min.x, b.min.x),
			std::min(a.min.y, b.min.y),
			std::min(a.min.z, b.min.z)),
		Vec<3, T>(
			std::max(a.max.x, b.max.x),
			std::max(a.max.y, b.max.y),
			std::max(a.max.z, b.max.z))
	};
}

/// Combines 2 AABBs
template <uint16_t D, typename T>
inline Aabb<D, T> operator+(const Aabb<D, T>& a, const Aabb<D, T>& b) noexcept
{
	return combineAabbs(a, b);
}

/// Aabb2 alias
using Aabb2 = Aabb<2, float>;

/// Aabb3 alias
using Aabb3 = Aabb<3, float>;

// namespace nph
}
