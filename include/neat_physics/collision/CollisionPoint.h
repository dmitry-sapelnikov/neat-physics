#pragma once

// Includes
#include <array>
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// Maximum number of collision points between 2 geometries
static constexpr uint32_t MAX_COLLISION_POINTS = 2;

/// Collision point between 2 geometries
/// \note Unfortunately it is not possible to apply const-members trick
/// to ensure fulfillment of the member constraints, so we rely only on
/// assertions in the constructor
struct CollisionPoint
{
	/// Pair of geometry features yielding a contact point
	struct GeometryFeature
	{
		/// Geometry index (0 - 1)
		char geometry{ 0 };

		/// Edge index (0 - 3)
		char edge{ 0 };

		[[nodiscard]] bool operator==(
			const GeometryFeature& other) const noexcept
		{
			return geometry == other.geometry && edge == other.edge;
		}

		[[nodiscard]] bool operator<(
			const GeometryFeature& other) const noexcept
		{
			return geometry != other.geometry ?
				geometry < other.geometry :
				edge < other.edge;
		}
	};

	/// A pair of geometry features
	using GeometryFeaturePair = std::array<GeometryFeature, 2>;

	/// Position of the contact point in world space
	Vec2 position;

	/// Contact normal, pointing from body A to body B
	Vec2 normal;

	/// Penetration depth
	float penetration;

	/// Index of the clipping box
	uint32_t clipBoxIndex;

	/// The collision point in the box local frames 
	std::array<Vec2, 2> localPoints;

	/// Contact normal in the clipping box frame
	Vec2 localContactNormal;

	/// A pair of features yielding this contact point
	GeometryFeaturePair featurePair;

	/// Default constructor (no initialization)
	CollisionPoint() noexcept = default;

	/// Constructor
	/// Asserts:
	/// - normal is normalized
	/// - penetration >= 0
	CollisionPoint(
		const Vec2& inPosition,
		const Vec2& inNormal,
		float inPenetration,
		const GeometryFeaturePair& inFeaturePair,
		uint32_t inClipBoxIndex,
		const std::array<Vec2, 2>& inLocalPoints,
		const Vec2& inLocalContactNormal
		) noexcept :

		position(inPosition),
		normal(inNormal),
		penetration(inPenetration),
		featurePair(inFeaturePair),
		clipBoxIndex(inClipBoxIndex),
		localPoints(inLocalPoints),
		localContactNormal(inLocalContactNormal)
	{
		assert(normal.isNormalized());
		assert(penetration >= 0.0f);
		assert(clipBoxIndex == 0 || clipBoxIndex == 1);
		assert(localContactNormal.isNormalized());
	}
};

// Array of the max number of collison points
using CollisionPointArray = std::array<CollisionPoint, MAX_COLLISION_POINTS>;

} // namespace nph
