// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <vector>
#include "neat_physics/Body.h"
#include "neat_physics/collision/Aabb.h"
#include "neat_physics/collision/BroadPhaseCallback.h"

namespace nph
{

/// Broad-phase collision detection using sweep-and-prune algorithm
class BroadPhase
{
public:
	/// A pair of body indices
	using BodyIndexPair = std::pair<uint32_t, uint32_t>;

	/// Array of body index pairs
	using BodyIndexPairArray = std::vector<BodyIndexPair>;

	/// Array of Aabbs
	using AabbArray = std::vector<Aabb>;

	/// Constructor
	BroadPhase(const BodyArray& bodies) noexcept :
		mBodies(bodies)
	{
	}

	/// Returns the AABBs of the bodies
	[[nodiscard]] const AabbArray& getAabbs() const noexcept
	{
		return mAabbs;
	}

	/// Updates the pairs of bodies which AABBs are overlapping
	template <typename Callback>
	void update(Callback callback)
	{
		// Reserve-emplace because Aabb is immutable
		prepareSweep();
		sweepAxis(callback);
	}

private:
	/// Prepares for the sweep-and-prune algorithm
	void prepareSweep();

	/// Endpoint of a segment
	struct Endpoint
	{
		/// Coordinate value
		float position;

		/// Segment index
		uint32_t index;

		/// Start point flag
		bool isStart;

		/// Operator < for sorting
		bool operator<(const Endpoint& other) const noexcept
		{
			return position != other.position ?
				position < other.position :
				isStart < other.isStart;
		}
	};

	/// Sweeps the AABBs along the X axis
	template <typename Callback>
	void sweepAxis(Callback callback)
	{
		mActivePoints.clear();
		for (const auto& endpoint : mEndpoints)
		{
			if (endpoint.isStart)
			{
				const uint32_t i1 = endpoint.index;
				const Body& bodyA = mBodies[i1];
				const Aabb& aabbA = mAabbs[i1];

				for (uint32_t i2 : mActivePoints)
				{
					if (bodyA.isStatic() && mBodies[i2].isStatic())
					{
						continue;
					}

					// If y-axes don't intersect
					const Aabb& aabbB = mAabbs[i2];
					if (aabbA.max.y < aabbB.min.y ||
						aabbB.max.y < aabbA.min.y)
					{
						continue;
					}

					if (i1 < i2)
					{
						callback(i1, i2);
					}
					else
					{
						callback(i2, i1);
					}
				}
				mActiveMapping[endpoint.index] = static_cast<uint32_t>(mActivePoints.size());
				mActivePoints.push_back(i1);
			}
			else
			{
				// Swap and pop
				const uint32_t idx = mActiveMapping[endpoint.index];
				const uint32_t last = mActivePoints.back();
				mActivePoints[idx] = last;
				mActiveMapping[last] = idx;
				mActivePoints.pop_back();
			}
		}
	}

	/// Reference to the bodies
	const BodyArray& mBodies;

	/// AABBs of the bodies
	AabbArray mAabbs;

	/// Endpoints for the sweep-and-prune algorithm
	std::vector<Endpoint> mEndpoints;

	/// Active set of segment indices during the pruning phase
	std::vector<uint32_t> mActivePoints;

	/// Mapping for segment index - index in active set during
	/// the pruning phase
	std::vector<uint32_t> mActiveMapping;
};

// namespace nph
}
