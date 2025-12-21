// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <algorithm>
#include <vector>
#include "neat_physics/Body.h"
#include "neat_physics/collision/Aabb.h"
#include "neat_physics/collision/BroadPhaseCallback.h"

namespace nph
{

/// Broad-phase collision detection using sweep-and-prune algorithm
template <uint16_t D>
class BroadPhase
{
public:
	/// A pair of body indices
	using BodyIndexPair = std::pair<uint32_t, uint32_t>;

	/// Array of body index pairs
	using BodyIndexPairArray = std::vector<BodyIndexPair>;

	/// Array of Aabbs
	using AabbArray = std::vector<Aabb<D>>;

	/// Constructor
	BroadPhase(const BodyArray<D>& bodies) noexcept :
		mBodies(bodies)
	{
	}

	/// Returns the AABBs of the bodies
	[[nodiscard]] const AabbArray& getAabbs() const noexcept
	{
		return mAabbs;
	}

	/// Updates the pairs of bodies which AABBs are overlapping
	void update(BroadPhaseCallback& callback);

private:
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
	void sweepAxis(BroadPhaseCallback& callback);

	/// Reference to the bodies
	const BodyArray<D>& mBodies;

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

template <uint16_t D>
void BroadPhase<D>::update(BroadPhaseCallback& callback)
{
	// Reserve-emplace because Aabb is immutable
	mAabbs.clear();
	mAabbs.reserve(mBodies.size());
	for (size_t i = 0; i < mBodies.size(); ++i)
	{
		mAabbs.emplace_back(getAabb(
			mBodies[i].position,
			mBodies[i].rotation.getMat(),
			mBodies[i].halfSize));
	}

	mActiveMapping.resize(mBodies.size());
	// Very inefficient, but it is really actual since
	// currently there is no body removal,only clearing all
	if (mEndpoints.size() > mBodies.size() * 2)
	{
		mEndpoints.clear();
	}

	// Add endpoints for bodies that have been added since the last update
	assert(mEndpoints.size() % 2 == 0);
	for (int32_t ei = static_cast<int32_t>(mEndpoints.size() >> 1);
		ei < static_cast<int32_t>(mBodies.size()); ++ei) // Endpoint index
	{
		mEndpoints.emplace_back(0.0f, ei, true);
		mEndpoints.emplace_back(0.0f, ei, false);
	}

	// Update endpoint positions
	for (auto& endpoint : mEndpoints)
	{
		endpoint.position = endpoint.isStart ?
			mAabbs[endpoint.index].min.x :
			mAabbs[endpoint.index].max.x;
	}

	// \todo Explicitly use the insertion sort
	std::sort(mEndpoints.begin(), mEndpoints.end());
	sweepAxis(callback);
}

template <uint16_t D>
void BroadPhase<D>::sweepAxis(BroadPhaseCallback& callback)
{
	mActivePoints.clear();
	for (const auto& endpoint : mEndpoints)
	{
		if (endpoint.isStart)
		{
			const uint32_t i1 = endpoint.index;
			const Body<D>& bodyA = mBodies[i1];
			const Aabb<D>& aabbA = mAabbs[i1];

			for (uint32_t i2 : mActivePoints)
			{
				if (bodyA.isStatic() && mBodies[i2].isStatic())
				{
					continue;
				}

				// If y-axes don't intersect
				const Aabb<D>& aabbB = mAabbs[i2];
				bool overlap = true;
				for (uint16_t dim = 1; dim < D; ++dim)
				{
					if (aabbA.max[dim] < aabbB.min[dim] ||
						aabbB.max[dim] < aabbA.min[dim])
					{
						overlap = false;
						break;
					}
				}

				if (!overlap)
				{
					continue;
				}

				if (i1 < i2)
				{
					callback.onCollision(i1, i2);
				}
				else
				{
					callback.onCollision(i2, i1);
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

// namespace nph
}
