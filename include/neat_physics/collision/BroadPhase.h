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
	using AabbArray = std::vector<Aabb2>;

	/// Constructor
	BroadPhase(const BodyArray<2>& bodies) noexcept :
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
	const BodyArray<2>& mBodies;

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
