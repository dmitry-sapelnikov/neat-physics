// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <algorithm>
#include "neat_physics/Body.h"
#include "Plane.h"
#include "NarrowPhase.h"

namespace nph
{

namespace
{

/// Clipped point
struct ClippedPoint
{
	/// Position
	Vec2 position;

	/// Feature pair yielding this point
	CollisionPoint::GeometryFeaturePair featurePair{};
};

/// Clipped edge
using ClippedEdge = std::array<ClippedPoint, 2>;

/// Clips an edge by a plane
bool clipEdgeByPlane(
	const ClippedEdge& source,
	const Plane& clipPlane,
	uint32_t clipBody,
	uint32_t clipAxisInd,
	ClippedEdge& target)
{
	uint32_t pointCount = 0;
	std::array<float, 2> distances;
	for (size_t pi = 0; pi < 2; ++pi)
	{
		distances[pi] = clipPlane.getDistance(source[pi].position);
		if (distances[pi] <= 0.0f)
		{
			target[pointCount++] = source[pi];
		}
	}

	if (pointCount == 1 &&
		distances[0] * distances[1] < 0.0f)
	{
		ClippedPoint& point = target[pointCount++];

		const float lerpFactor = distances[0] / (distances[0] - distances[1]);
		point.position =
			source[0].position +
			lerpFactor * (source[1].position - source[0].position);

		// Keep the feature of the point which lies in the negative halfspace of the plane,
		// while overriding the feature of the point lying in the positive halfspace
		// with the clip body and clip edge
		const uint32_t pi = (distances[0] <= 0.0f);
		point.featurePair = source[pi].featurePair;
		point.featurePair[pi].geometry = clipBody;
		point.featurePair[pi].edge = clipAxisInd;
	}
	return pointCount == 2;
}

} // anonymous namespace


uint32_t getBoxBoxCollision(
	const Vec2Array2& positions,
	const RotationArray2 rotations,
	const Vec2Array2& halfSizes,
	CollisionPointArray& result)
{
	assert(halfSizes[0].x > 0.0f && halfSizes[0].y > 0.0f);
	assert(halfSizes[1].x > 0.0f && halfSizes[1].y > 0.0f);

	// Inverse rotation matrices
	const Mat22Array2 invRotations{
		rotations[0].getInverseMat(),
		rotations[1].getInverseMat()
	};

	// Step 1: find the min penetration or a separating axis
	uint32_t clipBoxInd;
	uint32_t clipAxisInd; // 0 - x axis, 1 - y axis
	Vec2 minPenetrationDir;
	{
		const Vec2 centersVec = positions[1] - positions[0];
		// A -> B relative rotation
		const Mat22 abRelRotation = invRotations[0] * rotations[1].getMat();

		const Mat22Array2 absRelRotations{
			abs(abRelRotation),
			abs(abRelRotation.getTransposed())
		};

		float minPenetration = std::numeric_limits<float>::max();
		for (uint32_t bi = 0; bi < 2; ++bi) // box index
		{
			const Vec2 otherBoxProjections =
				abs(invRotations[bi] * centersVec) -
				absRelRotations[1 - bi] * halfSizes[1 - bi];

			const Vec2 penetrations = halfSizes[bi] - otherBoxProjections;
			for (uint32_t ai = 0; ai < 2; ++ai) // axis index
			{
				if (penetrations[ai] < 0.0f)
				{
					return 0;
				}

				if (penetrations[ai] < minPenetration)
				{
					minPenetration = penetrations[ai];
					clipBoxInd = bi;
					clipAxisInd = ai;
				}
			}
		}
		minPenetrationDir = rotations[clipBoxInd].getMat()[clipAxisInd];
		// Should be directed from a to b
		if (dot(minPenetrationDir, centersVec) < 0.0f)
		{
			minPenetrationDir = -minPenetrationDir;
		}
	}

	// We need to invert clip normal if the clipping box is box B
	const Vec2 clipNormal = (clipBoxInd == 0) ?
		minPenetrationDir :
		-minPenetrationDir;

	// Step 2: find the incident edge
	const uint32_t incidentBoxInd = 1 - clipBoxInd;
	ClippedEdge edge;
	{
		//        e0
		//    v1      v0
		//    |        |
		// e1 |        | e3
		//    |        |
		//    v2      v3
		//        e2

		// Vertex signs for box vertices
		static constexpr int VERTEX_SIGNS[4][2] = {
			{  1,  1 },
			{ -1,  1 },
			{ -1, -1 },
			{  1, -1 }
		};

		// Clip normal is in the world space,
		// we need to transform it to the local space of the incident box
		const Vec2 incidentDir = -(invRotations[incidentBoxInd] * clipNormal);

		uint32_t incidentEdge;
		if (std::abs(incidentDir.x) > std::abs(incidentDir.y))
		{
			// +-X direction
			incidentEdge = incidentDir.x > 0.0 ? 3 : 1;
		}
		else
		{
			// +-Y direction
			incidentEdge = incidentDir.y > 0.0 ? 0 : 2;
		}

		for (uint32_t pi = 0; pi < 2; ++pi) // edge point index
		{
			ClippedPoint& point = edge[pi];
			const uint32_t pointIndex = (incidentEdge + pi) % 4;
			const Vec2 localPosition{
				VERTEX_SIGNS[pointIndex][0] * halfSizes[incidentBoxInd].x,
				VERTEX_SIGNS[pointIndex][1] * halfSizes[incidentBoxInd].y };

			for (uint32_t fi = 0; fi < 2; ++fi) // point feature index
			{
				point.featurePair[fi].geometry = incidentBoxInd;
				// e3, e0 for v0,
				// e0, e1 for v1, etc.
				// for fi = 0 we get the previous index for pointIndex,
				// for fi = 1 we get the point index
				point.featurePair[fi].edge = (pointIndex + 3 - 3 * fi) % 4;
			}
			point.position =
				positions[incidentBoxInd] +
				rotations[incidentBoxInd].getMat() * localPosition;
		}
	}

	// Step 3: clip the incident edge over the side edges of the clip box
	{
		// The side normal is another axis of the clip box
		const uint32_t sideAxisInd = 1 - clipAxisInd;

		const Vec2 sideNormal1 = rotations[clipBoxInd].getMat()[sideAxisInd];

		const Plane sideClipPlane1(
			sideNormal1,
			positions[clipBoxInd],
			halfSizes[clipBoxInd][sideAxisInd]);
		// clipEdge 0 (x-direction) -> 2, clipEdge 1 (y-direction) -> 1
		const uint32_t sideEdge1 = 2 - clipAxisInd;

		const Plane sideClipPlane2(
			-sideNormal1,
			positions[clipBoxInd],
			halfSizes[clipBoxInd][sideAxisInd]);
		const uint32_t sideEdge2 = (sideEdge1 + 2) % 4; // 180 degrees rotation

		ClippedEdge temp;
		// First clip the edge storing result to temp, then clip temp back to edge
		if (!clipEdgeByPlane(edge, sideClipPlane1, clipBoxInd, sideEdge1, temp) ||
			!clipEdgeByPlane(temp, sideClipPlane2, clipBoxInd, sideEdge2, edge))
		{
			return 0;
		}
	}

	// Step 4: create the collision points
	uint32_t resultPointCount = 0;
	{
		const Plane clipPlane(
			clipNormal,
			positions[clipBoxInd],
			halfSizes[clipBoxInd][clipAxisInd]);

		for (uint32_t pi = 0; pi < 2; ++pi) // point index
		{
			ClippedPoint& point = edge[pi];
			const float penetration = -clipPlane.getDistance(point.position);
			if (penetration < 0.0f)
			{
				continue;
			}

			Vec2Array2 localPoints;
			const Vec2 resultPosition =
				edge[pi].position + penetration * clipNormal;

			localPoints[clipBoxInd] =
				invRotations[clipBoxInd] *
				(resultPosition - positions[clipBoxInd]);

			localPoints[incidentBoxInd] =
				invRotations[incidentBoxInd] *
				(edge[pi].position - positions[incidentBoxInd]);

			// Keep ordering in case if we have a flip of the
			// clipping-incident boxes.
			// This keeps the collision points persistent
			if (point.featurePair[1] < point.featurePair[0])
			{
				std::swap(point.featurePair[0], point.featurePair[1]);
			}

			result[resultPointCount++] = CollisionPoint(
				resultPosition,
				minPenetrationDir,
				penetration,
				point.featurePair,
				clipBoxInd,
				localPoints,
				invRotations[clipBoxInd] * clipNormal
			);
		}
	}
	return resultPointCount;
}

} // namespace nph
