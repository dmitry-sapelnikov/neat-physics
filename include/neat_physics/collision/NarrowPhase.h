// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <limits>
#include "neat_physics/math/MathFunctions.h"
#include "neat_physics/math/Rotation.h"
#include "neat_physics/collision/CollisionPoint.h"
#include "neat_physics/collision/Plane.h"
#include "neat_physics/collision/BoxGeometry.h"
#include "neat_physics/collision/CollisionSettings.h"

namespace nph
{

/// Clipped point
template <uint16_t D>
struct ClippedPoint
{
	/// Position
	Vec<D> position;

	/// Feature pair yielding this point
	CollisionPoint<D>::GeometryFeaturePair featurePair;
};

/// Clipped face
template <uint16_t D>
using ClippedFace =
std::array<ClippedPoint<D>, BoxGeometry<D>::MAX_INTERSECTION_POINTS>;

/// Clips an edge by a plane
template <uint16_t D>
uint32_t clipFaceByPlane(
	const ClippedFace<D>& source,
	uint32_t sourcePointsCount,
	const Plane<D>& clipPlane,
	uint32_t clipBody,
	uint32_t clipFaceInd,
	ClippedFace<D>& target)
{
	uint32_t pointCount = 0;
	for (uint32_t si = 0; si < sourcePointsCount - (sourcePointsCount == 2); ++si)
	{
		std::array<float, 2> distances;
		uint32_t pointsInside = 0;
		for (size_t pi = 0; pi < 2; ++pi)
		{
			const ClippedPoint<D>& point = source[(si + pi) % sourcePointsCount];
			distances[pi] = clipPlane.getDistance(point.position);
			if (distances[pi] <= 0.0f)
			{
				if (pi == 0 || sourcePointsCount == 2)
				{
					target[pointCount++] = point;
				}
				++pointsInside;
			}
		}

		if (distances[0] * distances[1] < 0.0f)
		{
			ClippedPoint<D>& point = target[pointCount++];

			const float lerpFactor = distances[0] / (distances[0] - distances[1]);
			const ClippedPoint<D>& point1 = source[si];
			const ClippedPoint<D>& point2 = source[(si + 1) % sourcePointsCount];

			point.position =
				point1.position +
				lerpFactor * (point2.position - point1.position);

			// Keep the feature of the point which lies in the negative halfspace of the plane,
			// while overriding the feature of the point lying in the positive halfspace
			// with the clip body and clip edge
			const uint32_t fi = (distances[0] <= 0.0f);
			point.featurePair = source[(si + fi) % sourcePointsCount].featurePair;
			point.featurePair[fi].geometry = static_cast<char>(clipBody);
			point.featurePair[fi].face = static_cast<char>(clipFaceInd);
		}
	}
	return pointCount;
}

template <uint16_t D>
using BoxFaceSidePlanes =
std::array<std::pair<uint32_t, Plane<D>>, BoxGeometry<D>::VERTICES_PER_FACE>;

template <uint16_t D>
BoxFaceSidePlanes<D> getSidePlanes(
	uint32_t faceIndex,
	const Vec<D>& position,
	const Mat<D, D>& rotation,
	const Vec<D>& halfSize)
{
	const auto& adjacentFaces = BoxGeometry<D>::ADJACENT_FACE_INDICES[faceIndex];
	BoxFaceSidePlanes<D> result;
	for (uint32_t i = 0; i < BoxGeometry<D>::VERTICES_PER_FACE; ++i)
	{
		result[i].first = adjacentFaces[i];

		const uint32_t faceAxisInd =
			BoxGeometry<D>::AXES_INDICES[adjacentFaces[i]];

		const Vec<D>& normal = BoxGeometry<D>::NORMALS[adjacentFaces[i]];

		result[i].second = Plane<D>(
			normal[faceAxisInd] > 0.0f ?
				rotation[faceAxisInd] :
				-rotation[faceAxisInd],
			position,
			halfSize[BoxGeometry<D>::AXES_INDICES[adjacentFaces[i]]]);
	}
	return result;
}

template <uint16_t D>
ClippedFace<D> getIncidentFaceVertices(
	const uint32_t clipBoxInd,
	const Vec<D>& clipNormal,
	const Vec<D>& position,
	const Rotation<D>& rotation,
	const Mat<D, D>& invRotation,
	const Vec<D>& halfSize)
{
	// Clip normal is in the world space,
	// we need to transform it to the local space of the incident box
	const Vec<D> incidentDir = -(invRotation * clipNormal);

	ClippedFace<D> result;
	const uint32_t faceInd = BoxGeometry<D>::getFaceIndex(incidentDir);

	auto vertexIndices = BoxGeometry<D>::FACE_VERTEX_INDICES[faceInd];
	for (uint32_t vi = 0; vi < vertexIndices.size(); ++vi)
	{
		ClippedPoint<D>& point = result[vi];
		const Vec<D> localPosition = BoxGeometry<D>::getVertex(
			vertexIndices[vi],
			halfSize);

		const auto& faceIndices = BoxGeometry<D>::VERTEX_FACE_INDICES[vertexIndices[vi]];
		for (uint32_t fi = 0; fi < D; ++fi) // point feature index
		{
			point.featurePair[fi].geometry = static_cast<char>(1 - clipBoxInd);
			point.featurePair[fi].face = static_cast<char>(faceIndices[fi]);
		}

		point.position =
			position +
			rotation.getMat() * localPosition;
	}
	return result;
}

inline uint32_t reduceCollisionManifold(
	ClippedFace<3>& points,
	uint32_t pointsCount)
{
	using IndArray = std::array<uint32_t, BoxGeometry<3>::MAX_INTERSECTION_POINTS>;

	if (pointsCount <= CollisionPoint<3>::MAX_POINTS)
	{
		return pointsCount;
	}

	// Use a greedy algorithm to remove points 
	// which areas with adjacent vertices are the smallest
	IndArray sourceIndArray;
	for (uint32_t i = 0; i < pointsCount; ++i)
	{
		sourceIndArray[i] = i;
	}
	IndArray targetIndArray;

	IndArray* source = &sourceIndArray;
	uint32_t sourceSize = pointsCount;
	IndArray* target = &targetIndArray;

	for (uint32_t i = 0; i < pointsCount - CollisionPoint<3>::MAX_POINTS; ++i)
	{
		float minArea = std::numeric_limits<float>::max();
		uint32_t minInd = 0;
		for (uint32_t pi = 0; pi < sourceSize; ++pi)
		{
			const Vec3& prevPoint =
				points[(*source)[(pi + sourceSize - 1) % sourceSize]].position;

			const Vec3& currPoint = points[(*source)[pi]].position;

			const Vec3& nextPoint =
				points[(*source)[(pi + 1) % sourceSize]].position;

			const float areaSquared =
				cross(prevPoint - currPoint, nextPoint - currPoint).lengthSquared();

			if (areaSquared < minArea)
			{
				minArea = areaSquared;
				minInd = pi;
			}
		}

		uint32_t targetInd = 0;
		for (uint32_t pi = 0; pi < sourceSize; ++pi)
		{
			if (pi != minInd)
			{
				(*target)[targetInd++] = (*source)[pi];
			}
		}

		std::swap(source, target);
		--sourceSize;
	}

	for (uint32_t pi = 0; pi < CollisionPoint<3>::MAX_POINTS; ++pi)
	{
		const uint32_t readInd = (*source)[pi];
		assert(pi <= readInd);
		if (readInd != pi)
		{
			points[pi] = points[readInd];
		}
	}
	return CollisionPoint<3>::MAX_POINTS;
}

template <uint16_t D>
uint32_t getBoxBoxCollision(
	const std::array<Vec<D>, 2>& positions,
	const std::array<Rotation<D>, 2>& rotations,
	const std::array<Vec<D>, 2>& halfSizes,
	const CollisionSettings& settings,
	CollisionPointArray<D>& result)
{
	const float separationFactor =
		settings.getSeparationFactor();

	const float axisPreferenceFactor =
		settings.getMinPenetrationAxisPreferenceFactor();

	/// Factor to make face axes of box #1 more preferable that box #2,
	/// face axes more preferable than edge-edge axes
	using MatArray2 = std::array<Mat<D, D>, 2>;

	for (uint16_t ai = 0; ai < D; ++ai) // axis index
	{
		assert(halfSizes[0][ai] > 0.0f);
		assert(halfSizes[1][ai] > 0.0f);
	}

	// Inverse rotation matrices
	const MatArray2 invRotations{
		rotations[0].getInverseMat(),
		rotations[1].getInverseMat()
	};

	// Step 1: find the min penetration or a separating axis
	const Vec<D> centersVec = positions[1] - positions[0];
	float minPenetration = std::numeric_limits<float>::max();

	uint32_t clipBoxInd = 0;
	Vec<D> minPenetrationDir;
	{
		uint32_t clipAxisInd = 0;
		// A -> B relative rotation
		const Mat<D, D> relRotation = invRotations[1] * rotations[0].getMat();

		const MatArray2 absRelRotations{
			abs(relRotation),
			abs(relRotation.getTransposed())
		};

		for (uint32_t bi = 0; bi < 2; ++bi) // box index
		{
			const Vec<D> otherBoxProjections =
				abs(invRotations[bi] * centersVec) -
				absRelRotations[1 - bi] * halfSizes[1 - bi];

			const Vec<D> penetrations = halfSizes[bi] - otherBoxProjections;
			for (uint16_t ai = 0; ai < D; ++ai) // axis index
			{
				if (penetrations[ai] < -separationFactor)
				{
					return 0;
				}

				if (penetrations[ai] < minPenetration * axisPreferenceFactor)
				{
					minPenetration = penetrations[ai];
					clipBoxInd = bi;
					clipAxisInd = ai;
				}
			}
		}
		minPenetrationDir = rotations[clipBoxInd].getMat()[clipAxisInd];
	}

	// Test edge-edge axes
	if constexpr (D == 3)
	{
		bool edgeEdgeContact = false;
		for (uint16_t e1 = 0; e1 < 3; ++e1) // box A edge index
		{
			const Vec<D>& edgeDir1 = rotations[0].getMat()[e1];
			for (uint16_t e2 = 0; e2 < 3; ++e2) // box B edge index
			{
				const Vec<D>& edgeDir2 = rotations[1].getMat()[e2];

				Vec<D> axis = cross(edgeDir1, edgeDir2);
				const float lengthSquared = axis.lengthSquared();
				if (lengthSquared < 100.0f * FLT_EPSILON)
				{
					continue;
				}

				const float invLength = 1.0f / std::sqrt(lengthSquared);
				axis *= invLength;

				// Calculate the radius (half-extent) of each box along the separating axis
				// This is the sum of projections of the box half-extents onto the axis
				const Vec<D> localAxisA = invRotations[0] * axis;
				const Vec<D> localAxisB = invRotations[1] * axis;

				float radiusA = dot(halfSizes[0], abs(localAxisA));
				float radiusB = dot(halfSizes[1], abs(localAxisB));
				const float distance = std::abs(dot(axis, centersVec));
				float penetration = radiusA + radiusB - distance;

				if (penetration < -separationFactor)
				{
					return 0;
				}

				if (penetration < minPenetration * axisPreferenceFactor)
				{
					edgeEdgeContact = true;
					minPenetration = penetration;
					minPenetrationDir = axis;
				}
			}
		}

		if (edgeEdgeContact)
		{
			const uint32_t box1FaceInd = BoxGeometry<3>::getFaceIndex(
				invRotations[0] * minPenetrationDir);

			const Vec3& box1FaceNormal =
				rotations[0].getMat() *
				BoxGeometry<3>::NORMALS[box1FaceInd];

			const uint32_t box2FaceInd = BoxGeometry<3>::getFaceIndex(
				invRotations[1] * minPenetrationDir);

			const Vec3& box2FaceNormal =
				rotations[1].getMat() *
				BoxGeometry<3>::NORMALS[box2FaceInd];

			if (std::abs(dot(box1FaceNormal, minPenetrationDir)) >
				std::abs(dot(box2FaceNormal, minPenetrationDir)))
			{
				minPenetrationDir = box1FaceNormal;
				clipBoxInd = 0;
			}
			else
			{
				minPenetrationDir = box2FaceNormal;
				clipBoxInd = 1;
			}
		}
	}

	// Should be directed from a to b
	if (dot(minPenetrationDir, centersVec) < 0.0f)
	{
		minPenetrationDir = -minPenetrationDir;
	}

	// We need to invert clip normal if the clipping box is box B
	const Vec<D> clipNormal = (clipBoxInd == 0) ?
		minPenetrationDir :
		-minPenetrationDir;

	const uint32_t clipFaceInd = BoxGeometry<D>::getFaceIndex(
		invRotations[clipBoxInd] * clipNormal);

	uint32_t incidentBoxInd = 1 - clipBoxInd;
	ClippedFace<D> incidentFace = getIncidentFaceVertices(
		clipBoxInd,
		clipNormal,
		positions[incidentBoxInd],
		rotations[incidentBoxInd],
		invRotations[incidentBoxInd],
		halfSizes[incidentBoxInd]);

	// Step 3: clip the incident edge over the side edges of the clip box
	// The side normal is another axis of the clip box
	ClippedFace<D> temp;
	ClippedFace<D>* source = &incidentFace;
	ClippedFace<D>* target = &temp;

	const auto sidePlanes = getSidePlanes(
		clipFaceInd,
		positions[clipBoxInd],
		rotations[clipBoxInd].getMat(),
		halfSizes[clipBoxInd]);

	uint32_t clippedPointsCount = BoxGeometry<D>::VERTICES_PER_FACE;
	for (const auto& sidePlane : sidePlanes)
	{
		clippedPointsCount = clipFaceByPlane(
			*source,
			clippedPointsCount,
			sidePlane.second,
			clipBoxInd,
			sidePlane.first,
			*target);

		if (clippedPointsCount < 2)
		{
			return 0;
		}
		std::swap(source, target);
	}

	// Step 4: create the collision points
	uint32_t resultPointCount = 0;
	{
		const uint32_t clipAxisInd = BoxGeometry<D>::AXES_INDICES[clipFaceInd];
		const Plane<D> clipPlane(
			clipNormal,
			positions[clipBoxInd],
			halfSizes[clipBoxInd][clipAxisInd]);

		uint32_t pointsCount = 0;
		for (uint32_t pi = 0; pi < clippedPointsCount; ++pi) // point index
		{
			ClippedPoint<D>& point = (*source)[pi];
			const float penetration = -clipPlane.getDistance(point.position);
			if (penetration >= -separationFactor)
			{
				if (pointsCount != pi)
				{
					(*source)[pointsCount] = point;
				}
				++pointsCount;
			}
		}

		if constexpr (D == 3)
		{
			pointsCount = reduceCollisionManifold(
				*source,
				pointsCount);
		}

		for (uint32_t pi = 0; pi < pointsCount; ++pi) // point index
		{
			ClippedPoint<D>& point = (*source)[pi];
			// \todo: optimize double-calculation of the penetration
			const float penetration = -clipPlane.getDistance(point.position);
			const Vec<D> planePoint = point.position + penetration * clipNormal;

			std::array<Vec<D>, 2> localPoints;
			localPoints[clipBoxInd] =
				invRotations[clipBoxInd] *
				(planePoint - positions[clipBoxInd]);

			localPoints[incidentBoxInd] =
				invRotations[incidentBoxInd] *
				(point.position - positions[incidentBoxInd]);

			std::sort(point.featurePair.begin(), point.featurePair.end());

			result[resultPointCount++] = CollisionPoint<D>(
				point.position,
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

// End of namespace nph
}
