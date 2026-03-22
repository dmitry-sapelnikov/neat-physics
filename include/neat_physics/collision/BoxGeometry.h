// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <array>
#include "neat_physics/math/MathFunctions.h"
#include "neat_physics/math/Vec2.h"
#include "neat_physics/math/Vec3.h"

namespace nph
{

/// Box geometry static data
template <uint16_t D>
struct BoxGeometry;

/// 2D box geometry static data specialization
template<>
struct BoxGeometry<2>
{
	//        e0
	//    v1 ---- v0
	//    |        |
	// e1 |        | e3
	//    |        |
	//    v2 ---- v3
	//        e2

	/// Number of vertices
	static constexpr uint32_t VERTEX_COUNT = 4;

	/// Number of faces (edges)
	static constexpr uint32_t FACE_COUNT = 4;

	/// Number of vertices per face
	static constexpr uint32_t VERTICES_PER_FACE = 2;

	/// Max intersection points between 2 boxes
	static constexpr uint32_t MAX_INTERSECTION_POINTS = 2;

	/// Face normals
	static constexpr std::array<Vec2, 4> NORMALS = {
		Vec2{  0.0f,  1.0f },
		Vec2{ -1.0f,  0.0f },
		Vec2{  0.0f, -1.0f },
		Vec2{  1.0f,  0.0f }
	};

	/// Indices of axes parallel to face normals
	static constexpr std::array<int, 4> AXES_INDICES = { 1, 0, 1, 0 };

	/// Vertex to face indices mapping
	static constexpr std::array<std::array<uint32_t, 2>, 4>
		VERTEX_FACE_INDICES = { {
			{3, 0},
			{0, 1},
			{1, 2},
			{2, 3}
		} };

	/// Face to vertex indices mapping
	static constexpr std::array<std::array<uint32_t, 2>, 4>
		FACE_VERTEX_INDICES = { {
			{0, 1},
			{1, 2},
			{2, 3},
			{3, 0}
		} };

	/// Adjacent face indices for each face
	static constexpr std::array<std::array<uint32_t, 2>, 4>
		ADJACENT_FACE_INDICES = { {
			{3, 1},
			{0, 2},
			{1, 3},
			{2, 0}
		} };

	/// Returns the face index in the given direction
	static [[nodiscard]] uint32_t getFaceIndex(const Vec2& direction) noexcept
	{
		if (std::abs(direction.x) > std::abs(direction.y))
		{
			// +-X direction
			return direction.x > 0.0 ? 3 : 1;
		}
		else
		{
			// +-Y direction
			return direction.y > 0.0 ? 0 : 2;
		}
	}

	static [[nodiscard]] Vec2 getVertex(
		uint32_t vertexIndex,
		const Vec2& boxHalfSize) noexcept
	{	
		// Vertex signs for box vertices
		static constexpr Vec2 VERTEX_SIGNS[8] = {
			{  1.0f,  1.0f },
			{ -1.0f,  1.0f },
			{ -1.0f, -1.0f },
			{  1.0f, -1.0f },

			{  1.0f,  1.0f },
			{ -1.0f,  1.0f },
			{ -1.0f, -1.0f },
			{  1.0f, -1.0f }
		};

		return VERTEX_SIGNS[vertexIndex] * boxHalfSize;
	}
};

/// 3D box geometry static data specialization
template <>
struct BoxGeometry<3>
{
	//       v5 ----e4---- v4
	//       /|           /|
	//     e9 |   F0    e8 |        y
	//     / e5     F5  /  e7       ^
	//   v1 ----e0---- v0  |        |
	//    |F1 |        | F3|        0--> x
	//    |  v6 ----e6-|-- v7      /
	//   e1  /  F4     e3 /       z
	//    |e10    F2   | e11
	//    |/           |/
	//   v2 ----e2---- v3
	/// Number of vertices
	static constexpr uint32_t VERTEX_COUNT = 8;

	/// Number of faces
	static constexpr uint32_t FACE_COUNT = 6;

	/// Number of vertices per face
	static constexpr uint32_t VERTICES_PER_FACE = 4;

	/// Max intersection points between 2 boxes
	static constexpr uint32_t MAX_INTERSECTION_POINTS = 8;

	/// Face normals
	static constexpr std::array<Vec3, 6> NORMALS = {
		Vec3{ 0.0f,  1.0f,  0.0f},
		Vec3{-1.0f,  0.0f,  0.0f},
		Vec3{ 0.0f, -1.0f,  0.0f},
		Vec3{ 1.0f,  0.0f,  0.0f},
		Vec3{ 0.0f,  0.0f,  1.0f},
		Vec3{ 0.0f,  0.0f, -1.0f}
	};

	/// Indices of axes parallel to face normals
	static constexpr std::array<int, 6>
		AXES_INDICES = {
			1, 0, 1, 0, 2, 2
	};

	/// Vertex to face indices mapping
	static constexpr std::array<std::array<uint32_t, 3>, 8>
		VERTEX_FACE_INDICES = { {
			{3, 0, 4},
			{0, 1, 4},
			{1, 2, 4},
			{2, 3, 4},
			{3, 0, 5},
			{0, 1, 5},
			{1, 2, 5},
			{2, 3, 5}
		} };

	/// Face to vertex indices mapping
	static constexpr std::array<std::array<uint32_t, 4>, 6>
		FACE_VERTEX_INDICES = { {
			{0, 4, 5, 1},
			{1, 5, 6, 2},
			{2, 6, 7, 3},
			{3, 7, 4, 0},
			{0, 1, 2, 3},
			{4, 7, 6, 5}
		} };

	/// Adjacent face indices for each face
	static constexpr std::array<std::array<uint32_t, 4>, 6>
		ADJACENT_FACE_INDICES = { {
			{3, 1, 4, 5},
			{0, 2, 4, 5},
			{1, 3, 4, 5},
			{2, 0, 4, 5},
			{0, 1, 2, 3},
			{0, 3, 2, 1}
		} };

	static [[nodiscard]] uint32_t getFaceIndex(const Vec3& direction)
	{
		uint32_t maxAbsAxis = static_cast<uint32_t>(getMaxAbsAxis(direction));
		switch (maxAbsAxis)
		{
		case 0:
			return direction.x > 0.0f ? 3 : 1; // +-X

		case 1:
			return direction.y > 0.0f ? 0 : 2; // +-Y

		case 2:
			return direction.z > 0.0f ? 4 : 5; // +-Z

		default:
			return 0;
		}
	}

	/// Returns the vertex position given its index and box half-size
	static [[nodiscard]] Vec3 getVertex(
		uint32_t vertexIndex,
		const Vec3& boxHalfSize) noexcept
	{
		// Vertex signs for box vertices
		static constexpr Vec3 VERTEX_SIGNS[8] = {
			{  1.0f,  1.0f,  1.0f },
			{ -1.0f,  1.0f,  1.0f },
			{ -1.0f, -1.0f,  1.0f },
			{  1.0f, -1.0f,  1.0f },
			{  1.0f,  1.0f, -1.0f },
			{ -1.0f,  1.0f, -1.0f },
			{ -1.0f, -1.0f, -1.0f },
			{  1.0f, -1.0f, -1.0f }
		};
		return VERTEX_SIGNS[vertexIndex] * boxHalfSize;
	}
};

/// Computes the AABB for a box-shaped geometry
template <uint16_t D>
Aabb<D> getAabb(
	const Vec<D>& position,
	const Mat<D, D>& rotation,
	const Vec<D>& halfSize)
{
	const Vec<D> aabbHalfSize = abs(rotation) * halfSize;
	return {
		position - aabbHalfSize,
		position + aabbHalfSize
	};
}

// End of namespace nph
}
