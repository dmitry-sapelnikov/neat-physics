// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec.h"

namespace nph
{

/// Generic vector template
template <uint16_t D>
struct Vec;

/// 2-dimensional vector
template <>
struct Vec<2>
{
	/// X component, intentionally uninitialized
	float x;

	/// Y component, intentionally uninitialized
	float y;

	/// Default constructor (no initialization)
	Vec() noexcept = default;

	/// Constructor with components
	constexpr Vec(float inX, float inY) noexcept :
		x(inX), y(inY)
	{
	}

	/// Returns the squared length of the vector
	[[nodiscard]] float lengthSquared() const noexcept
	{
		return x * x + y * y;
	}

	/// Returns the length of the vector
	[[nodiscard]] float length() const noexcept
	{
		return std::sqrt(lengthSquared());
	}

	/// Returns a normalized version of the vector, or a zero vector
	/// if the vector length < FLT_EPSILON
	[[nodiscard]] Vec getNormalized() const noexcept
	{
		const float len = length();
		if (len < FLT_EPSILON)
		{
			return { 0.0f, 0.0f };
		}
		const float invLen = 1.0f / len;
		return { x * invLen, y * invLen };
	}

	/// Checks if the vector is normalized
	[[nodiscard]] bool isNormalized() const noexcept
	{
		return std::abs(lengthSquared() - 1.0f) < 100.0f * FLT_EPSILON;
	}

	/// Indexing operator (const version)
	[[nodiscard]] float operator[](int index) const noexcept
	{
		assert(index < 2);
		return *(&x + index);
	}

	/// Indexing operator (non-const version)
	[[nodiscard]] float& operator[](int index) noexcept
	{
		assert(index < 2);
		return *(&x + index);
	}

	/// Negation operator
	[[nodiscard]] Vec operator-() const noexcept
	{
		return { -x, -y };
	}

	/// Addition assignment operator
	Vec& operator+=(const Vec& vec) noexcept
	{
		x += vec.x;
		y += vec.y;
		return *this;
	}

	/// Subtraction assignment operator
	Vec& operator-=(const Vec& vec) noexcept
	{
		x -= vec.x;
		y -= vec.y;
		return *this;
	}

	/// Scalar multiplication assignment operator
	Vec& operator*=(float scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		return *this;
	}
};

/// 2D vector alias
using Vec2 = Vec<2>;

/// Dot product of two vectors
inline [[nodiscard]] float dot(
	const Vec2& vecA,
	const Vec2& vecB) noexcept
{
	return vecA.x * vecB.x + vecA.y * vecB.y;
}

/// Cross product of 2 xy vectors
/// \return Scalar z-component of the 3D cross product
inline [[nodiscard]] float cross(
	const Vec2& xyA,
	const Vec2& xyB) noexcept
{
	return xyA.x * xyB.y - xyA.y * xyB.x;
}

/// Cross product of a xy vector and a z-axis value
inline [[nodiscard]] Vec2 cross(
	const Vec2& xy,
	float z) noexcept
{
	return { xy.y * z, -xy.x * z };
}

/// Cross product of a z-axis value and a xy vector
inline [[nodiscard]] Vec2 cross(
	float z,
	const Vec2& xy) noexcept
{
	return { -xy.y * z, xy.x * z };
}

/// Component-wise absolute value of a vector
inline [[nodiscard]] Vec2 abs(const Vec2& vec) noexcept
{
	return { std::abs(vec.x), std::abs(vec.y) };
}

} // namespace nph
