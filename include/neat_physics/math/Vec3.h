// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cfloat>
#include "neat_physics/math/Vec.h"

namespace nph
{

/// Represents a 3D vector
template <>
struct Vec<3>
{
	/// X component, intentionally uninitialized
	float x;

	/// Y component, intentionally uninitialized
	float y;

	/// Z component, intentionally uninitialized
	float z;

	/// Default constructor (no initialization)
	Vec() noexcept = default;

	/// Constructor with components
	constexpr Vec(float inX, float inY, float inZ) noexcept :
		x(inX), y(inY), z(inZ)
	{
	}

	/// Returns the squared length of the vector
	[[nodiscard]] float lengthSquared() const noexcept
	{
		return x * x + y * y + z * z;
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
			return { 0.0f, 0.0f, 0.0f };
		}
		const float invLen = 1.0f / len;
		return { x * invLen, y * invLen, z * invLen };
	}

	/// Checks if the vector is near zero
	[[nodiscard]] bool isNearZero() const noexcept
	{
		return lengthSquared() < 100.0f * FLT_EPSILON;
	}

	/// Checks if the vector is normalized
	[[nodiscard]] bool isNormalized() const noexcept
	{
		return std::abs(lengthSquared() - 1.0f) < 100.0f * FLT_EPSILON;
	}

	/// Indexing operator (const version)
	[[nodiscard]] float operator[](int index) const noexcept
	{
		assert(index < 3);
		return *(&x + index);
	}

	/// Indexing operator (non-const version)
	[[nodiscard]] float& operator[](int index) noexcept
	{
		assert(index < 3);
		return *(&x + index);
	}

	/// Negation operator
	[[nodiscard]] Vec operator-() const noexcept
	{
		return { -x, -y, -z };
	}

	/// Addition assignment operator
	Vec& operator+=(const Vec& vec) noexcept
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return *this;
	}

	/// Subtraction assignment operator
	Vec& operator-=(const Vec& vec) noexcept
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return *this;
	}

	/// Scalar multiplication assignment operator
	Vec& operator*=(float scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		return *this;
	}
};

/// 3D vector alias
using Vec3 = Vec<3>;

/// Dot product
inline [[nodiscard]] float dot(
	const Vec3& vecA,
	const Vec3& vecB) noexcept
{
	return vecA.x * vecB.x + vecA.y * vecB.y + vecA.z * vecB.z;
}

/// Cross product
inline [[nodiscard]] Vec3 cross(
	const Vec3& vecA,
	const Vec3& vecB) noexcept
{
	return {
		vecA.y * vecB.z - vecA.z * vecB.y,
		vecA.z * vecB.x - vecA.x * vecB.z,
		vecA.x * vecB.y - vecA.y * vecB.x };
}

/// Returns the distance, azimuth, and inclination of the vector,
/// the angles are in radians.
inline [[nodiscard]] Vec3 getDistanceAzimuthInclination(const Vec3& vec) noexcept
{
	const float distance = vec.length();
	return distance < FLT_EPSILON ?
		Vec3(0.0f, 0.0f, 0.0f) :
		Vec3(distance, std::atan2(vec.y, vec.x), std::asin(vec.z / distance));
}

/// Sets the distance, azimuth, and inclination to a vector,
/// the angles are in radians.
inline Vec3 setDistanceAzimuthInclination(const Vec3& vec) noexcept
{
	const float distance = vec.x;
	const float cosInclination = std::cos(vec.z);
	return Vec3 {
		distance * std::cos(vec.y) * cosInclination,
		distance * std::sin(vec.y) * cosInclination,
		distance * std::sin(vec.z)};
}

/// Returns the component-wise absolute value of a vector
inline [[nodiscard]] Vec3 abs(const Vec3& vec) noexcept
{
	return { std::abs(vec.x), std::abs(vec.y), std::abs(vec.z) };
}

} // namespace nph
