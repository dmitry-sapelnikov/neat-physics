// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <algorithm>
#include <limits>
#include <cmath>
#include "neat_physics/math/Vec.h"

namespace nph
{

/// Represents a 3D vector
template <typename T>
struct Vec<3, T>
{
	/// X component, intentionally uninitialized
	T x;

	/// Y component, intentionally uninitialized
	T y;

	/// Z component, intentionally uninitialized
	T z;

	/// Default constructor (no initialization)
	Vec() noexcept = default;

	/// One-element constructor
	explicit constexpr Vec(T value) noexcept :
		x(value), y(value), z(value)
	{
	}

	/// Constructor with components
	constexpr Vec(T inX, T inY, T inZ) noexcept :
		x(inX), y(inY), z(inZ)
	{
	}

	/// Returns the squared length of the vector
	[[nodiscard]] T lengthSquared() const noexcept
	{
		return x * x + y * y + z * z;
	}

	/// Returns the length of the vector
	[[nodiscard]] T length() const noexcept
	{
		return std::sqrt(lengthSquared());
	}

	/// Returns a normalized version of the vector, or a zero vector
	/// if the vector length < FLT_EPSILON
	[[nodiscard]] Vec getNormalized() const noexcept
	{
		const T len = length();
		if (len < std::numeric_limits<T>::epsilon())
		{
			return { T(0), T(0), T(0) };
		}
		const T invLen = T(1) / len;
		return { x * invLen, y * invLen, z * invLen };
	}

	/// Checks if the vector is near zero
	[[nodiscard]] bool isNearZero() const noexcept
	{
		return
			lengthSquared() < 
			T(100) * std::numeric_limits<T>::epsilon();
	}

	/// Checks if the vector is normalized
	[[nodiscard]] bool isNormalized() const noexcept
	{
		return
			std::abs(lengthSquared() - T(1)) <
			T(100) * std::numeric_limits<T>::epsilon();
	}

	/// Indexing operator (const version)
	[[nodiscard]] T operator[](int index) const noexcept
	{
		assert(index < 3);
		return *(&x + index);
	}

	/// Indexing operator (non-const version)
	[[nodiscard]] T& operator[](int index) noexcept
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
	Vec& operator*=(T scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		return *this;
	}

	/// Component-wise multiplication assignment operator
	Vec& operator*=(const Vec& vec) noexcept
	{
		x *= vec.x;
		y *= vec.y;
		z *= vec.z;
		return *this;
	}
};

/// Dot product
template <typename T>
inline [[nodiscard]] T dot(
	const Vec<3, T>& vecA,
	const Vec<3, T>& vecB) noexcept
{
	return vecA.x * vecB.x + vecA.y * vecB.y + vecA.z * vecB.z;
}

/// Cross product
template <typename T>
inline [[nodiscard]] Vec<3, T> cross(
	const Vec<3, T>& vecA,
	const Vec<3, T>& vecB) noexcept
{
	return {
		vecA.y * vecB.z - vecA.z * vecB.y,
		vecA.z * vecB.x - vecA.x * vecB.z,
		vecA.x * vecB.y - vecA.y * vecB.x };
}

/// Returns the component-wise absolute value of a vector
template <typename T>
inline [[nodiscard]] Vec<3, T> abs(const Vec<3, T>& vec) noexcept
{
	return { std::abs(vec.x), std::abs(vec.y), std::abs(vec.z) };
}

/// Returns the index of the axis with the maximum absolute value
template <typename T>
inline [[nodiscard]] uint32_t getMaxAbsAxis(const Vec<3, T>& vec)  noexcept
{
	const float nx = std::abs(vec[0]);
	const float ny = std::abs(vec[1]);
	const float nz = std::abs(vec[2]);
	return nx >= std::max(ny, nz) ?
		0 :
		ny >= std::max(nx, nz) ? 1 : 2;
}

/// Returns a normalized vector that is perpendicular to the input vector
template <typename T>
inline Vec<3, T> getNormalizedPerpendicular(const Vec<3, T>& vec)
{
	if (std::abs(vec.x) > std::abs(vec.y))
	{
		const float len = sqrt(vec.x * vec.x + vec.z * vec.z);
		return (T(1) / len) * Vec<3, T>(vec.z, 0.0f, -vec.x);
	}
	else
	{
		const float len = std::sqrt(vec.y * vec.y + vec.z * vec.z);
		return (T(1) / len) * Vec<3, T>(0.0f, vec.z, -vec.y);
	}
}

/// Returns the distance, azimuth, and inclination of the vector,
/// the angles are in radians
template <typename T>
inline [[nodiscard]] Vec<3, T> getDistanceAzimuthInclination(const Vec<3, T>& vec) noexcept
{
	const float distance = vec.length();
	return distance < std::numeric_limits<float>::epsilon() ?
		Vec<3, T>(T(0), T(0), T(0)) :
		Vec<3, T>(distance, std::atan2(vec.y, vec.x), std::asin(vec.z / distance));
}

/// Sets the distance, azimuth, and inclination to a vector,
/// the angles are in radians.
template <typename T>
inline Vec<3, T> setDistanceAzimuthInclination(const Vec<3, T>& vec) noexcept
{
	const float distance = vec.x;
	const float cosInclination = std::cos(vec.z);
	return {
		distance * std::cos(vec.y) * cosInclination,
		distance * std::sin(vec.y) * cosInclination,
		distance * std::sin(vec.z) };
}

/// 3D vector alias
using Vec3 = Vec<3, float>;

} // namespace nph
