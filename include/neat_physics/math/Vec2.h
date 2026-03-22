// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <limits>
#include "neat_physics/math/Vec.h"

namespace nph
{

/// 2-dimensional vector
template <typename T>
struct Vec<2, T>
{
	/// X component, intentionally uninitialized
	T x;

	/// Y component, intentionally uninitialized
	T y;

	/// Default constructor (no initialization)
	Vec() noexcept = default;

	/// One-element constructor
	explicit constexpr Vec(T value) noexcept :
		x(value), y(value)
	{
	}

	/// Constructor with components
	constexpr Vec(T inX, T inY) noexcept :
		x(inX), y(inY)
	{
	}

	/// Returns the squared length of the vector
	[[nodiscard]] T lengthSquared() const noexcept
	{
		return x * x + y * y;
	}

	/// Returns the length of the vector
	[[nodiscard]] T length() const noexcept
	{
		return std::sqrt(lengthSquared());
	}

	/// Checks if the vector is near zero
	[[nodiscard]] bool isNearZero() const noexcept
	{
		return lengthSquared() < T(100) * std::numeric_limits<T>::epsilon();
	}

	/// Returns a normalized version of the vector, or a zero vector
	/// if the vector length < std::numeric_limits<T>::epsilon()
	[[nodiscard]] Vec getNormalized() const noexcept
	{
		const T len = length();
		if (len < std::numeric_limits<T>::epsilon())
		{
			return { T(0), T(0) };
		}
		const T invLen = T(1) / len;
		return { x * invLen, y * invLen };
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
		assert(index < 2);
		return *(&x + index);
	}

	/// Indexing operator (non-const version)
	[[nodiscard]] T& operator[](int index) noexcept
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
	Vec& operator*=(T scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		return *this;
	}

	/// Vector-vector multiplication assignment operator
	Vec& operator*=(const Vec& vec) noexcept
	{
		x *= vec.x;
		y *= vec.y;
		return *this;
	}
};

/// Dot product of two vectors
template <typename T>
inline [[nodiscard]] float dot(
	const Vec<2, T>& vecA,
	const Vec<2, T>& vecB) noexcept
{
	return vecA.x * vecB.x + vecA.y * vecB.y;
}

/// Cross product of 2 xy vectors
/// \return Scalar z-component of the 3D cross product
template <typename T>
inline [[nodiscard]] float cross(
	const Vec<2, T>& xyA,
	const Vec<2, T>& xyB) noexcept
{
	return xyA.x * xyB.y - xyA.y * xyB.x;
}

/// Cross product of a xy vector and a z-axis value
template <typename T>
inline [[nodiscard]] Vec<2, T> cross(
	const Vec<2, T>& xy,
	T z) noexcept
{
	return { xy.y * z, -xy.x * z };
}

/// Cross product of a z-axis value and a xy vector
template <typename T>
inline [[nodiscard]] Vec<2, T> cross(
	T z,
	const Vec<2, T>& xy) noexcept
{
	return { -xy.y * z, xy.x * z };
}

/// Component-wise absolute value of a vector
template <typename T>
inline [[nodiscard]] Vec<2, T> abs(const Vec<2, T>& vec) noexcept
{
	return { std::abs(vec.x), std::abs(vec.y) };
}

/// Vec2 alias
using Vec2 = Vec<2, float>;

} // namespace nph
