#pragma once

// Includes
#include <cassert>
#include <cmath>

namespace nph
{

/// 2-dimensional vector
struct Vec2
{
	/// X component, intentionally uninitialized
	float x;

	/// Y component, intentionally uninitialized
	float y;

	/// Default constructor (no initialization)
	Vec2() noexcept = default;

	/// Constructor with components
	constexpr Vec2(float inX, float inY) noexcept :
		x(inX), y(inY)
	{
	}

	/// Sets the vector components
	void set(float inX, float inY) noexcept
	{
		x = inX;
		y = inY;
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
	[[nodiscard]] Vec2 operator-() const noexcept
	{
		return { -x, -y };
	}

	/// Addition assignment operator
	Vec2& operator+=(const Vec2& vec) noexcept
	{
		x += vec.x;
		y += vec.y;
		return *this;
	}

	/// Subtraction assignment operator
	Vec2& operator-=(const Vec2& vec) noexcept
	{
		x -= vec.x;
		y -= vec.y;
		return *this;
	}

	/// Scalar multiplication assignment operator
	Vec2& operator*=(float scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		return *this;
	}
};

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

/// Vector addition operator
inline [[nodiscard]] Vec2 operator+(
	const Vec2& vecA,
	const Vec2& vecB) noexcept
{
	return Vec2(vecA) += vecB;
}

/// Vector subtraction operator
inline [[nodiscard]] Vec2 operator-(
	const Vec2& vecA,
	const Vec2& vecB) noexcept
{
	return Vec2(vecA) -= vecB;
}

/// Scalar multiplication operator
///	\note the vector-scalar operator is
/// intentionally not defined to avoid
/// optimization flaws like 2.0f * vec * 3.0f
inline [[nodiscard]] Vec2 operator*(
	float scalar,
	const Vec2& vec) noexcept
{
	return Vec2(vec) *= scalar;
}

/// Component-wise absolute value of a vector
inline [[nodiscard]] Vec2 abs(const Vec2& vec) noexcept
{
	return { std::abs(vec.x), std::abs(vec.y) };
}

/// Returns a vector rotated by 90 degrees counter-clockwise
inline [[nodiscard]] Vec2 getLeftOrthoVec(const Vec2& vec) noexcept
{
	return { -vec.y, vec.x };
}

/// Returns a vector rotated by 90 degrees clockwise
inline [[nodiscard]] Vec2 getRightOrthoVec(const Vec2& vec) noexcept
{
	return { vec.y, -vec.x };
}

} // namespace nph
