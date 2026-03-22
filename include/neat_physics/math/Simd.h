// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

#include "neat_physics/core/Check.h"

namespace nph::simd
{

/// Enclose the vectorclass library into 
/// the nph::simd namespace to avoid polluting the global namespace
#include <vectorclass/vectorclass.h>

/// A simple wrapper around a single float value to provide
/// a consistent interface with the other vectorclass library types
class Vec1f
{
public:
	/// Default constructor
	/// Does not initialize the value
	Vec1f() = default;

	/// Constructor from a single float value
	Vec1f(float value) :
		v(value)
	{
	}

	/// Returns the number of elements in the vector (1 for Vec1f)
	static constexpr int size()
	{
		return 1;
	}

	/// Inserts a float value at the specified index (0 for Vec1f)
	void insert([[maybe_unused]] int index, float value)
	{
		NPH_ASSERT(index == 0);
		v = value;
	}

	/// Implicit conversion to float
	operator float() const
	{
		return v;
	}

	/// Unary negation operator
	Vec1f operator-() const
	{
		return Vec1f{ -v };
	}

	/// Subscript operator to access the value (0 for Vec1f)
	float operator[]([[maybe_unused]] int index) const
	{
		NPH_ASSERT(index == 0);
		return v;
	}

	/// Addition assignment
	Vec1f& operator+=(float other)
	{
		v += other;
		return *this;
	}

	/// Subtraction assignment
	Vec1f& operator-=(float other)
	{
		v -= other;
		return *this;
	}

	/// Multiplication assignment
	Vec1f& operator*=(float other)
	{
		v *= other;
		return *this;
	}

	/// Addition assignment with another Vec1f
	Vec1f& operator+=(const Vec1f& other)
	{
		v += other.v;
		return *this;
	}

	/// Subtraction assignment with another Vec1f
	Vec1f& operator-=(const Vec1f& other)
	{
		v -= other.v;
		return *this;
	}

	/// Multiplication assignment with another Vec1f
	Vec1f& operator*=(const Vec1f& other)
	{
		v *= other.v;
		return *this;
	}

	/// Multiplication operator for Vec1f and float
	friend Vec1f operator*(const Vec1f& a, float b)
	{
		return { a.v * b };
	}

	/// Multiplication operator for float and Vec1f
	friend Vec1f operator*(float a, const Vec1f& b)
	{
		return { a * b.v };
	}

	/// Multiplication operator for Vec1f
	friend Vec1f operator*(const Vec1f& a, const Vec1f& b)
	{
		return { a.v * b.v };
	}

	/// Addition operator for Vec1f
	friend Vec1f operator+(const Vec1f& a, const Vec1f& b)
	{
		return { a.v + b.v };
	}

	/// Subtraction operator for Vec1f
	friend Vec1f operator-(const Vec1f& a, const Vec1f& b)
	{
		return { a.v - b.v };
	}

private:
	// The stored float value
	float v;
};

} // namespace nph::simd
