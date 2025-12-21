// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Mat.h"
#include "neat_physics/math/Vec3.h"

namespace nph
{

/// 3x3 matrix, column major
template <>
struct Mat<3, 3>
{
	/// Column #1, intentionally uninitialized
	Vec3 col1;

	/// Column #2, intentionally uninitialized
	Vec3 col2;

	/// Column #3, intentionally uninitialized
	Vec3 col3;

	/// Default constructor (no initialization)
	Mat() noexcept = default;

	/// Constructor with columns
	constexpr Mat(
		const Vec3& inCol1,
		const Vec3& inCol2,
		const Vec3& inCol3) noexcept :

		col1(inCol1),
		col2(inCol2),
		col3(inCol3)
	{
	}

	/// Returns the transposed matrix
	[[nodiscard]] Mat getTransposed() const noexcept
	{
		return { 
			{ col1.x, col2.x, col3.x },
			{ col1.y, col2.y, col3.y },
			{ col1.z, col2.z, col3.z } };
	}

	/// Column access operator (const version)
	[[nodiscard]] const Vec3& operator[](int index) const noexcept
	{
		assert(index < 3);
		return *(&col1 + index);
	}

	/// Column access operator (non-const version)
	[[nodiscard]] Vec3& operator[](int index) noexcept
	{
		assert(index < 3);
		return *(&col1 + index);
	}

	/// Addition assignment operator
	Mat& operator+=(const Mat& other) noexcept
	{
		col1 += other.col1;
		col2 += other.col2;
		col3 += other.col3;
		return *this;
	}

	/// Subtraction assignment operator
	Mat& operator-=(const Mat& other) noexcept
	{
		col1 -= other.col1;
		col2 -= other.col2;
		col3 -= other.col3;
		return *this;
	}
};

/// 3x3 matrix alias
using Mat33 = Mat<3, 3>;

/// Matrix-vector multiplication operator
inline [[nodiscard]] Vec3 operator*(
	const Mat33& mat,
	const Vec3& vec) noexcept
{
	// This works faster than 3 dot() calls
	return {
		mat.col1.x * vec.x + mat.col2.x * vec.y + mat.col3.x * vec.z,
		mat.col1.y * vec.x + mat.col2.y * vec.y + mat.col3.y * vec.z,
		mat.col1.z * vec.x + mat.col2.z * vec.y + mat.col3.z * vec.z
	};
}

/// Matrix-matrix multiplication operator
inline [[nodiscard]] Mat33 operator*(
	const Mat33& matA,
	const Mat33& matB) noexcept
{
	return { matA * matB.col1, matA * matB.col2, matA * matB.col3 };
}

/// Component-wise absolute value of a matrix
inline [[nodiscard]] Mat33 abs(const Mat33& mat) noexcept
{
	return { abs(mat.col1), abs(mat.col2), abs(mat.col3) };
}

} // namespace nph
