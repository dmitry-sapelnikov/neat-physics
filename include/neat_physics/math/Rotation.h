// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Mat.h"

namespace nph
{

/// Generic rotation template
template <uint16_t D>
class Rotation;

/// 2D rotation (angle + rotation matrix)
template <>
class Rotation<2>
{
public:
	/// Default constructor (no initialization)
	Rotation() noexcept = default;

	/// Constructor from angle in radians
	explicit Rotation(float angleRad) noexcept :
		mAngleRad(angleRad),
		mMat(rotationMat(mAngleRad))
	{
	}

	/// Returns the rotation angle in radians
	[[nodiscard]] float getAngle() const noexcept
	{
		return mAngleRad;
	}

	/// Sets the rotation angle in radians
	void setAngle(float angleRad) noexcept
	{
		mAngleRad = angleRad;
		mMat = rotationMat(mAngleRad);
	}

	/// Returns the rotation matrix
	[[nodiscard]] const Mat22& getMat() const noexcept
	{
		return mMat;
	}

	/// Returns the inverse rotation matrix
	/// (equal to the transposed matrix)
	[[nodiscard]] Mat22 getInverseMat() const noexcept
	{
		return mMat.getTransposed();
	}

private:
	/// Angle in radians
	float mAngleRad;

	/// Rotation matrix
	Mat22 mMat;
};

/// 2D rotation alias
using Rotation2 = Rotation<2>;

} // namespace nph
