// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Mat22.h"
#include "neat_physics/math/Mat33.h"
#include "neat_physics/math/Quat.h"

namespace nph
{

template <uint16_t D>
struct RotationTraits;

/// 2D rotation (angle + rotation matrix)
template <uint16_t D>
class Rotation
{
public:
	using RotationType = typename RotationTraits<D>::Type;
	using RotationMat = Mat<D, D>;

	/// Default constructor (no initialization)
	Rotation() noexcept = default;

	/// Constructor from angle in radians
	explicit Rotation(const RotationType& rotation) noexcept :
		mRotation(rotation),
		mMat(rotationMat(mRotation))
	{
	}

	/// Returns the rotation angle in radians
	[[nodiscard]] float get() const noexcept
	{
		return mRotation;
	}

	/// Sets the rotation
	void set(const RotationType& rotation) noexcept
	{
		mRotation = rotation;
		mMat = rotationMat(mRotation);
	}

	/// Returns the rotation matrix
	[[nodiscard]] const RotationMat& getMat() const noexcept
	{
		return mMat;
	}

	/// Returns the inverse rotation matrix
	/// (equal to the transposed matrix)
	[[nodiscard]] RotationMat getInverseMat() const noexcept
	{
		return mMat.getTransposed();
	}

private:
	/// Angle in radians
	RotationType mRotation;

	/// Rotation matrix
	RotationMat mMat;
};

template <>
struct RotationTraits<2>
{
	using Type = float;
};

template <>
struct RotationTraits<3>
{
	using Type = Quat;
};

} // namespace nph
