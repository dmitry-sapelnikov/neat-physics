// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Mat22.h"
#include "neat_physics/math/Mat33.h"
#include "neat_physics/math/Quat.h"
#include "neat_physics/math/AxisAngle.h"

namespace nph
{

/// Rotation traits
template <uint16_t D, typename T>
struct RotationTraits;

/// 2D rotation (angle + rotation matrix)
template <uint16_t D, typename T = float>
class Rotation
{
public:
	/// Rotation type (angle in radians for 2D, quaternion for 3D)
	using RotationType = typename RotationTraits<D, T>::Type;

	/// Rotation matrix type
	using RotationMat = Mat<D, D, T>;

	/// Default constructor (no initialization)
	Rotation() noexcept = default;

	/// Constructor from angle in radians
	explicit Rotation(const AxisAngle<D, T>& axisAngle) noexcept :
		mRotation(axisAngle),
		mMat(rotationMat(mRotation))
	{
	}

	/// Returns the rotation
	[[nodiscard]] const RotationType& get() const noexcept
	{
		return mRotation;
	}

	/// Sets the rotation
	void set(const RotationType& rotation) noexcept
	{
		mRotation = rotation;
		mMat = rotationMat(mRotation);
	}

	/// Sets the axis-angle rotation
	void setAxisAngle(const AxisAngle<D, T>& axisAngle) noexcept
	{
		mRotation = RotationType(axisAngle);
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

/// 2D rotation traits specialization
template <typename T>
struct RotationTraits<2, T>
{
	/// Rotation representation for 2D (angle in radians)
	using Type = T;
};

/// 3D rotation traits specialization
template <typename T>
struct RotationTraits<3, T>
{
	/// Rotation representation for 3D (quaternion)
	using Type = QuatT<T>;
};

} // namespace nph
