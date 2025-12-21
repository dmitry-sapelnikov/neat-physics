// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

namespace nph
{

/// Generic matrix template
template <uint16_t R, uint16_t C>
struct Mat;

/// Matrix addition operator
template <uint16_t R, uint16_t C>
inline [[nodiscard]] Mat<R, C> operator+(
	const Mat<R, C>& matA,
	const Mat<R, C>& matB) noexcept
{
	return Mat<R, C>(matA) += matB;
}

/// Matrix subtraction operator
template <uint16_t R, uint16_t C>
inline [[nodiscard]] Mat<R, C> operator-(
	const Mat<R, C>& matA,
	const Mat<R, C>& matB) noexcept
{
	return Mat<R, C>(matA) -= matB;
}

} // namespace nph
