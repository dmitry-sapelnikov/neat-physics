// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cassert>

namespace nph
{

/// RGB color
struct Color
{
	/// Red channel
	float r{ 0.0f };

	/// Green channel
	float g{ 0.0f };

	/// Blue channel
	float b{ 0.0f };

	/// Indexing operator (const version)
	const float& operator[](size_t index) const
	{
		assert(index < 3);
		return *(&r + index);
	}

	/// Indexing operator
	float& operator[](size_t index)
	{
		assert(index < 3);
		return *(&r + index);
	}
};

// Creates a rainbow-style color ramp for a value in [0, 1], going from blue to white to red
inline Color getColorRamp(float t)
{
	const float r = std::clamp(4.0f * (t - 0.25f), 0.0f, 1.0f);
	const float g = std::clamp(4.0f * std::min(t, 1.0f - t), 0.0f, 1.0f);
	const float b = std::clamp(4.0f * (0.75f - t), 0.0f, 1.0f);
	return { r, g, b };
}

} // namespace nph
