#pragma once

#include <vector>
#include "neat_physics/Body.h"

namespace nph
{

/// Physics world
class World
{
public:
	/// Body container type
	using BodiesType = std::vector<Body>;

	/// Constructor
	/// \param maxBodies Maximum number of bodies in the world; must be > 0
	explicit World(uint32_t maxBodies);

	/// Get the bodies in the world
	[[nodiscard]] const BodiesType& getBodies() const noexcept
	{
		return mBodies;
	}

	/// Add a body to the world
	/// \return the added body or nullptr if the body could not be added
	/// (e.g., maximum number of bodies reached)
	Body* addBody(
		const Vec2& size,
		float mass,
		float friction,
		const Vec2& position = {0.0f, 0.0f},
		float rotationRad = 0.0f);

	/// Remove all entities from the world
	void clear() noexcept;

private:
	/// Bodies in the world
	BodiesType mBodies;
};

} // namespace nph
