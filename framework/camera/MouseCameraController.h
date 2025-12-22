// OpenGL tutorials and engine (https://github.com/dmitry-sapelnikov/opengl-tutorials)
// SPDX-FileCopyrightText: 2024-2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec2.h"
#include "neat_physics/math/Mat44.h"
#include "../MouseInput.h"

namespace nph
{

class Camera;

/// Implementation of the mouse camera controller
class MouseCameraController
{
public:
	/// Constructor
	MouseCameraController(
		Camera& camera,
		float rotationSpeed,
		float zoomSpeed,
		float targetMinDistance,
		float targetMaxDistance);

	/// Returns the camera
	const Camera& getCamera() noexcept
	{
		return mCamera;
	}

	/// Updates the camera
	bool update(const MouseInput& mouseInput) noexcept;

	/// Sets the camera position
	void setCameraPosition(const Vec3& position);

	/// Sets the camera target
	void setCameraTarget(const Vec3& target);

	/// Sets the window size
	void setWindowSize(const Vec2& size);

private:
	/// Updates the zoom level
	void updateZoom();

	/// The camera
	Camera& mCamera;

	/// Rotation speed
	float mRotationSpeed;

	/// Zoom speed
	float mZoomSpeed;

	/// Camera position
	Vec3 mPosition;

	/// Camera target
	Vec3 mInitialTarget;

	/// Camera position
	Vec3 mInitialPosition;

	Vec3 mDragStart;

	/// The camera's yaw, in degrees
	float mYaw = 0.0F;

	/// The camera's pitch, in degrees
	float mPitch = 0.0F;

	/// The basis for the pitch and yaw
	Mat44 mPitchYawBasis = Mat44::identity();

	/// Min allowed distance to the camera target
	float mTargetMinDistance;

	/// Max allowed distance from the camera target
	float mTargetMaxDistance;

	/// Rotation process flag
	bool mRotating = false;

	/// Translation process flag
	bool mTranslating = false;

	/// Translation start point in the screen coordinates
	Vec2 mMouseStart;

	float mPrevZoom = 0.0f;

	/// Current zoom value
	float mCurrentZoom = 0.0f;
};

// End of the namespace nph
}
