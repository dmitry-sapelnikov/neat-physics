// OpenGL tutorials and engine (https://github.com/dmitry-sapelnikov/opengl-tutorials)
// SPDX-FileCopyrightText: 2024-2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Point2.h"
#include "neat_physics/math/Mat44.h"

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
		float mouseSpeed,
		float zoomSpeed,
		float targetMinDistance,
		float targetMaxDistance);

	/// Returns the camera
	Camera& getCamera() noexcept
	{
		return mCamera;
	}

	/// Updates the camera
	void updateCamera(
		bool leftMouse,
		bool midMouse,
		int mouseWheel) noexcept;

private:
	/// The camera
	Camera& mCamera;

	/// Rotation speed
	float mMouseSpeed;

	/// Zoom speed
	float mZoomSpeed;

	/// Camera position
	Vec3 mPosition;

	/// Camera target
	Vec3 mInitialTarget;

	/// Camera position
	Vec3 mInitialPosition;

	Point2u mWindowSize;

	Mat44 mProjectionViewInv;

	Vec3 mDragStart;

	/// The camera's yaw, in degrees
	float mYaw = 0.0F;

	/// The camera's pitch, in degrees
	float mPitch = 0.0F;

	/// The basis for the pitch and yaw
	Mat44 mPitchYawBasis = Mat44::identity();

	/// Min allowed distance to the camera target
	float mTargetMinDistance = 0.1f;

	/// Max allowed distance from the camera target
	float mTargetMaxDistance = 10000.0f;

	/// Rotation process flag
	bool mRotating = false;

	/// Translation process flag
	bool mTranslating = false;

	/// Translation start point in the screen coordinates
	Point2i mMouseStart;

	float mPrevZoom = 0.0f;

	/// Current zoom value
	float mCurrentZoom = 0.0f;

	/// Mouse position in the screen coordinates
	Point2i mMousePosition;

	bool mLeftMouseButton{ false };
	bool mMiddleMouseButton{ false };
};

// End of the namespace nph
}
