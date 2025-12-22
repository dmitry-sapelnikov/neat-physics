// OpenGL tutorials and engine (https://github.com/dmitry-sapelnikov/opengl-tutorials)
// SPDX-FileCopyrightText: 2024-2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "MouseCameraController.h"

#include <algorithm>
#include "neat_physics/math/MathFunctions.h"
#include "Camera.h"

namespace nph
{

MouseCameraController::MouseCameraController(
	Camera& camera,
	float rotationSpeed,
	float zoomSpeed,
	float targetMinDistance,
	float targetMaxDistance) :

	mCamera(camera),
	mRotationSpeed(rotationSpeed),
	mZoomSpeed(zoomSpeed),
	mTargetMinDistance(targetMinDistance),
	mTargetMaxDistance(targetMaxDistance)
{
	assert(mRotationSpeed >= 0.0f);
	assert(mZoomSpeed > 0.0f);
	assert(mTargetMinDistance > 0.0f);
	assert(mTargetMaxDistance > mTargetMinDistance);

	updateZoom();

	const CameraView& view = mCamera.getView();
	const Vec3 up = view.getUp();
	const Vec3 right = view.getRight();
	const Vec3 front = cross(up, right).getNormalized();

	mPitchYawBasis.setAxis(0, front);
	mPitchYawBasis.setAxis(1, right);
	mPitchYawBasis.setAxis(2, up);

	Mat44 pitchYawBasisInv = mPitchYawBasis.getInverse();
	const Vec3 directionLocal = pitchYawBasisInv * view.getDirection();
	const Vec3 distanceAzimuthInclination =
		getDistanceAzimuthInclination(directionLocal);

	mYaw = toDegrees(distanceAzimuthInclination.y);
	mPitch = toDegrees(distanceAzimuthInclination.z);
}

void MouseCameraController::setCameraPosition(const Vec3& position)
{
	mCamera.getView().setPosition(position);
	updateZoom();
}

void MouseCameraController::setCameraTarget(const Vec3& target)
{
	mCamera.getView().setTarget(target);
	updateZoom();
}

bool MouseCameraController::update(const MouseInput& mouseInput) noexcept
{
	auto& view = mCamera.getView();

	mCurrentZoom *= std::pow(
		2.0f,
		-static_cast<float>(mouseInput.mouseWheel) * 0.25f * (mZoomSpeed / 100.0f));

	mCurrentZoom = std::clamp(
		mCurrentZoom,
		mTargetMinDistance,
		mTargetMaxDistance);

	// Zoom
	if (mPrevZoom != mCurrentZoom)
	{
		view.setPosition(view.getTarget() - mCurrentZoom * view.getDirection());
		mPrevZoom = mCurrentZoom;
		return true;
	}

	// Rotation
	if (mouseInput.leftMouseDown && mRotationSpeed > 0.0f)
	{
		if (!mRotating)
		{
			mMouseStart = mouseInput.position;
			mRotating = true;
		}
		else
		{
			const Vec2 mouseDelta = mouseInput.position - mMouseStart;

			mYaw += mouseDelta.x * mRotationSpeed;

			// The pitch is inverted because the mouse Y-axis is inverted
			mPitch -= mouseDelta.y * mRotationSpeed;

			// Clamp the pitch to prevent the camera from flipping
			mPitch = std::clamp(mPitch, -89.0f, 89.0f);

			const Vec3 localDir = setDistanceAzimuthInclination(
				{1.0f, toRadians(mYaw), toRadians(mPitch)});

			const Vec3 direction = mPitchYawBasis * localDir;
			view.setPosition(view.getTarget() - mCurrentZoom * direction);

			mMouseStart = mouseInput.position;
			return true;
		}
	}
	else
	{
		mRotating = false;
	}

	// Translation
	if (mouseInput.middleMouseDown)
	{
		if (!mTranslating)
		{
			mMouseStart = mouseInput.position;
			mTranslating = true;
			mInitialTarget = view.getTarget();
			mInitialPosition = view.getPosition();

			mDragStart = mCamera.screenToCameraRay(mMouseStart);
		}
		else
		{
			const Vec3 currentDrag = mCamera.screenToCameraRay(mouseInput.position);
			const Vec3 distance = mInitialTarget - mInitialPosition;
			const float proj = dot(distance, currentDrag);
			assert(proj > 0.0f);
			const Vec3 deltaWorld = (distance.lengthSquared() / proj) * (mDragStart - currentDrag);
			view.setTarget(mInitialTarget + deltaWorld);
			view.setPosition(mInitialPosition + deltaWorld);
		}
		return true;
	}
	else
	{
		mTranslating = false;
	}

	return false;
}

void MouseCameraController::updateZoom()
{
	const auto& view = mCamera.getView();
	mPrevZoom = std::clamp(
		(view.getPosition() - view.getTarget()).length(),
		mTargetMinDistance,
		mTargetMaxDistance);

	mCurrentZoom = mPrevZoom;
}

// End of the namespace nph
}
