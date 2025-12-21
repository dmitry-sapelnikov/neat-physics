// Includes
#include "Camera.h"
#include "neat_physics/math/MathFunctions.h"

namespace nph
{

CameraView::CameraView(
	const Vec3& position,
	const Vec3& target,
	const Vec3& up) :

	mPosition(position),
	mTarget(target),
	mUp(up)
{
	assert(!(mPosition - mTarget).isNearZero());
	assert(!mUp.isNearZero());
	update();
}

void CameraView::update() noexcept
{
	mMatrix = Mat44::lookAtMatrix(mPosition, mTarget, mUp);
}

CameraProjection::CameraProjection(
	float fov,
	float nearPlane,
	float farPlane,
	const Point2u& windowSize) :

	mFov(fov),
	mNearPlane(nearPlane),
	mFarPlane(farPlane)
{
	assert(fov > FLOAT_EPSILON);
	assert(nearPlane > FLOAT_EPSILON);
	assert(farPlane > nearPlane);
	setWindowSize(windowSize);
}

void CameraProjection::setWindowSize(const Point2u& size) noexcept
{
	assert(size.x > 0);
	assert(size.y > 0);
	mWindowSize = size;
	update();
}

void CameraProjection::update() noexcept
{
	float aspectRatio = 0.0f;
	if (!mAspectRatio.has_value())
	{
		aspectRatio =
			static_cast<float>(mWindowSize.x) /
			static_cast<float>(mWindowSize.y);
	}
	else
	{
		aspectRatio = *mAspectRatio;
	}

	mMatrix = getMatrix(aspectRatio);
}

Mat44 CameraProjection::getMatrix(float aspectRatio) const noexcept
{
	return Mat44::perspectiveProjectionMatrix(
		toRadians(mFov),
		aspectRatio,
		mNearPlane,
		mFarPlane);
}

// Global functions
Vec3 screenToCameraRay(
	const Point2i& screenPoint,
	const Point2u& windowSize,
	const Vec3& cameraPosition,
	const Mat44& cameraProjectionViewInverse)
{
	assert(windowSize.x != 0);
	assert(windowSize.y != 0);

	// Convert the screen point to the normalized device coordinates (NDC)
	const Vec3 ndc(
		(2.0f * screenPoint.x) / windowSize.x - 1.0f,
		1.0f - (2.0f * screenPoint.y) / windowSize.y,
		1.0f);
	return cameraProjectionViewInverse * ndc - cameraPosition;
}

} // namespace nph
