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
	const Vec2& windowSize) :

	mFov(fov),
	mNearPlane(nearPlane),
	mFarPlane(farPlane)
{
	assert(fov > FLT_EPSILON);
	assert(nearPlane > FLT_EPSILON);
	assert(farPlane > nearPlane);
	setWindowSize(windowSize);
}

void CameraProjection::setWindowSize(const Vec2& size) noexcept
{
	assert(size.x > 0.0f);
	assert(size.y > 0.0f);
	mWindowSize = size;
	update();
}

void CameraProjection::update() noexcept
{
	const float aspectRatio =
		static_cast<float>(mWindowSize.x) /
		static_cast<float>(mWindowSize.y);
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

Vec3 Camera::screenToCameraRay(const Vec2& screenPoint) noexcept
{
	const Vec2& windowSize = mProjection.getWindowSize();

	// Convert the screen point to the normalized device coordinates (NDC)
	const Vec3 ndc(
		(2.0f * screenPoint.x) / windowSize.x - 1.0f,
		1.0f - (2.0f * screenPoint.y) / windowSize.y,
		1.0f);
	const Mat44 cameraProjectionViewInverse =
		(mProjection.getMatrix() * mView.getMatrix()).getInverse();

	return cameraProjectionViewInverse * ndc - mView.getPosition();
}

} // namespace nph
