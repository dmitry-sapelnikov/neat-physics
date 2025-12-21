#pragma once

// Includes
#include <optional>
#include "neat_physics/math/Mat44.h"
#include "neat_physics/math/Point2.h"

namespace nph
{

/// Camera view
class CameraView
{
public:
	/// Constructor
	CameraView(
		const Vec3& position,
		const Vec3& target,
		const Vec3& up);

	/// Returns the view matrix
	const Mat44& getMatrix() const noexcept
	{
		return mMatrix;
	}

	/// Returns the position of the camera
	const Vec3& getPosition() const noexcept
	{
		return mPosition;
	}

	/// Sets the position of the camera
	void setPosition(const Vec3& position) noexcept
	{
		mPosition = position;
		update();
	}

	/// Returns the target of the camera
	const Vec3& getTarget() const noexcept
	{
		return mTarget;
	}

	/// Sets the target of the camera
	void setTarget(const Vec3& target) noexcept
	{
		mTarget = target;
		update();
	}

	/// Returns the up vector of the camera
	const Vec3& getUp() const noexcept
	{
		return mUp;
	}

	/// Sets the up vector of the camera
	void setUp(const Vec3& up) noexcept
	{
		mUp = up;
		update();
	}

	/// Returns the direction vector of the camera
	Vec3 getDirection() const noexcept
	{
		return (getTarget() - getPosition()).getNormalized();
	}

	/// Returns the right vector of the camera
	Vec3 getRight() const noexcept
	{
		return cross(getDirection(), getUp()).getNormalized();
	}

private:
	/// Updates the view matrix
	void update() noexcept;

	/// The view matrix
	Mat44 mMatrix;

	/// The position of the camera
	Vec3 mPosition;

	/// The target of the camera
	Vec3 mTarget;

	/// The up vector of the camera
	Vec3 mUp;
};

/// Implementation of the camera projection
class CameraProjection
{
public:
	/// Constructor
	CameraProjection(
		float fov,
		float nearPlane,
		float farPlane,
		const Point2u& windowSize);

	/// Returns the projection matrix
	const Mat44& getMatrix() const noexcept
	{
		return mMatrix;
	}

	/// Returns the projection matrix with the given aspect ratio
	Mat44 getMatrix(float aspectRatio) const noexcept;

	/// Return the field of view, in degrees
	float getFov() const noexcept
	{
		return mFov;
	}

	/// Sets the field of view, in degrees
	void setFov(float fov) noexcept
	{
		mFov = fov;
		update();
	}

	/// Returns the near plane
	float getNearPlane() const noexcept
	{
		return mNearPlane;
	}

	/// Sets the near plane
	void setNearPlane(float nearPlane) noexcept
	{
		mNearPlane = nearPlane;
		update();
	}

	/// Returns the far plane
	float getFarPlane() const noexcept
	{
		return mFarPlane;
	}

	/// Sets the far plane
	void setFarPlane(float farPlane) noexcept
	{
		mFarPlane = farPlane;
		update();
	}

	/// Returns the window size
	[[nodiscard]] const Point2u& getWindowSize() const noexcept
	{
		return mWindowSize;
	}

	/// Sets the window size
	void setWindowSize(const Point2u& size) noexcept;

private:
	/// Updates the projection matrix
	void update() noexcept;

	/// The field of view, in degrees
	float mFov;

	/// The near plane
	float mNearPlane;

	/// The far plane
	float mFarPlane;

	/// Window size
	Point2u mWindowSize;

	/// The aspect ratio, i.e. width / height
	std::optional<float> mAspectRatio;

	/// The projection matrix
	Mat44 mMatrix;
};

/// Implementation of the camera
class Camera
{
public:
	/// Constructor
	Camera(
		const Vec3& position,
		const Vec3& target,
		const Vec3& up,

		float fov,
		float nearPlane,
		float farPlane,
		const Point2u& windowSize) noexcept :

		mView(position, target, up),
		mProjection(fov, nearPlane, farPlane, windowSize)
	{
	}

	/// Returns the view (const)
	const CameraView& getView() const noexcept
	{
		return mView;
	}

	/// Returns the view (non-const)
	CameraView& getView() noexcept
	{
		return mView;
	}

	/// Returns the projection (const)
	const CameraProjection& getProjection() const noexcept
	{
		return mProjection;
	}

	/// Returns the projection (non-const)
	CameraProjection& getProjection() noexcept
	{
		return mProjection;
	}

private:
	/// The view
	CameraView mView;

	/// The projection
	CameraProjection mProjection;
};

/// Converts a screen point to a camera ray in world space
Vec3 screenToCameraRay(
	const Point2i& screenPoint,
	const Point2u& windowSize,
	const Vec3& cameraPosition,
	const Mat44& cameraProjectionViewInverse);

} // namespace nph
