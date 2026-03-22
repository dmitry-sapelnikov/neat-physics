// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <cassert>
#include <chrono>
#include <random>
#include <iostream>
#include <neat_physics/core/Ecs.h>
#include <neat_physics/math/Simd.h>
#include <neat_physics/math/Vec3.h>
#include <neat_physics/math/Mat33.h>
#include <neat_physics/math/Quat.h>
#include <neat_physics/math/AxisAngle.h>
#include <neat_physics/body/Inertia.h>

namespace nph
{

bool operator==(const Vec3& a, const Vec3& b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool operator!=(const Vec3& a, const Vec3& b)
{
	return !(a == b);
}

bool operator==(const Quat& a, const Quat& b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

bool operator!=(const Quat& a, const Quat& b)
{
	return !(a == b);
}

struct TestBody
{
	Vec3 position;
	Vec3 linearVelocity;

	Quat rotation;
	Vec3 angularVelocity;
};

template<typename FloatType>
__forceinline void rotationIntegration(
	FloatType& qx,
	FloatType& qy,
	FloatType& qz,
	FloatType& qw,
	FloatType wx,
	FloatType wy,
	FloatType wz,
	float timeStep)
{
	timeStep *= 0.5f;
	FloatType dx = timeStep * (wx * qw + wy * qz - wz * qy);
	FloatType dy = timeStep * (-wx * qz + wy * qw + wz * qx);
	FloatType dz = timeStep * (wx * qy - wy * qx + wz * qw);
	FloatType dw = timeStep * (-wx * qx - wy * qy - wz * qz);

	qx += dx;
	qy += dy;
	qz += dz;
	qw += dw;

	// Normalize the quaternion
	FloatType invNorm = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
	qx *= invNorm;
	qy *= invNorm;
	qz *= invNorm;
	qw *= invNorm;
}

template <typename FloatType>
struct BodyArrays
{
	FloatType* vX;
	FloatType* vY;
	FloatType* vZ;
	FloatType* pX;
	FloatType* pY;
	FloatType* pZ;

	FloatType* aX;
	FloatType* aY;
	FloatType* aZ;
	FloatType* qX;
	FloatType* qY;
	FloatType* qZ;
	FloatType* qW;

	std::vector<FloatType, AlignedAllocator<FloatType, 64>> pool;
	size_t size = 0;

	void resize(size_t elements)
	{
		if constexpr (std::is_same_v<FloatType, float>)
		{
			size = elements;
		}
		else
		{
			size = (elements + FloatType::size() - 1) / FloatType::size();
		}

		pool.resize(13 * size);
		vX = pool.data();
		vY = pool.data() + size;
		vZ = pool.data() + 2 * size;
		
		pX = pool.data() + 3 * size;
		pY = pool.data() + 4 * size;
		pZ = pool.data() + 5 * size;

		aX = pool.data() + 6 * size;
		aY = pool.data() + 7 * size;
		aZ = pool.data() + 8 * size;

		qX = pool.data() + 9 * size;
		qY = pool.data() + 10 * size;
		qZ = pool.data() + 11 * size;
		qW = pool.data() + 12 * size;
	}
};

// An idea of SoAoS:
// Instead of having separate arrays for each component,
// we have arrays of blocks, where each block contains 4 components of the same type
// I try to align it with cache line size (64 bytes) to improve cache performance

template <typename FloatType, size_t D>
struct Block
{
	FloatType rows[D];

	const FloatType& operator[](size_t index) const
	{
		return rows[index];
	}

	FloatType& operator[](size_t index)
	{
		return rows[index];
	}
};

template <typename FloatType>
struct BodyArrays2
{
	using Block2T = Block<FloatType, 2>;

	Block2T* pXY;
	Block2T* pZvX;
	Block2T* vYZ;
	
	Block2T* qXY;
	Block2T* qZW;

	Block2T* aXY;
	FloatType* aZ;

	std::vector<FloatType, AlignedAllocator<FloatType, 64>> pool;
	size_t size = 0;

	void resize(size_t elements)
	{
		size = (elements + FloatType::size() - 1) / FloatType::size();
		pool.resize(13 * size);
		pXY = reinterpret_cast<Block2T*>(pool.data());
		pZvX = reinterpret_cast<Block2T*>(pool.data() + size * 2);
		vYZ = reinterpret_cast<Block2T*>(pool.data() + size * 4);
		qXY = reinterpret_cast<Block2T*>(pool.data() + size * 6);
		qZW = reinterpret_cast<Block2T*>(pool.data() + size * 8);
		aXY = reinterpret_cast<Block2T*>(pool.data() + size * 10);
		aZ = pool.data() + size * 12;
	}
};

template <typename FloatType>
struct BodyArrays4
{
	using Block4T = Block<FloatType, 4>;

	Block4T* pXYZvX;
	Block4T* vYZaXY;
	FloatType* aZ;
	Block4T* q;
	std::vector<FloatType, AlignedAllocator<FloatType, 64>> pool;
	size_t size = 0;

	void resize(size_t elements)
	{
		size = (elements + FloatType::size() - 1) / FloatType::size();
		pool.resize(13 * size);

		pXYZvX = reinterpret_cast<Block4T*>(pool.data());
		vYZaXY = reinterpret_cast<Block4T*>(pool.data() + size * 4);
		q = reinterpret_cast<Block4T*>(pool.data() + size * 8);
		aZ = pool.data() + size * 12;
	}
};

template <typename FloatType>
struct BodyArraysNatural
{
	using Block3T = Block<FloatType, 3>;
	using Block4T = Block<FloatType, 4>;

	Block3T* p;

	Block3T* v;

	Block3T* a;

	Block4T* q;

	std::vector<FloatType, AlignedAllocator<FloatType, 64>> pool;
	size_t size = 0;

	void resize(size_t elements)
	{
		size = (elements + FloatType::size() - 1) / FloatType::size();
		pool.resize(13 * size);

		p = reinterpret_cast<Block3T*>(pool.data());
		v = reinterpret_cast<Block3T*>(pool.data() + size * 3);
		a = reinterpret_cast<Block3T*>(pool.data() + size * 6);
		q = reinterpret_cast<Block4T*>(pool.data() + size * 9);
	}
};

template <typename FloatType>
BodyArrays<FloatType> fillSoA(const std::vector<TestBody>& bodies)
{
	BodyArrays<FloatType> result;
	result.resize(bodies.size());
	for (size_t i = 0; i < bodies.size(); ++i)
	{
		size_t d = i / FloatType::size();
		int m = i % FloatType::size();

		result.pX[d].insert(m, bodies[i].position.x);
		result.pY[d].insert(m, bodies[i].position.y);
		result.pZ[d].insert(m, bodies[i].position.z);
		result.vX[d].insert(m, bodies[i].linearVelocity.x);
		result.vY[d].insert(m, bodies[i].linearVelocity.y);
		result.vZ[d].insert(m, bodies[i].linearVelocity.z);

		const auto& body = bodies[i];
		const Vec3& a = body.angularVelocity;
		const Quat& q = body.rotation;

		result.aX[d].insert(m, a.x);
		result.aY[d].insert(m, a.y);
		result.aZ[d].insert(m, a.z);

		result.qX[d].insert(m, q.x);
		result.qY[d].insert(m, q.y);
		result.qZ[d].insert(m, q.z);
		result.qW[d].insert(m, q.w);
	}
	return result;
}

template <typename FloatType>
BodyArrays2<FloatType> fillSoAoS2(const std::vector<TestBody>& bodies)
{
	BodyArrays2<FloatType> result;
	result.resize(bodies.size());
	for (size_t i = 0; i < bodies.size(); ++i)
	{
		size_t d = i / FloatType::size();
		int m = i % FloatType::size();

		const auto& body = bodies[i];

		auto& pXY = result.pXY[d];
		pXY[0].insert(m, body.position.x);
		pXY[1].insert(m, body.position.y);

		auto& pZvX = result.pZvX[d];
		pZvX[0].insert(m, body.position.z);
		pZvX[1].insert(m, body.linearVelocity.x);

		auto& vYZ = result.vYZ[d];
		vYZ[0].insert(m, body.linearVelocity.y);
		vYZ[1].insert(m, body.linearVelocity.z);

		auto& qXY = result.qXY[d];
		qXY[0].insert(m, body.rotation.x);
		qXY[1].insert(m, body.rotation.y);

		auto& qZW = result.qZW[d];
		qZW[0].insert(m, body.rotation.z);
		qZW[1].insert(m, body.rotation.w);

		auto& aXY = result.aXY[d];
		aXY[0].insert(m, body.angularVelocity.x);
		aXY[1].insert(m, body.angularVelocity.y);

		result.aZ[d].insert(m, body.angularVelocity.z);
	}
	return result;
}

template <typename FloatType>
BodyArrays4<FloatType> fillSoAoS4(const std::vector<TestBody>& bodies)
{
	BodyArrays4<FloatType> result;
	result.resize(bodies.size());
	for (size_t i = 0; i < bodies.size(); ++i)
	{
		size_t d = i / FloatType::size();
		int m = i % FloatType::size();

		const auto& body = bodies[i];

		auto& q = result.q[d];
		q[0].insert(m, body.rotation.x);
		q[1].insert(m, body.rotation.y);
		q[2].insert(m, body.rotation.z);
		q[3].insert(m, body.rotation.w);

		auto& pXYZvX = result.pXYZvX[d];
		pXYZvX[0].insert(m, body.position.x);
		pXYZvX[1].insert(m, body.position.y);
		pXYZvX[2].insert(m, body.position.z);
		pXYZvX[3].insert(m, body.linearVelocity.x);

		auto& vYZaXY = result.vYZaXY[d];
		vYZaXY[0].insert(m, body.linearVelocity.y);
		vYZaXY[1].insert(m, body.linearVelocity.z);
		vYZaXY[2].insert(m, body.angularVelocity.x);
		vYZaXY[3].insert(m, body.angularVelocity.y);

		result.aZ[d].insert(m, body.angularVelocity.z);
	}
	return result;
}

template <typename FloatType>
BodyArraysNatural<FloatType> fillSoAoSNatural(const std::vector<TestBody>& bodies)
{
	BodyArraysNatural<FloatType> result;
	result.resize(bodies.size());
	for (size_t i = 0; i < bodies.size(); ++i)
	{
		size_t d = i / FloatType::size();
		int m = i % FloatType::size();

		const auto& body = bodies[i];
		auto& p = result.p[d];
		p[0].insert(m, body.position.x);
		p[1].insert(m, body.position.y);
		p[2].insert(m, body.position.z);

		auto& v = result.v[d];
		v[0].insert(m, body.linearVelocity.x);
		v[1].insert(m, body.linearVelocity.y);
		v[2].insert(m, body.linearVelocity.z);

		auto& q = result.q[d];
		q[0].insert(m, body.rotation.x);
		q[1].insert(m, body.rotation.y);
		q[2].insert(m, body.rotation.z);
		q[3].insert(m, body.rotation.w);

		auto& a = result.a[d];
		a[0].insert(m, body.angularVelocity.x);
		a[1].insert(m, body.angularVelocity.y);
		a[2].insert(m, body.angularVelocity.z);
	}
	return result;
}

void testBodyArray(
	std::vector<TestBody>& bodies,
	size_t iterations,
	float timeStep,
	std::chrono::duration<double, std::milli>& refDuration)
{
	{
		auto tic = std::chrono::high_resolution_clock::now();
		for (size_t iter = 0; iter < iterations; ++iter)
		{
			for (auto& body : bodies)
			{
				body.position += timeStep * body.linearVelocity;
				rotationIntegration(
					body.rotation.x,
					body.rotation.y,
					body.rotation.z,
					body.rotation.w,
					body.angularVelocity.x,
					body.angularVelocity.y,
					body.angularVelocity.z,
					timeStep);
			}
		}
		auto toc = std::chrono::high_resolution_clock::now();
		refDuration = toc - tic;
		std::cout << "AoS: " << refDuration.count() << " ms" << std::endl;
	}
}

template <typename FloatType>
void testBodyArray(
	BodyArrays<FloatType>& arrays,
	size_t iterations,
	float timeStep,
	const std::string& instructionSetName,
	const std::chrono::duration<double, std::milli>& refDuration)
{
	auto tic = std::chrono::high_resolution_clock::now();
	for (size_t iter = 0; iter < iterations; ++iter)
	{
		for (size_t i = 0; i < arrays.size; ++i)
		{
			arrays.pX[i] += arrays.vX[i] * timeStep;
			arrays.pY[i] += arrays.vY[i] * timeStep;
			arrays.pZ[i] += arrays.vZ[i] * timeStep;

			rotationIntegration(
					arrays.qX[i],
					arrays.qY[i],
					arrays.qZ[i],
					arrays.qW[i],
					arrays.aX[i],
					arrays.aY[i],
					arrays.aZ[i],
					timeStep);
		}
	}
	auto toc = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = toc - tic;
	std::cout << "1-element SoA with " << instructionSetName << ": " << duration.count() << " ms ";
	std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
}

template <typename FloatType>
void testBodyArray(
	BodyArrays2<FloatType>& arrays,
	size_t iterations,
	float timeStep,
	const std::string& instructionSetName,
	const std::chrono::duration<double, std::milli>& refDuration)
{
	auto tic = std::chrono::high_resolution_clock::now();
	for (size_t iter = 0; iter < iterations; ++iter)
	{
		for (size_t i = 0; i < arrays.size; ++i)
		{
			auto& pXY = arrays.pXY[i];
			auto& pZvX = arrays.pZvX[i];
			auto& vYZ = arrays.vYZ[i];

			pXY[0] += pZvX[1] * timeStep;
			pXY[1] += vYZ[0] * timeStep;
			pZvX[0] += vYZ[1] * timeStep;

			auto& qXY = arrays.qXY[i];
			auto& qZW = arrays.qZW[i];
			auto& aXY = arrays.aXY[i];
			auto& aZ = arrays.aZ[i];
			rotationIntegration(
				qXY[0],
				qXY[1],
				qZW[0],
				qZW[1],
				aXY[0],
				aXY[1],
				aZ,
				timeStep);
		}
	}
	auto toc = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = toc - tic;
	std::cout << "2-element SoAoS with " << instructionSetName << ": " << duration.count() << " ms ";
	std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
}

template <typename FloatType>
void testBodyArray(
	BodyArrays4<FloatType>& arrays,
	size_t iterations,
	float timeStep,
	const std::string& instructionSetName,
	const std::chrono::duration<double, std::milli>& refDuration)
{
	auto tic = std::chrono::high_resolution_clock::now();
	for (size_t iter = 0; iter < iterations; ++iter)
	{
		for (size_t i = 0; i < arrays.size; ++i)
		{
			auto& q = arrays.q[i];
			auto& pXYZvX = arrays.pXYZvX[i];
			auto& vYZaXY = arrays.vYZaXY[i];
			auto& aZ = arrays.aZ[i];
			pXYZvX[0] += pXYZvX[3] * timeStep;
			pXYZvX[1] += vYZaXY[0] * timeStep;
			pXYZvX[2] += vYZaXY[1] * timeStep;
			rotationIntegration(
				q[0],
				q[1],
				q[2],
				q[3],
				vYZaXY[2],
				vYZaXY[3],
				aZ,
				timeStep);
		}
	}
	auto toc = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = toc - tic;
	std::cout << "4-element SoAoS with " << instructionSetName << ": " << duration.count() << " ms ";
	std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
}

template <typename FloatType>
void testBodyArray(
	BodyArraysNatural<FloatType>& arrays,
	size_t iterations,
	float timeStep,
	const std::string& instructionSetName,
	const std::chrono::duration<double, std::milli>& refDuration)
{
	auto tic = std::chrono::high_resolution_clock::now();
	for (size_t iter = 0; iter < iterations; ++iter)
	{
		for (size_t i = 0; i < arrays.size; ++i)
		{
			auto& p = arrays.p[i];
			auto& v = arrays.v[i];
			auto& q = arrays.q[i];
			auto& a = arrays.a[i];
			p[0] += v[0] * timeStep;
			p[1] += v[1] * timeStep;
			p[2] += v[2] * timeStep;
			rotationIntegration(
				q[0],
				q[1],
				q[2],
				q[3],
				a[0],
				a[1],
				a[2],
				timeStep);
		}
	}
	auto toc = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = toc - tic;
	std::cout << "'natural' SoAoS with " << instructionSetName << ": " << duration.count() << " ms ";
	std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
}


template <typename FloatType>
void compareResults(
	const std::vector<TestBody>& reference,
	const BodyArrays<FloatType>& test,
	const std::string& instructionSetName)
{
	std::cout << std::endl << "Comparing results with " << instructionSetName << " SoA:" << std::endl;
	for (size_t i = 0; i < reference.size(); ++i)
	{
		const auto& body = reference[i];

		size_t d = i / FloatType::size();
		int m = i % FloatType::size();

		const nph::Vec3 position(test.pX[d][m], test.pY[d][m], test.pZ[d][m]);
		const nph::Quat rotation(test.qX[d][m], test.qY[d][m], test.qZ[d][m], test.qW[d][m]);

		if (body.position != position)
		{
			std::cerr << "Position mismatch at index " << i << std::endl;
		}

		if (body.rotation != rotation)
		{
			std::cerr << "Rotation mismatch at index " << i << std::endl;
		}
	}
	std::cout << "Comparison finished!" << std::endl;
}

template <typename FloatType>
void compareResults(
	const std::vector<TestBody>& reference,
	const BodyArrays4<FloatType>& test,
	const std::string& instructionSetName)
{
	std::cout << std::endl << "Comparing results with " << instructionSetName << " 4-element SoAoS:" << std::endl;
	for (size_t i = 0; i < reference.size(); ++i)
	{
		const auto& body = reference[i];
		size_t d = i / FloatType::size();
		int m = i % FloatType::size();
		const Vec3 position(test.pXYZvX[d][0][m], test.pXYZvX[d][1][m], test.pXYZvX[d][2][m]);
		const Quat rotation(test.q[d][0][m], test.q[d][1][m], test.q[d][2][m], test.q[d][3][m]);
		if (body.position != position)
		{
			std::cerr << "Position mismatch at index " << i << std::endl;
		}
		const Quat& refRotation = body.rotation;
		if (refRotation != rotation)
		{
			std::cerr << "Rotation mismatch at index " << i << std::endl;
		}
	}
	std::cout << "Comparison finished!" << std::endl;
}

template <typename FloatType>
void compareResults(
	const std::vector<TestBody>& reference,
	const BodyArraysNatural<FloatType>& test,
	const std::string& instructionSetName)
{
	std::cout << std::endl << "Comparing results with " << instructionSetName << " 'natural' SoAoS:" << std::endl;
	for (size_t i = 0; i < reference.size(); ++i)
	{
		const auto& body = reference[i];
		size_t d = i / FloatType::size();
		int m = i % FloatType::size();
		const Vec3 position(test.p[d][0][m], test.p[d][1][m], test.p[d][2][m]);
		const Quat rotation(test.q[d][0][m], test.q[d][1][m], test.q[d][2][m], test.q[d][3][m]);
		if (body.position != position)
		{
			std::cerr << "Position mismatch at index " << i << std::endl;
		}
		const Quat& refRotation = body.rotation;
		if (refRotation != rotation)
		{
			std::cerr << "Rotation mismatch at index " << i << std::endl;
		}
	}
	std::cout << "Comparison finished!" << std::endl;
}

namespace ecs
{
	struct PosX {};

	struct PosY {};

	struct PosZ {};

	struct LinVelX {};

	struct LinVelY {};

	struct LinVelZ {};

	struct RotX {};

	struct RotY {};

	struct RotZ {};

	struct RotW {};

	struct AngVelX {};

	struct AngVelY {};

	struct AngVelZ {};

	struct Pos {};

	struct LinVel{};

	struct Rot{};

	struct AngVel{};

	struct AllComponents {};

	struct Body {};

	template <typename FloatType>
	struct BodyStruct
	{
		Vec<3, FloatType> position;
		Vec<3, FloatType> linearVelocity;
		QuatT<FloatType> rotation;
		Vec<3, FloatType> angularVelocity;
	};

	template <typename FloatType>
	using BodyArray = Storage<Layout<
		Block<PosX, FloatType>,
		Block<PosY, FloatType>,
		Block<PosZ, FloatType>,
		Block<LinVelX, FloatType>,
		Block<LinVelY, FloatType>,
		Block<LinVelZ, FloatType>,
		Block<RotX, FloatType>,
		Block<RotY, FloatType>,
		Block<RotZ, FloatType>,
		Block<RotW, FloatType>,
		Block<AngVelX, FloatType>,
		Block<AngVelY, FloatType>,
		Block<AngVelZ, FloatType>>>;

	template <typename FloatType>
	using NaturalBodyArray = Storage<Layout<
		Block<Pos, FloatType, 3>,
		Block<LinVel, FloatType, 3>,
		Block<Rot, FloatType, 4>,
		Block<AngVel, FloatType, 3>>>;

	template <typename FloatType>
	using AllBodyArray = Storage<Layout<
		Block<AllComponents, FloatType, 13>>>;

	template <typename FloatType>
	using BodyStructArray = Storage<Layout<Block<Body, BodyStruct<FloatType>, 1>>>;

	template <typename FloatType>
	auto fillBodyArray(const std::vector<TestBody>& bodies)
	{
		static_assert(sizeof(FloatType) % sizeof(float) == 0, "FloatType must be a multiple of float");
		constexpr size_t components = sizeof(FloatType) / sizeof(float);

		BodyArray<FloatType> result((bodies.size() + components - 1) / components);
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			const auto& body = bodies[i];

			result.getPtr<PosX>(d)->insert(m, body.position.x);
			result.getPtr<PosY>(d)->insert(m, body.position.y);
			result.getPtr<PosZ>(d)->insert(m, body.position.z);
			result.getPtr<LinVelX>(d)->insert(m, body.linearVelocity.x);
			result.getPtr<LinVelY>(d)->insert(m, body.linearVelocity.y);
			result.getPtr<LinVelZ>(d)->insert(m, body.linearVelocity.z);

			const Vec3& a = body.angularVelocity;
			const Quat& q = body.rotation;

			result.getPtr<AngVelX>(d)->insert(m, a.x);
			result.getPtr<AngVelY>(d)->insert(m, a.y);
			result.getPtr<AngVelZ>(d)->insert(m, a.z);

			result.getPtr<RotX>(d)->insert(m, q.x);
			result.getPtr<RotY>(d)->insert(m, q.y);
			result.getPtr<RotZ>(d)->insert(m, q.z);
			result.getPtr<RotW>(d)->insert(m, q.w);
		}
		return result;
	}

	template <typename FloatType>
	auto fillNaturalBodyArray(const std::vector<TestBody>& bodies)
	{
		static_assert(sizeof(FloatType) % sizeof(float) == 0, "FloatType must be a multiple of float");
		constexpr size_t components = sizeof(FloatType) / sizeof(float);
		NaturalBodyArray<FloatType> result((bodies.size() + components - 1) / components);
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			const auto& body = bodies[i];

			auto* p = result.getPtr<Pos>(d);
			p[0].insert(m, body.position.x);
			p[1].insert(m, body.position.y);
			p[2].insert(m, body.position.z);

			auto* v = result.getPtr<LinVel>(d);
			v[0].insert(m, body.linearVelocity.x);
			v[1].insert(m, body.linearVelocity.y);
			v[2].insert(m, body.linearVelocity.z);

			auto* q = result.getPtr<Rot>(d);
			q[0].insert(m, body.rotation.x);
			q[1].insert(m, body.rotation.y);
			q[2].insert(m, body.rotation.z);
			q[3].insert(m, body.rotation.w);

			auto* a = result.getPtr<AngVel>(d);
			a[0].insert(m, body.angularVelocity.x);
			a[1].insert(m, body.angularVelocity.y);
			a[2].insert(m, body.angularVelocity.z);
		}
		return result;
	}

	template <typename FloatType>
	auto fillAllBodyArray(const std::vector<TestBody>& bodies)
	{
		static_assert(sizeof(FloatType) % sizeof(float) == 0, "FloatType must be a multiple of float");
		constexpr size_t components = sizeof(FloatType) / sizeof(float);
		AllBodyArray<FloatType> result((bodies.size() + components - 1) / components);
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			const auto& body = bodies[i];
			auto* p = result.getPtr<AllComponents>(d);
			p[0].insert(m, body.position.x);
			p[1].insert(m, body.position.y);
			p[2].insert(m, body.position.z);
			p[3].insert(m, body.linearVelocity.x);
			p[4].insert(m, body.linearVelocity.y);
			p[5].insert(m, body.linearVelocity.z);
			p[6].insert(m, body.rotation.x);
			p[7].insert(m, body.rotation.y);
			p[8].insert(m, body.rotation.z);
			p[9].insert(m, body.rotation.w);
			p[10].insert(m, body.angularVelocity.x);
			p[11].insert(m, body.angularVelocity.y);
			p[12].insert(m, body.angularVelocity.z);
		}
		return result;
	}

	template <typename FloatType>
	BodyStructArray<FloatType> fillBodyStructArray(const std::vector<TestBody>& bodies)
	{
		static_assert(sizeof(FloatType) % sizeof(float) == 0, "FloatType must be a multiple of float");
		BodyStructArray<FloatType> result((bodies.size() + FloatType::size() - 1) / FloatType::size());;
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			auto& bodiesBlock = *result.getPtr<Body>(d);
			bodiesBlock.position.x.insert(m, bodies[i].position.x);
			bodiesBlock.position.y.insert(m, bodies[i].position.y);
			bodiesBlock.position.z.insert(m, bodies[i].position.z);

			bodiesBlock.linearVelocity.x.insert(m, bodies[i].linearVelocity.x);
			bodiesBlock.linearVelocity.y.insert(m, bodies[i].linearVelocity.y);
			bodiesBlock.linearVelocity.z.insert(m, bodies[i].linearVelocity.z);

			bodiesBlock.rotation.x.insert(m, bodies[i].rotation.x);
			bodiesBlock.rotation.y.insert(m, bodies[i].rotation.y);
			bodiesBlock.rotation.z.insert(m, bodies[i].rotation.z);
			bodiesBlock.rotation.w.insert(m, bodies[i].rotation.w);

			bodiesBlock.angularVelocity.x.insert(m, bodies[i].angularVelocity.x);
			bodiesBlock.angularVelocity.y.insert(m, bodies[i].angularVelocity.y);
			bodiesBlock.angularVelocity.z.insert(m, bodies[i].angularVelocity.z);
		}
		return result;
	}

	template <typename FloatType>
	void testBodyArray(
		BodyArray<FloatType>& arrays,
		size_t iterations,
		float timeStep,
		const std::string& instructionSetName,
		const std::chrono::duration<double, std::milli>& refDuration)
	{
		auto tic = std::chrono::high_resolution_clock::now();

		auto px = arrays.getComponent<PosX>();
		auto py = arrays.getComponent<PosY>();
		auto pz = arrays.getComponent<PosZ>();

		auto vx = arrays.getComponent<LinVelX>();
		auto vy = arrays.getComponent<LinVelY>();
		auto vz = arrays.getComponent<LinVelZ>();
		auto qx = arrays.getComponent<RotX>();
		auto qy = arrays.getComponent<RotY>();
		auto qz = arrays.getComponent<RotZ>();
		auto qw = arrays.getComponent<RotW>();
		auto ax = arrays.getComponent<AngVelX>();
		auto ay = arrays.getComponent<AngVelY>();
		auto az = arrays.getComponent<AngVelZ>();

		for (size_t iter = 0; iter < iterations; ++iter)
		{
			for (size_t i = 0; i < arrays.size(); ++i)
			{
				px[i] += vx[i] * timeStep;
				py[i] += vy[i] * timeStep;
				pz[i] += vz[i] * timeStep;
				rotationIntegration(
					qx[i],
					qy[i],
					qz[i],
					qw[i],
					ax[i],
					ay[i],
					az[i],
					timeStep);
			}
		}

		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> duration = toc - tic;
		std::cout << "1-element SoA (ECS) with " << instructionSetName << ": " << duration.count() << " ms ";
		std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
	}

	template <typename FloatType>
	void testBodyArray(
		NaturalBodyArray<FloatType>& arrays,
		size_t iterations,
		float timeStep,
		const std::string& instructionSetName,
		const std::chrono::duration<double, std::milli>& refDuration)
	{
		auto tic = std::chrono::high_resolution_clock::now();
		auto* p = arrays.getPtr<Pos>(0);
		auto* v = arrays.getPtr<LinVel>(0);
		auto* q = arrays.getPtr<Rot>(0);
		auto* a = arrays.getPtr<AngVel>(0);
		for (size_t iter = 0; iter < iterations; ++iter)
		{
			for (size_t i = 0; i < arrays.size(); ++i)
			{
				p[i * 3] += v[i * 3] * timeStep;
				p[i * 3 + 1] += v[i * 3 + 1] * timeStep;
				p[i * 3 + 2] += v[i * 3 + 2] * timeStep;
				rotationIntegration(
					q[i * 4],
					q[i * 4 + 1],
					q[i * 4 + 2],
					q[i * 4 + 3],
					a[i * 3],
					a[i * 3 + 1],
					a[i * 3 + 2],
					timeStep);
			}
		}
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> duration = toc - tic;
		std::cout << "'natural' SoAoS (ECS) with " << instructionSetName << ": " << duration.count() << " ms ";
		std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
	}

	template <typename FloatType>
	void testBodyArray(
		AllBodyArray<FloatType>& arrays,
		size_t iterations,
		float timeStep,
		const std::string& instructionSetName,
		const std::chrono::duration<double, std::milli>& refDuration)
	{
		auto tic = std::chrono::high_resolution_clock::now();
		auto data = arrays.getComponent<AllComponents>();
		for (size_t iter = 0; iter < iterations; ++iter)
		{
			for (size_t i = 0; i < arrays.size(); ++i)
			{
				size_t base = i * 13;
				data[base + 0] += data[base + 3] * timeStep;
				data[base + 1] += data[base + 4] * timeStep;
				data[base + 2] += data[base + 5] * timeStep;
				rotationIntegration(
					data[base + 6],
					data[base + 7],
					data[base + 8],
					data[base + 9],
					data[base + 10],
					data[base + 11],
					data[base + 12],
					timeStep);
			}
		}
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> duration = toc - tic;
		std::cout << "Raw AoSoA (ECS) with " << instructionSetName << ": " << duration.count() << " ms ";
		std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
	}

	template <typename FloatType>
	void testBodyArray(
		BodyStructArray<FloatType>& arrays,
		size_t iterations,
		float timeStep,
		const std::string& instructionSetName,
		const std::chrono::duration<double, std::milli>& refDuration)
	{
		auto tic = std::chrono::high_resolution_clock::now();
		auto bodies = arrays.getComponent<Body>();
		for (size_t iter = 0; iter < iterations; ++iter)
		{
			for (size_t i = 0; i < arrays.size(); ++i)
			{
				auto& body = bodies[i];
				body.position += timeStep * body.linearVelocity;
				rotationIntegration(
					body.rotation.x,
					body.rotation.y,
					body.rotation.z,
					body.rotation.w,
					body.angularVelocity.x,
					body.angularVelocity.y,
					body.angularVelocity.z,
					timeStep);
			}
		}
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> duration = toc - tic;
		std::cout << "BodyStruct AoSoA (ECS) with " << instructionSetName << ": " << duration.count() << " ms ";
		std::cout << "(Speedup: " << refDuration.count() / duration.count() << "x)" << std::endl;
	}

	/// Compare results between AoS and ECS SoA implementations
	template <typename FloatType>
	void compareResults(
		const std::vector<TestBody>& reference,
		const BodyArray<FloatType>& test,
		const std::string& instructionSetName)
	{
		std::cout << std::endl << "Comparing results with " << instructionSetName << " SoA (ECS):" << std::endl;
		auto px = test.getComponent<PosX>();
		auto py = test.getComponent<PosY>();
		auto pz = test.getComponent<PosZ>();
		auto qx = test.getComponent<RotX>();
		auto qy = test.getComponent<RotY>();
		auto qz = test.getComponent<RotZ>();
		auto qw = test.getComponent<RotW>();
		for (size_t i = 0; i < reference.size(); ++i)
		{
			const auto& body = reference[i];
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			const Vec3 position(px[d][m], py[d][m], pz[d][m]);
			const Quat rotation(qx[d][m], qy[d][m], qz[d][m], qw[d][m]);
			if (body.position != position)
			{
				std::cerr << "Position mismatch at index " << i << std::endl;
			}
			const Quat& refRotation = body.rotation;
			if (refRotation != rotation)
			{
				std::cerr << "Rotation mismatch at index " << i << std::endl;
			}
		}
		std::cout << "Comparison finished!" << std::endl;
	}

	template <typename FloatType>
	void compareResultsNatural(
		const std::vector<TestBody>& reference,
		const NaturalBodyArray<FloatType>& test,
		const std::string& instructionSetName)
	{
		std::cout << std::endl << "Comparing results with " << instructionSetName << " 'natural' SoAoS (ECS):" << std::endl;
		auto* p = test.getPtr<Pos>(0);
		auto* q = test.getPtr<Rot>(0);
		for (size_t i = 0; i < reference.size(); ++i)
		{
			const auto& body = reference[i];
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			const Vec3 position(p[d * 3][m], p[d * 3 + 1][m], p[d * 3 + 2][m]);
			const Quat rotation(q[d * 4][m], q[d * 4 + 1][m], q[d * 4 + 2][m], q[d * 4 + 3][m]);
			if (body.position != position)
			{
				std::cerr << "Position mismatch at index " << i << std::endl;
			}
			const Quat& refRotation = body.rotation;
			if (refRotation != rotation)
			{
				std::cerr << "Rotation mismatch at index " << i << std::endl;
			}
		}
		std::cout << "Comparison finished!" << std::endl;
	}

	template <typename FloatType>
	void compareResultsAll(
		const std::vector<TestBody>& reference,
		const AllBodyArray<FloatType>& test,
		const std::string& instructionSetName)
	{
		std::cout << std::endl << "Comparing results with " << instructionSetName << " AoS (ECS):" << std::endl;
		auto data = test.getComponent<AllComponents>();
		for (size_t i = 0; i < reference.size(); ++i)
		{
			const auto& body = reference[i];
			size_t d = i / FloatType::size();
			int m = i % FloatType::size();
			size_t base = d * 13;
			const Vec3 position(data[base + 0][m], data[base + 1][m], data[base + 2][m]);
			const Quat rotation(data[base + 6][m], data[base + 7][m], data[base + 8][m], data[base + 9][m]);
			if (body.position != position)
			{
				std::cerr << "Position mismatch at index " << i << std::endl;
			}
			const Quat& refRotation = body.rotation;
			if (refRotation != rotation)
			{
				std::cerr << "Rotation mismatch at index " << i << std::endl;
			}
		}
		std::cout << "Comparison finished!" << std::endl;
	}
}

} // namespace nph

using namespace nph;

/// SoA test entry point
int wmain()
{
	// Test bodies integration
	constexpr float TIME_STEP = 1.0f / 60.0f;
	constexpr size_t ITERATIONS = 50000;
	constexpr size_t BODY_COUNT = 10000;

	std::vector<TestBody> bodies;
	bodies.reserve(BODY_COUNT);

	std::mt19937 rng(123);
	std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

	for (size_t i = 0; i < BODY_COUNT; ++i)
	{
		TestBody body;
		body.position = { dist(rng), dist(rng), dist(rng) };
		body.linearVelocity = { dist(rng), dist(rng), dist(rng) };

		body.angularVelocity = AxisAngle<3>(dist(rng), dist(rng), dist(rng));
		body.rotation = Quat(AxisAngle<3>(dist(rng), dist(rng), dist(rng)));
		bodies.push_back(body);
	}

	BodyArrays<simd::Vec1f> bodyArrays = fillSoA<simd::Vec1f>(bodies);
	BodyArrays<simd::Vec4f> bodyArrays4 = fillSoA<simd::Vec4f>(bodies);
	BodyArrays2<simd::Vec4f> bodyArrays24 = fillSoAoS2<simd::Vec4f>(bodies);
	BodyArrays4<simd::Vec4f> bodyArrays44 = fillSoAoS4<simd::Vec4f>(bodies);
	BodyArraysNatural<simd::Vec4f> bodyArrays4Natural = fillSoAoSNatural<simd::Vec4f>(bodies);
	BodyArrays<simd::Vec8f> bodyArrays8 = fillSoA<simd::Vec8f>(bodies);
	BodyArrays2<simd::Vec8f> bodyArrays28 = fillSoAoS2<simd::Vec8f>(bodies);
	BodyArrays4<simd::Vec8f> bodyArrays48 = fillSoAoS4<simd::Vec8f>(bodies);
	BodyArraysNatural<simd::Vec8f> bodyArrays8Natural = fillSoAoSNatural<simd::Vec8f>(bodies);

	auto ecsArray = ecs::fillBodyArray<simd::Vec1f>(bodies);
	auto ecsArray4 = ecs::fillBodyArray<simd::Vec4f>(bodies);
	auto ecsArray8 = ecs::fillBodyArray<simd::Vec8f>(bodies);

	auto ecsArray4Natural = ecs::fillNaturalBodyArray<simd::Vec4f>(bodies);
	auto ecsArray8Natural = ecs::fillNaturalBodyArray<simd::Vec8f>(bodies);

	auto ecsArrayAll = ecs::fillAllBodyArray<simd::Vec1f>(bodies);
	auto ecsArrayAll4 = ecs::fillAllBodyArray<simd::Vec4f>(bodies);
	auto ecsArrayAll8 = ecs::fillAllBodyArray<simd::Vec8f>(bodies);

	auto ecsArrayStruct = ecs::fillBodyStructArray<simd::Vec1f>(bodies);
	auto ecsArrayStruct4 = ecs::fillBodyStructArray<simd::Vec4f>(bodies);
	auto ecsArrayStruct8 = ecs::fillBodyStructArray<simd::Vec8f>(bodies);

	std::chrono::duration<double, std::milli> refDuration;
	testBodyArray(bodies, ITERATIONS, TIME_STEP, refDuration);
	testBodyArray(bodyArrays, ITERATIONS, TIME_STEP, "float", refDuration);

	ecs::testBodyArray(ecsArrayAll, ITERATIONS, TIME_STEP, "float", refDuration);
	ecs::testBodyArray(ecsArrayStruct, ITERATIONS, TIME_STEP, "float", refDuration);
	ecs::testBodyArray(ecsArray, ITERATIONS, TIME_STEP, "float", refDuration);

	std::cout << std::endl << "----------------------------------------------\n\n";
	testBodyArray(bodyArrays4, ITERATIONS, TIME_STEP, "SSE", refDuration);
	testBodyArray(bodyArrays24, ITERATIONS, TIME_STEP, "SSE", refDuration);
	testBodyArray(bodyArrays44, ITERATIONS, TIME_STEP, "SSE", refDuration);
	testBodyArray(bodyArrays4Natural, ITERATIONS, TIME_STEP, "SSE", refDuration);

	ecs::testBodyArray(ecsArrayAll4, ITERATIONS, TIME_STEP, "SSE", refDuration);
	ecs::testBodyArray(ecsArrayStruct4, ITERATIONS, TIME_STEP, "SSE", refDuration);
	ecs::testBodyArray(ecsArray4, ITERATIONS, TIME_STEP, "SSE", refDuration);
	ecs::testBodyArray(ecsArray4Natural, ITERATIONS, TIME_STEP, "SSE", refDuration);

	std::cout << std::endl << "----------------------------------------------\n\n";
	
	testBodyArray(bodyArrays8, ITERATIONS, TIME_STEP, "AVX", refDuration);
	testBodyArray(bodyArrays28, ITERATIONS, TIME_STEP, "AVX", refDuration);
	testBodyArray(bodyArrays48, ITERATIONS, TIME_STEP, "AVX", refDuration);
	testBodyArray(bodyArrays8Natural, ITERATIONS, TIME_STEP, "AVX", refDuration);

	ecs::testBodyArray(ecsArrayAll8, ITERATIONS, TIME_STEP, "AVX", refDuration);
	ecs::testBodyArray(ecsArrayStruct8, ITERATIONS, TIME_STEP, "AVX", refDuration);
	ecs::testBodyArray(ecsArray8, ITERATIONS, TIME_STEP, "AVX", refDuration);
	ecs::testBodyArray(ecsArray8Natural, ITERATIONS, TIME_STEP, "AVX", refDuration);

	std::cout << std::endl << "----------------------------------------------\n\n";

	compareResults(bodies, bodyArrays, "float");
	compareResults(bodies, bodyArrays4, "SSE");
	compareResults(bodies, bodyArrays44, "SSE");
	compareResults(bodies, bodyArrays4Natural, "SSE");

	compareResults(bodies, bodyArrays8, "AVX");
	compareResults(bodies, bodyArrays48, "AVX");
	compareResults(bodies, bodyArrays8Natural, "AVX");

	ecs::compareResults(bodies, ecsArray, "float");
	ecs::compareResults(bodies, ecsArray4, "SSE");
	ecs::compareResults(bodies, ecsArray8, "AVX");

	ecs::compareResultsNatural(bodies, ecsArray4Natural, "SSE");
	ecs::compareResultsNatural(bodies, ecsArray8Natural, "AVX");

	ecs::compareResultsAll(bodies, ecsArrayAll, "float");
	ecs::compareResultsAll(bodies, ecsArrayAll4, "SSE");

	// Prevent closing of the console immediately
	std::cout << "Press Enter to exit...";
	std::cin.get();

	return 0;
}
