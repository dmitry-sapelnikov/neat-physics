#pragma once

// Includes
#include <cstdint>

namespace nph
{

template <typename T>
struct Point2
{
	T x;
	T y;
};

template <typename T>
Point2<T> operator+(const Point2<T>& a, const Point2<T>& b) noexcept
{
	return { a.x + b.x, a.y + b.y };
}

template <typename T>
Point2<T> operator-(const Point2<T>& a, const Point2<T>& b) noexcept
{
	return { a.x - b.x, a.y - b.y };
}

using Point2u = Point2<uint32_t>;

using Point2i = Point2<int32_t>;

// End of the namespace gltut
}
