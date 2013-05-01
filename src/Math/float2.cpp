/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file float2.cpp
	@author Jukka Jylänki
	@brief */
#include "float2.h"
#include "float3.h"
#include "float4.h"
#include "MathFunc.h"
#include "../Algorithm/Random/LCG.h"
#include "assume.h"
#include <string.h>
#include <stdlib.h>
#include <locale.h>

#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#include <iostream>
#include <utility>
#include <algorithm>
#endif

MATH_BEGIN_NAMESPACE

using namespace std;

float2::float2(float x_, float y_)
:x(x_), y(y_)
{
}

float2::float2(float scalar)
:x(scalar), y(scalar)
{
}

float2::float2(const float *data)
{
	assume(data);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!data)
		return;
#endif
	x = data[0];
	y = data[1];
}

float *float2::ptr()
{
	return &x;
}

const float *float2::ptr() const
{
	return &x;
}

CONST_WIN32 float float2::At(int index) const
{
	assume(index >= 0);
	assume(index < Size);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (index < 0 || index >= Size)
		return FLOAT_NAN;
#endif
	return ptr()[index];
}

float &float2::At(int index)
{
	assume(index >= 0);
	assume(index < Size);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (index < 0 || index >= Size)
		return ptr()[0];
#endif
	return ptr()[index];
}

float2 float2::Swizzled(int i, int j) const
{
	return float2(At(i), At(j));
}

float3 float2::Swizzled(int i, int j, int k) const
{
	return float3(At(i), At(j), At(k));
}

float4 float2::Swizzled(int i, int j, int k, int l) const
{
	return float4(At(i), At(j), At(k), At(l));
}

float float2::LengthSq() const
{
	return x*x + y*y;
}

float float2::Length() const
{
	return Sqrt(LengthSq());
}

void float2::SetFromPolarCoordinates(float theta, float length)
{
	x = Cos(theta) * length;
	y = Sin(theta) * length;
}

float2 float2::FromPolarCoordinates(float theta, float length)
{
	float2 euclidean;
	euclidean.SetFromPolarCoordinates(theta, length);
	return euclidean;
}

float2 float2::ToPolarCoordinates() const
{
	float radius = Length();
	if (radius > 1e-4f)
		return float2(atan2(y, x), radius);
	else
		return float2::zero;
}

float float2::AimedAngle() const
{
	assume(!IsZero());
	return atan2(y, x);
}

float float2::Normalize()
{
	assume(IsFinite());
	float lengthSq = LengthSq();
	if (lengthSq > 1e-6f)
	{
		float length = Sqrt(lengthSq);
		*this *= 1.f / length;
		return length;
	}
	else
	{
		Set(1.f, 0.f); // We will always produce a normalized vector.
		return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
	}
}

float2 float2::Normalized() const
{
	float2 copy = *this;
	float oldLength = copy.Normalize();
	assume(oldLength > 0.f && "float2::Normalized() failed!");
	MARK_UNUSED(oldLength);
	return copy;
}

float float2::ScaleToLength(float newLength)
{
	float length = LengthSq();
	if (length < 1e-6f)
		return 0.f;

	length = Sqrt(length);
	float scalar = newLength / length;
	x *= scalar;
	y *= scalar;
	return length;
}

float2 float2::ScaledToLength(float newLength) const
{
	assume(!IsZero());

	float2 v = *this;
	v.ScaleToLength(newLength);
	return v;
}

bool float2::IsNormalized(float epsilonSq) const
{
	return fabs(LengthSq()-1.f) <= epsilonSq;
}

bool float2::IsZero(float epsilonSq) const
{
	return fabs(LengthSq()) <= epsilonSq;
}

bool float2::IsFinite() const
{
	return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y);
}

bool float2::IsPerpendicular(const float2 &other, float epsilon) const
{
	return fabs(Dot(other)) <= epsilon;
}

bool float2::Equals(const float2 &rhs, float epsilon) const
{
	return EqualAbs(x, rhs.x, epsilon) && EqualAbs(y, rhs.y, epsilon);
}

bool float2::Equals(float x_, float y_, float epsilon) const
{
	return EqualAbs(x, x_, epsilon) && EqualAbs(y, y_, epsilon);
}

/// It is too performance-heavy to set the locale in each serialization and deserialization function call.
/// Therefore expect the user to has a proper locale set up for the application at startup. This is assert()ed
/// at debug runs.
bool IsNeutralCLocale()
{
#ifndef ANDROID ///\todo error: 'struct lconv' has no member named 'decimal_point'
	lconv *lc = localeconv();
	if (strcmp(lc->decimal_point, "."))
		return false;
#endif
	return true;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string float2::ToString() const
{
	char str[256];
	sprintf(str, "(%f, %f)", x, y);
	return std::string(str);
}

std::string float2::SerializeToString() const
{
	assert(IsNeutralCLocale());
	char str[256];
	sprintf(str, "%f %f", x, y);
	return std::string(str);
}
#endif

float2 float2::FromString(const char *str)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return float2::nan;
	if (*str == '(')
		++str;
	float2 f;
	f.x = (float)strtod(str, const_cast<char**>(&str));
	while(*str == ' ' || *str == '\t')
		++str;
	if (*str == ',' || *str == ';')
		++str;
	f.y = (float)strtod(str, const_cast<char**>(&str));
	return f;
}

float float2::SumOfElements() const
{
	return x + y;
}

float float2::ProductOfElements() const
{
	return x * y;
}

float float2::AverageOfElements() const
{
	return (x + y) / 2.f;
}

float float2::MinElement() const
{
	return MATH_NS::Min(x, y);
}

int float2::MinElementIndex() const
{
	return (x <= y) ? 0 : 1;
}

float float2::MaxElement() const
{
	return  MATH_NS::Max(x, y);
}

int float2::MaxElementIndex() const
{
	return (x > y) ? 0 : 1;
}

float2 float2::Abs() const
{
	return float2(fabs(x), fabs(y));
}

float2 float2::Neg() const
{
	return float2(-x, -y);
}

float2 float2::Recip() const
{
	return float2(1.f/x, 1.f/y);
}

float2 float2::Min(float floor) const
{
	return float2(MATH_NS::Min(x, floor),  MATH_NS::Min(x, floor));
}

float2 float2::Min(const float2 &floor) const
{
	return float2(MATH_NS::Min(x, floor.x),  MATH_NS::Min(x, floor.x));
}

float2 float2::Max(float ceil) const
{
	return float2(MATH_NS::Max(x, ceil),  MATH_NS::Max(x, ceil));
}

float2 float2::Max(const float2 &ceil) const
{
	return float2(MATH_NS::Max(x, ceil.x),  MATH_NS::Max(x, ceil.x));
}

float2 float2::Clamp(const float2 &floor, const float2 &ceil) const
{
	return float2(MATH_NS::Clamp(x, floor.x, ceil.x),  MATH_NS::Clamp(y, floor.y, ceil.y));
}

float2 float2::Clamp(float floor, float ceil) const
{
	return float2(MATH_NS::Clamp(x, floor, ceil),  MATH_NS::Clamp(y, floor, ceil));
}

float2 float2::Clamp01() const
{
	return Clamp(0.f, 1.f);
}

float float2::DistanceSq(const float2 &rhs) const
{
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	return dx*dx + dy*dy;
}

float float2::Distance(const float2 &rhs) const
{
	return Sqrt(DistanceSq(rhs));
}

float float2::Dot(const float2 &rhs) const
{
	return x * rhs.x + y * rhs.y;
}

float2 float2::Perp() const
{
	return float2(-y, x);
}

float float2::PerpDot(const float2 &rhs) const
{
	return -y * rhs.x + x * rhs.y;
}

float2 float2::Reflect(const float2 &normal) const
{
	assume(normal.IsNormalized());
	return 2.f * this->ProjectToNorm(normal) - *this;
}

/// Implementation from http://www.flipcode.com/archives/reflection_transmission.pdf .
float2 float2::Refract(const float2 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const
{
	// This code is duplicated in float3::Refract.
	float n = negativeSideRefractionIndex / positiveSideRefractionIndex;
	float cosI = this->Dot(normal);
	float sinT2 = n*n*(1.f - cosI*cosI);
	if (sinT2 > 1.f) // Total internal reflection occurs?
		return (-*this).Reflect(normal);
	return n * *this - (n + Sqrt(1.f - sinT2)) * normal;
}

float2 float2::ProjectTo(const float2 &direction) const
{
	assume(!direction.IsZero());
	return direction * this->Dot(direction) / direction.LengthSq();
}

float2 float2::ProjectToNorm(const float2 &direction) const
{
	assume(direction.IsNormalized());
	return direction * this->Dot(direction);
}

float float2::AngleBetween(const float2 &other) const
{
	return acos(Dot(other)) / Sqrt(LengthSq() * other.LengthSq());
}

float float2::AngleBetweenNorm(const float2 &other) const
{
	assume(this->IsNormalized());
	assume(other.IsNormalized());
	return acos(Dot(other));
}

float2 float2::Lerp(const float2 &b, float t) const
{
	assume(0.f <= t && t <= 1.f);
	return (1.f - t) * *this + t * b;
}

float2 float2::Lerp(const float2 &a, const float2 &b, float t)
{
	return a.Lerp(b, t);
}

void float2::Decompose(const float2 &direction, float2 &outParallel, float2 &outPerpendicular) const
{
	assume(direction.IsNormalized());
	outParallel = this->Dot(direction) * direction;
	outPerpendicular = *this - outParallel;
}

void float2::Orthogonalize(const float2 &a, float2 &b)
{
	assume(!a.IsZero());
	b -= a.Dot(b) / a.Length() * a;
}

bool float2::AreOrthogonal(const float2 &a, const float2 &b, float epsilon)
{
	return a.IsPerpendicular(b, epsilon);
}


void float2::Orthonormalize(float2 &a, float2 &b)
{
	assume(!a.IsZero());
	a.Normalize();
	b -= a.Dot(b) * a;
}

float2 float2::FromScalar(float scalar)
{
	return float2(scalar, scalar);
}

void float2::SetFromScalar(float scalar)
{
	x = scalar;
	y = scalar;
}

void float2::Set(float x_, float y_)
{
	x = x_;
	y = y_;
}

void float2::Rotate90CW()
{
	float oldX = x;
	x = y;
	y = -oldX;
}

float2 float2::Rotated90CW() const
{
	return float2(y, -x);
}

void float2::Rotate90CCW()
{
	float oldX = x;
	x = -y;
	y = oldX;
}

float2 float2::Rotated90CCW() const
{
	return float2(-y, x);
}

bool float2::OrientedCCW(const float2 &a, const float2 &b, const float2 &c)
{
	// Compute the determinant
	// | ax ay 1 |
	// | bx by 1 |
	// | cx cy 1 |
	// See Christer Ericson, Real-Time Collision Detection, p.32.
	return (a.x-c.x)*(b.y-c.y) - (a.y-c.y)*(b.x-c.x) >= 0.f;
}

class SortByPolarAngle
{
public:
	float2 perspective;

	bool operator()(const float2 &a, const float2 &b) const
	{
		float2 A = a - perspective;
		float2 B = b - perspective;
		return A.x*B.y < B.x*A.y;
	}
};

#ifdef MATH_ENABLE_STL_SUPPORT
void float2::ConvexHull(const float2 *pointArray, int numPoints, std::vector<float2> &outConvexHull)
{
	outConvexHull.clear();
	if (numPoints == 0)
		return;
	outConvexHull.insert(outConvexHull.end(), pointArray, pointArray + numPoints);
	int convexHullSize = ConvexHullInPlace(&outConvexHull[0], (int)outConvexHull.size());
	outConvexHull.resize(convexHullSize);
}
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
/** This function implements the Graham's Scan algorithm for finding the convex hull of
	a 2D point set. The running time is O(nlogn). For details, see
	"Introduction to Algorithms, 2nd ed.", by Cormen, Leiserson, Rivest, p.824, or
	a lecture by Shai Simonson: http://www.aduni.org/courses/algorithms/index.php?view=cw , lecture 02-13-01. */
int float2::ConvexHullInPlace(float2 *points, int nPoints)
{
	if (nPoints <= 3)
		return nPoints;
	// Find the lowest point of the set.
	float2 *lowest = &points[0];
	for(int i = 1; i < nPoints; ++i)
		if (points[i].y < lowest->y)
			lowest = &points[i];
	Swap(*lowest, points[0]);
	SortByPolarAngle pred;
	pred.perspective = points[0];
	std::sort(&points[1], &points[nPoints], pred);
	int nPointsInHull = 2; // Two first points are in the hull without checking.
	for(int i = 2; i < nPoints; ++i)
	{
		// The last two added points determine a line, check which side of that line the next point to be added lies in.
		float2 lineA = points[nPointsInHull-1] - points[nPointsInHull-2];
		float2 lineB = points[i] - points[nPointsInHull-2];
		float lineALen = lineA.LengthSq();
		float lineBLen = lineB.LengthSq();
		bool dropLastPointFromHull = false;
		if (lineALen >= 1e-5f)
			lineA /= Sqrt(lineALen);
		else
			dropLastPointFromHull = true;
		if (lineBLen >= 1e-5f)
			lineB /= Sqrt(lineBLen);
		float2 normal = float2(-lineA.y, lineA.x);
		if (dropLastPointFromHull ||  MATH_NS::Dot(normal, lineB) > 0.f || (MATH_NS::Dot(normal,lineB) > -1e-4f && lineBLen >= lineALen))// || (Length2(points[i] - points[nPointsInHull-1]) <= 1e-5f)) // lineB is to the left of lineA?
		{
			// Points[n-1] is not part of the convex hull. Drop that point and decrement i to reprocess the current point.
			// (It may be that the current point will cause lots of points to drop out of the convex hull.
			if (nPointsInHull > 2)
			{
				--nPointsInHull;
				--i;
			}
			else
				points[nPointsInHull-1] = points[i];
		}
		else
			points[nPointsInHull++] = points[i];
	}

	// The array points now stores the convex hull. For robustness,
	// prune all duplicate and redundant points from the hull (due to floating point imprecisions).
	for(int i = 0; i < nPointsInHull && nPointsInHull > 3; ++i)
	{
		// Remove any adjacent points that are too close.
		if (points[i].Equals(points[(i+1)%nPointsInHull]))
		{
			for(int j = i; j+1 < nPointsInHull; ++j)
				points[j] = points[j+1];
			--nPointsInHull;
			--i;
			continue;
		}

		// Remove any adjacent points that are on the same line.
		float2 dirA = points[(i+1)%nPointsInHull] - points[i];
		dirA.Normalize();
		float2 dirB = points[i] - points[(i+nPointsInHull-1)%nPointsInHull];
		dirB.Normalize();
		if (MATH_NS::Dot(dirA, dirB) >= 1.f - 1e-3f)
		{
			for(int j = i; j+1 < nPointsInHull; ++j)
				points[j] = points[j+1];
			--nPointsInHull;
			--i;
			continue;
		}
	}

	return nPointsInHull;
}
#endif

float float2::MinAreaRect(const float2 *pts, int numPoints, float2 &center, float2 &uDir, float2 &vDir, float &minU, float &maxU, float &minV, float &maxV)
{
	assume(pts || numPoints == 0);
	if (!pts)
		return 0.f;
	float minArea = FLT_MAX;

	// Loop through all edges formed by pairs of points.
	for(int i = 0, j = numPoints -1; i < numPoints; j = i, ++i)
	{
		// The edge formed by these two points.
		float2 e0 = pts[i] - pts[j];
		float len = e0.Normalize();
		if (len == 0)
			continue; // the points are duplicate, skip this axis.

		float2 e1 = e0.Rotated90CCW();

		// Find the most extreme points along the coordinate frame { e0, e1 }.

		///@todo Examine. A bug in the book? All the following are initialized to 0!.
		float min0 = FLOAT_INF;
		float min1 = FLOAT_INF;
		float max0 = -FLOAT_INF;
		float max1 = -FLOAT_INF;
		for(int k = 0; k < numPoints; ++k)
		{
			float2 d = pts[k] - pts[j];
			float dot =  MATH_NS::Dot(d, e0);
			if (dot < min0) min0 = dot;
			if (dot > max0) max0 = dot;
			dot =  MATH_NS::Dot(d, e1);
			if (dot < min1) min1 = dot;
			if (dot > max1) max1 = dot;
		}
		float area = (max0 - min0) * (max1 - min1);

		if (area < minArea)
		{
			minArea = area;
			center = pts[j] + 0.5f * ((min0 + max0) * e0 + (min1 + max1) * e1);
			uDir = e0;
			vDir = e1;
			minU = min0;
			maxU = max0;
			minV = min1;
			maxV = max1;
		}
	}
	return minArea;
}

float2 float2::RandomDir(LCG &lcg, float r)
{
	assume(r > 1e-3f);
	for(int i = 0; i < 1000; ++i)
	{
		float x = lcg.Float(-r, r);
		float y = lcg.Float(-r, r);
		float lenSq = x*x + y*y;
		if (lenSq >= 1e-6f && lenSq <= r*r)
			return r / Sqrt(lenSq) * float2(x,y);
	}
	assume(false && "Failed to generate a random float2 direction vector!");
	return float2(r, 0);
}

MUST_USE_RESULT float2 float2::RandomBox(LCG &lcg, float minElem, float maxElem)
{
	float x = lcg.Float(minElem, maxElem);
	float y = lcg.Float(minElem, maxElem);
	return float2(x, y);
}

float2 float2::operator +(const float2 &rhs) const
{
	return float2(x + rhs.x, y + rhs.y);
}

float2 float2::operator -(const float2 &rhs) const
{
	return float2(x - rhs.x, y - rhs.y);
}

float2 float2::operator -() const
{
	return float2(-x, -y);
}

float2 float2::operator *(float scalar) const
{
	return float2(x * scalar, y * scalar);
}

float2 operator *(float scalar, const float2 &rhs)
{
	return float2(scalar * rhs.x, scalar * rhs.y);
}

float2 float2::operator /(float scalar) const
{
	float invScalar = 1.f / scalar;
	return float2(x * invScalar, y * invScalar);
}

float2 operator /(float scalar, const float2 &rhs)
{
	return float2(scalar / rhs.x, scalar / rhs.y);
}

float2 &float2::operator +=(const float2 &rhs)
{
	x += rhs.x;
	y += rhs.y;

	return *this;
}

float2 &float2::operator -=(const float2 &rhs)
{
	x -= rhs.x;
	y -= rhs.y;

	return *this;
}

float2 &float2::operator *=(float scalar)
{
	x *= scalar;
	y *= scalar;

	return *this;
}

float2 float2::Add(float s) const
{
	return float2(x + s, y + s);
}

float2 float2::Sub(float s) const
{
	return float2(x - s, y - s);
}

float2 float2::SubLeft(float s) const
{
	return float2(s - x, s - y);
}

float2 float2::DivLeft(float s) const
{
	return float2(s / x, s / y);
}

float2 float2::Mul(const float2 &rhs) const
{
	return float2(x * rhs.x, y * rhs.y);
}

float2 float2::Div(const float2 &rhs) const
{
	return float2(x / rhs.x, y / rhs.y);
}

float2 &float2::operator /=(float scalar)
{
	float invScalar = 1.f / scalar;
	x *= invScalar;
	y *= invScalar;

	return *this;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float2 &rhs)
{
	std::string str = rhs.ToString();
	out << str;
	return out;
}
#endif

const float2 float2::zero = float2(0, 0);
const float2 float2::one = float2(1, 1);
const float2 float2::unitX = float2(1, 0);
const float2 float2::unitY = float2(0, 1);
const float2 float2::nan = float2(FLOAT_NAN, FLOAT_NAN);
const float2 float2::inf = float2(FLOAT_INF, FLOAT_INF);

MATH_END_NAMESPACE
