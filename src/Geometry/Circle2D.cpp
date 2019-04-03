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

   /** @file Circle2D.cpp
	   @author Jukka Jylänki
	   @brief Implementation for the Circle2D geometry object. */
#include "Circle2D.h"
#include "../Math/MathFunc.h"
#include "../Math/Swap.h"
#include "../Algorithm/Random/LCG.h"

MATH_BEGIN_NAMESPACE

Circle2D::Circle2D(const float2 &center, float radius)
:pos(center), r(radius)
{
}

// Uncomment for internal debugging
// #define DEBUG_MINCIRCLE

// Helper function to compute the minimal circle that contains the given three points.
// To avoid extra Sqrt() operations in hot inner loops, this function returns a Circle2D structure that has its radius squared (caller needs to compute circle.r = Sqrt(circle.r) to use)
static inline Circle2D MakeCircleSq(float denom, float AB_AC, float AB, float AC, const float2 &a, const float2 &b, const float2 &c, const float2 &ab, const float2 &ac)
{
    if (Abs(denom) < 1e-5f) // Each of a, b and c lie on a straight line?
    {
        if (AB_AC > 0.f)
            return AB > AC ? Circle2D((a+b)*0.5f, AB*0.25f) : Circle2D((a+c)*0.5f, AC*0.25f);
        else
            return Circle2D((b+c)*0.5f, (AB + AC - 2.f*AB_AC)*0.25f);
    }
    denom = 0.5f / denom;
    float s = (AC * AB - AB_AC * AC) * denom;
    float t = (AC * AB - AB_AC * AB) * denom;
    if (s < 0.f)
        return Circle2D((a + c) * 0.5f, AC * 0.25f);
    else if (t < 0.f)
        return Circle2D((a + b) * 0.5f, AB * 0.25f);
    else if (s + t > 1.f)
        return Circle2D((b + c) * 0.5f, (AB + AC - 2.f*AB_AC)*0.25f);
    else
    {
        const float2 center = s * ab + t * ac;
        return Circle2D(a + center, center.LengthSq());
    }
}

// Finds minimum circle that encloses a-d, with the preknowledge that a is not contained in minimal
// circle that encloses b-d. One of three points b-c will be redundant to define this new enclosing
// circle, that will be swapped in-place with a.
static Circle2D SmallestCircleEnclosing4Points(float2 &a, float2 &b, float2 &c, float2 &d)
{
    // Find the smallest circle that encloses each of a, b, c and d.
    // As prerequisite, we know that a is not contained by smallest circle that encloses b, c and d, enforce that precondition.
    assert1(!Circle2D::OptimalEnclosingCircle(b, c, d).Contains(a, -1e-1f), Circle2D::OptimalEnclosingCircle(b, c, d).SignedDistanceSq(a));
    
    // Therefore, the smallest circle that encloses each of a, b, c and d must pass through a. Test
    // the three possible candidate circles (a,b,c), (a,b,d) and (a,c,d), one of those must enclose
    // the remaining fourth point.

    float2 ab = b - a;
    float2 ac = c - a;
    const float AB = Dot(ab,ab);
    const float AC = Dot(ac,ac);

    Circle2D circle[3];
    float sqd[3];

    const float AB_AC = Dot(ab,ac);
    float denomABC = AB*AC - AB_AC*AB_AC;
    circle[0] = MakeCircleSq(denomABC, AB_AC, AB, AC, a, b, c, ab, ac);
    sqd[0] = (circle[0].pos - d).LengthSq() - circle[0].r;
    if (sqd[0] <= 0.f)
    {
        Swap(a, d);
        circle[0].r = Sqrt(circle[0].r);
        return circle[0];
    }

    float2 ad = d - a;
    const float AD = Dot(ad,ad);

    const float AB_AD = Dot(ab,ad);
    float denomABD = AB*AD - AB_AD*AB_AD;
    circle[1] = MakeCircleSq(denomABD, AB_AD, AB, AD, a, b, d, ab, ad);
    sqd[1] = (circle[1].pos - c).LengthSq() - circle[1].r;
    if (sqd[1] <= 0.f)
    {
        Swap(a, c);
        circle[1].r = Sqrt(circle[1].r);
        return circle[1];
    }

    const float AC_AD = Dot(ac,ad);
    float denomACD = AC*AD - AC_AD*AC_AD;
    circle[2] = MakeCircleSq(denomACD, AC_AD, AC, AD, a, c, d, ac, ad);
    sqd[2] = (circle[2].pos - b).LengthSq() - circle[2].r;
    if (sqd[2] <= 0.f)
    {
        Swap(a, b);
        circle[2].r = Sqrt(circle[2].r);
        return circle[2];
    }
    
    // Robustness: Due to numerical imprecision, it can happen that each circle
    // reports the fourth point to lie outside it - in such case, pick the circle
    // that the fourth point is the least outside of.
    int ci;
    if (sqd[0] <= sqd[1] && sqd[0] <= sqd[2])
    {
        Swap(a, d);
        ci = 0;
    }
    else if (sqd[1] <= sqd[2])
    {
        Swap(a, c);
        ci = 1;
    }
    else
    {
        Swap(a, b);
        ci = 2;
    }
    circle[ci].r = Sqrt(Max(circle[ci].pos.DistanceSq(a), circle[ci].pos.DistanceSq(b), circle[ci].pos.DistanceSq(c), circle[ci].pos.DistanceSq(d))) + 1e-5f;
    return circle[ci];
}

Circle2D Circle2D::OptimalEnclosingCircle(const float2 &a, const float2 &b, const float2 &c)
{
    Circle2D circle;
    const float2 ab = b - a;
    const float2 ac = c - a;
    const float AB = Dot(ab,ab);
    const float AC = Dot(ac,ac);
    const float AB_AC = Dot(ab,ac);
    float denom = AB*AC - AB_AC*AB_AC;
        
    if (Abs(denom) < 1e-5f) // Each of a, b and c lie on a straight line?
    {
        if (AB_AC > 0.f)
        {
            if (AB > AC)
            {
                circle.pos = (a+b)*0.5f;
                //circle.r = Sqrt(AB)*0.5f;
            }
            else
            {
                circle.pos = (a+c)*0.5f;
                //circle.r = Sqrt(AC)*0.5f;
            }
        }
        else
        {
            // ||b-c|| = ||(b-a+a-c)|| == ||(b-a)+(c-a)||==||ab-ac||=Dot(ab,ab)-2Dot(ab,ac)+Dot(ac,ac)
            circle.pos = (b+c)*0.5f;
            //circle.r = Sqrt(AB + AC - 2.f*AB_AC)*0.5f;
        }
    }
    else
    {
        denom = 0.5f / denom;
        float s = (AC * AB - AB_AC * AC) * denom;
        float t = (AC * AB - AB_AC * AB) * denom;
        if (s < 0.f)
        {
            circle.pos = (a + c) * 0.5f;
            //circle.r = Sqrt(AC) * 0.5f;
        }
        else if (t < 0.f)
        {
            circle.pos = (a + b) * 0.5f;
            //circle.r = Sqrt(AB) * 0.5f;
        }
        else if (s + t > 1.f)
        {
            circle.pos = (b + c) * 0.5f;
            //circle.r = Sqrt(AB + AC - 2.f*AB_AC)*0.5f;
        }
        else
        {
            const float2 center = s * ab + t * ac;
            circle.pos = a + center;
            // Two ways to compute the radius, one via parameterization, another via vector arithmetic:
            // circle.r = Sqrt(s*s*AB + 2.f*s*t*AB_AC +t*t*AC), but it is faster to compute via 'center' vector
            // circle.r = center.Length();
        }
    }
    // For robustness, take the radius to be the distance to the farthest point. (For fast math, adjust to use
    // the above individual circle.r = ...; assignments instead)
    circle.r = Sqrt(Max(circle.pos.DistanceSq(a), circle.pos.DistanceSq(b), circle.pos.DistanceSq(c))) + 1e-5f;
    return circle;
}

bool Circle2D::IsFinite() const
{
	return pos.IsFinite() && MATH_NS::IsFinite(r);
}

bool Circle2D::IsDegenerate() const
{
	return !(r > 0.f) || !pos.IsFinite(); // Peculiar order of testing so that NaNs end up being degenerate.
}

bool Circle2D::Contains(const float2 &point) const
{
	return pos.DistanceSq(point) <= r * r;
}

bool Circle2D::Contains(const float2 &point, float epsilon) const
{
	return pos.DistanceSq(point) <= r * r + epsilon;
}

float Circle2D::Distance(const float2 &point) const
{
    return Max(0.f, pos.Distance(point) - r);
}

float Circle2D::SignedDistance(const float2 &point) const
{
    return pos.Distance(point) - r;
}

float Circle2D::SignedDistanceSq(const float2 &point) const
{
    return pos.DistanceSq(point) - r*r;
}

Circle2D Circle2D::OptimalEnclosingCircle(const float2 *pointArray, int numPoints)
{
    assert(pointArray || numPoints == 0);
    
    // Special case handling for 0-3 points.
    switch(numPoints)
    {
        case 0: return Circle2D(float2::nan, -FLOAT_INF);
        case 1: return Circle2D(pointArray[0], 0.f);
        case 2:
        {
            float2 center = (pointArray[0] + pointArray[1]) * 0.5f;
            float r = Sqrt(Max(center.DistanceSq(pointArray[0]), center.DistanceSq(pointArray[1]))) + 1e-5f;
            return Circle2D(center, r);
        }
        case 3: return Circle2D::OptimalEnclosingCircle(pointArray[0], pointArray[1], pointArray[2]);
    }
    
	// Start off by computing the convex hull of the points, which prunes many points off from the problem space.
	float2 *pts = new float2[numPoints];
	memcpy(pts, pointArray, sizeof(float2)*numPoints);
	numPoints = float2_ConvexHullInPlace(pts, numPoints);

    // Use initial bounding box extents (min/max x and y) as fast guesses for the optimal
    // bounding sphere extents.
    for(int i = 0; i < numPoints; ++i)
    {
        if (pts[0].x < pts[i].x) Swap(pts[0], pts[i]);
        if (pts[1].x > pts[i].x) Swap(pts[1], pts[i]);
        if (pts[2].y < pts[i].y) Swap(pts[2], pts[i]);
        if (pts[3].y > pts[i].y) Swap(pts[3], pts[i]);
    }

    // Compute the minimal enclosing circle for the first three points.
	Circle2D minCircle = OptimalEnclosingCircle(pts[0], pts[1], pts[2]);
    float r2 = minCircle.r*minCircle.r;

    // Iteratively include the remaining points to the minimal circle.
    for(int i = 3; i < numPoints; ++i)
	{
        // If the new point is already inside the current bounding circle, it can be skipped
        float d2 = (pts[i] - minCircle.pos).LengthSq();
		if (d2 <= r2)
			continue;

        // The new point is outside the current bounding circle defined by pts[0]-pts[2].
        // Compute a new bounding circle that encloses pts[0]-pts[2] and the new point pts[i].
        // A circle is defined by at most three points, so one of the resulting points is redundant.
        // Swap points around so that pts[0]-pts[2] define the new minimum circle, and pts[i] will
        // have the redundant point.
        minCircle = SmallestCircleEnclosing4Points(pts[i], pts[0], pts[1], pts[2]);
        minCircle.r += 1e-3f;
        r2 = minCircle.r*minCircle.r;

        // Start again from scratch: pts[0]-pts[2] now has the new candidate.
        i = 2;

        mathassert1(minCircle.Contains(pts[0], 1e-3f), minCircle.SignedDistance(pts[0]));
        mathassert1(minCircle.Contains(pts[1], 1e-3f), minCircle.SignedDistance(pts[1]));
        mathassert1(minCircle.Contains(pts[2], 1e-3f), minCircle.SignedDistance(pts[2]));
	}
	delete[] pts;
	return minCircle;
}

float2 Circle2D::RandomPointInside(LCG &lcg)
{
	assume(r > 1e-3f);
	float2 v = float2::zero;
	// Rejection sampling analysis: The unit circle fills ~78.54% of the area of its enclosing rectangle, so this
	// loop is expected to take only very few iterations before succeeding.
	for (int i = 0; i < 1000; ++i)
	{
		v.x = lcg.Float(-r, r);
		v.y = lcg.Float(-r, r);
#if 0
        // Generate easy test cases
        v.x = (float)(int)v.x;
        v.y = (float)(int)v.y;
#endif
		if (v.LengthSq() <= r * r)
			return pos + v;
	}
	assume(false && "Circle2D::RandomPointInside failed!");

	// Failed to generate a point inside this circle. Return the circle center as fallback.
	return pos;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Circle2D::ToString() const
{
	char str[256];
	sprintf(str, "Circle2D(pos:(%.2f, %.2f) r:%.2f)",
		pos.x, pos.y, r);
	return str;
}
#endif

MATH_END_NAMESPACE
