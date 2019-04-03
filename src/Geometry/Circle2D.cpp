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

/** Computes the (s,t) coordinates of the smallest circle that passes through three points (0,0,0), ab and ac.
	@param ab The first point to fit the circle through.
	@param ac The second point to fit the circle through. The third point is hardcoded to (0,0,0). When fitting a circle
		through three points a, b and c, pass in b-a as the parameter ab, and c-a as the parameter ac (i.e. translate
		the coordinate system center to lie at a).
	@param s [out] Outputs the s-coordinate of the circle center (in the 2D barycentric UV convention)
	@param t [out] Outputs the t-coordinate of the circle center (in the 2D barycentric UV convention) To
		compute the actual point, calculate the expression origin + s*ab + t*ac.
	@note The returned circle is one that passes through the three points (0,0,0), ab and ac. It is NOT necessarily the
		smallest circle that encloses these three points!
	@return True if the function succeeded. False on failure. This function fails if the points (0,0,0), ab and ac
		are collinear, in which case there does not exist a circle that passes through the three given points. */
bool FitCircleThroughPoints(const float2 &ab, const float2 &ac, float &s, float &t)
{
	/* The task is to compute the circle that touches the three points
	   a, b and c. (Note that this is not necessarily the minimal radius circle enclosing
	   the points a, b and c!)

	   Denote by p the circle center position, and r the circle radius. If the circle
	   is to run through the points a, b and c, then the center point of the circle
	   must be equidistant to these points, i.e.

	      || p - a || == || p - b || == || p - c ||,

	   or

	      a^2 - 2ap + p^2 == b^2 - 2bp + p^2 == c^2 - 2cp + p^2.

	   Subtracting pairwise, we get

	      (b-a)p == (b^2 - a^2)/2 and       (1)
	      (c-a)p == (c^2 - a^2)/2.          (2)

	   Further, the point p can be represented as 2D barycentric coordinates (s,t) as follows:

	      p == a + s*(b-a) + t*(c-a).        (3)

	   Now, without loss of generality, assume that the point a lies at origin (translate the origin
	   of the coordinate system to be centered at the point a), i.e. make the substitutions
	   A = (0,0), B = b-a, C = c-a, and we have

	      Bp == B^2/2,            (1')
	      Cp == C^2/2 and         (2')
	       p == s*B + t*C.        (3') */

	const float BB = Dot(ab,ab);
	const float CC = Dot(ac,ac);
	const float BC = Dot(ab,ac);

	/* Substitute (3') into (1') and (2'), to obtain a matrix equation

	   ( B^2  BC  ) * (s) = (B^2 / 2)
	   ( BC   C^2 )   (t)   (C^2 / 2)

	   which equals
	
	   (s) = ( B^2  BC  )^-1  *  (B^2 / 2)
	   (t)   ( BC   C^2 )        (C^2 / 2)

	   	Use the formula for inverting a 2x2 matrix, and we have

	   (s) = 1 / (2 * B^2 * C^2 - (BC)^2) * ( C^2   -BC ) *  (B^2)
	   (t)                                  ( -BC   B^2 )    (C^2)
	*/

	float denom = BB*CC - BC*BC;

	if (EqualAbs(denom, 0.f, 5e-3f))
		return false;

	denom = 0.5f / denom; // == 1 / (2 * B^2 * C^2 - (BC)^2)

	s = (CC * BB - BC * CC) * denom;
	t = (CC * BB - BC * BB) * denom;

	return true;
}

// The epsilon value used for enclosing circle computations.
static const float sEpsilon = 1e-4f;

// Uncomment for internal debugging
// #define DEBUG_MINCIRCLE

/** For reference, see http://realtimecollisiondetection.net/blog/?p=20 . */
Circle2D Circle2D::OptimalEnclosingCircle(const float2 &a, const float2 &b, const float2 &c, const float2 &d, int &redundantPoint)
{
	Circle2D circle;

    redundantPoint = 3;
    circle = OptimalEnclosingCircle(a, b, c);
    if (!circle.Contains(d))
    {
        redundantPoint = 2;
        circle = OptimalEnclosingCircle(a, b, d);
        if (!circle.Contains(c))
        {
            redundantPoint = 1;
            circle = OptimalEnclosingCircle(a, c, d);
            if (!circle.Contains(b))
            {
#ifdef DEBUG_MINCIRCLE
                // TODO: By construction, the circle should always go through point 'a', but does not seem to be the case
                LOGI("Distances: %f, %f, %f, %f", OptimalEnclosingCircle(a, b, c).SignedDistance(d),
                     OptimalEnclosingCircle(a, b, d).SignedDistance(c),
                     OptimalEnclosingCircle(a, c, d).SignedDistance(b),
                     OptimalEnclosingCircle(b, c, d).SignedDistance(a)
                     );
                assert(false);
#endif
                redundantPoint = 0;
                circle = OptimalEnclosingCircle(b, c, d);
                circle.r = Max(circle.r, a.Distance(circle.pos) + 1e-3f); // For numerical stability, expand the radius of the circle so it certainly contains the fourth point.
                assume(circle.Contains(a));
            }
        }
    }

	// Allow floating point inconsistency and expand the radius by a small epsilon so that the containment tests
	// really contain the points (note that the points must be sufficiently near enough to the origin)
	circle.r += 2.f*sEpsilon; // We test against one epsilon, so expand using 2 epsilons.

#ifdef MATH_ASSERT_CORRECTNESS
	if (!circle.Contains(a, sEpsilon) || !circle.Contains(b, sEpsilon) || !circle.Contains(c, sEpsilon) || !circle.Contains(d, sEpsilon))
	{
		LOGI("Pos: %s, r: %f", circle.pos.ToString().c_str(), circle.r);
		LOGI("A: %s, dist: %f", a.ToString().c_str(), a.Distance(circle.pos));
		LOGI("B: %s, dist: %f", b.ToString().c_str(), b.Distance(circle.pos));
		LOGI("C: %s, dist: %f", c.ToString().c_str(), c.Distance(circle.pos));
		LOGI("D: %s, dist: %f", d.ToString().c_str(), d.Distance(circle.pos));
		mathassert(false);
	}
    float maxDist = Max(circle.pos.Distance(a), circle.pos.Distance(b), circle.pos.Distance(c), circle.pos.Distance(d));
    int numMaxDistance = 0;
    if (EqualAbs(circle.pos.Distance(a), maxDist, 1e-1f)) ++numMaxDistance;
    if (EqualAbs(circle.pos.Distance(b), maxDist, 1e-1f)) ++numMaxDistance;
    if (EqualAbs(circle.pos.Distance(c), maxDist, 1e-1f)) ++numMaxDistance;
    if (EqualAbs(circle.pos.Distance(d), maxDist, 1e-1f)) ++numMaxDistance;
    assert1(numMaxDistance >= 2, numMaxDistance);
#endif

	return circle;
}

Circle2D Circle2D::OptimalEnclosingCircle(const float2 &a, const float2 &b, const float2 &c)
{
    Circle2D circle;
    float2 ab = b - a;
    float2 ac = c - a;
    float s, t;
    const float BB = Dot(ab,ab);
    const float CC = Dot(ac,ac);
    const float BC = Dot(ab,ac);
    float denom = BB*CC - BC*BC;
        
    if (Abs(denom) < 1e-5f) // Each of a, b and c lie on a straight line?
    {
        if (BC > 0.f)
        {
            if (BB > CC)
            {
                circle.pos = (a+b)*0.5f;
                circle.r = Sqrt(BB)*0.5f;
            }
            else
            {
                circle.pos = (a+c)*0.5f;
                circle.r = Sqrt(CC)*0.5f;
            }
        }
        else
        {
            // ||b-c|| = ||(b-a+a-c)|| == ||(b-a)+(c-a)||==||ab-ac||=Dot(ab,ab)-2Dot(ab,ac)+Dot(ac,ac)
            circle.pos = (b+c)*0.5f;
            circle.r = Sqrt(BB + CC - 2.f*BC)*0.5f;
        }
    }
    else
    {
        denom = 0.5f / denom;
        s = (CC * BB - BC * CC) * denom;
        t = (CC * BB - BC * BB) * denom;
        if (s < 0.f)
        {
            circle.pos = (a + c) * 0.5f;
            circle.r = a.Distance(c) * 0.5f;
        }
        else if (t < 0.f)
        {
            circle.pos = (a + b) * 0.5f;
            circle.r = a.Distance(b) * 0.5f;
        }
        else if (s + t > 1.f)
        {
            circle.pos = (b + c) * 0.5f;
            circle.r = b.Distance(c) * 0.5f;
        }
        else
        {
            const float2 center = s * ab + t * ac;
            circle.pos = a + center;
            circle.r = center.Length();
        }
    }
    // For robustness, take the radius to be the distance to the farthest point. (comment the following line out if fast performance is desired instead)
    circle.r = Sqrt(Max(circle.pos.DistanceSq(a), circle.pos.DistanceSq(b), circle.pos.DistanceSq(c)));
    circle.r += 1e-5f;
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

#ifdef DEBUG_MINCIRCLE_FAILS
    float2 *pts2 = new float2[numPoints];
    memcpy(pts2, pts, sizeof(float2)*numPoints);
#endif
#ifdef DEBUG_MINCIRCLE
    LOGI("%d", numPoints);
    for(int i = 0; i < numPoints; ++i)
        LOGI("%s", pts[i].SerializeToCodeString().c_str());
    LOGI("%d points", numPoints);
#endif
    int numForward = 0;

	Circle2D minCircle = OptimalEnclosingCircle(pts[0], pts[1], pts[2]);
    float r2 = minCircle.r*minCircle.r;
    for(int i = 3; i < numPoints; ++i)
	{
#ifdef DEBUG_MINCIRCLE
        LOGI("Current circle: %s", minCircle.ToString().c_str());
        LOGI("Testing pt idx=%d: %s, dist to current circle: %f", i, pts[i].ToString().c_str(), pts[i].Distance(minCircle.pos) - minCircle.r);
#endif
        // Do a minCircle.Contains(pts[i]) check without recomputing the squared radius of the minimum
        // circle each time.
        float d2 = (pts[i] - minCircle.pos).LengthSq();
		if (d2 <= r2)
			continue;

        float2 newImportantPoint = pts[i];
#ifdef DEBUG_MINCIRCLE
        LOGI("%d is not contained", i);
        LOGI("newImportantPoint: %s, numForward=%d", newImportantPoint.ToString().c_str(), numForward);
#endif
		if (i > numForward)
			pts[i] = pts[numForward];
		for(int j = Min(i, numForward); j > 0; --j)
			pts[j] = pts[j-1];
		pts[0] = newImportantPoint;
		int redundantPoint = 0;
		minCircle = OptimalEnclosingCircle(pts[0], pts[1], pts[2], pts[3], redundantPoint);
        r2 = minCircle.r*minCircle.r;
        float2 redundantPt = pts[redundantPoint];

        for(int j = redundantPoint; j < 3; ++j)
            pts[j] = pts[j+1];
        pts[3] = redundantPt;

        // TODO: This would be faster than the above:
		//Swap(pts[redundantPoint], pts[3]);
		if (i > numForward)
			++numForward;
		i = 3;
        assert(minCircle.Contains(pts[0]));
        assert(minCircle.Contains(pts[1]));
        assert(minCircle.Contains(pts[2]));
        assert(minCircle.Contains(pts[3]));
#ifdef DEBUG_MINCIRCLE
        LOGI("New circle, distances:");
        for(int j = 0; j <= numPoints; ++j)
            LOGI("%s: %f", pts[j].SerializeToString().c_str(), minCircle.SignedDistance(pts[j]));
#endif
	}
#ifdef DEBUG_MINCIRCLE_FAILS
    for(int i = 0; i < numPoints; ++i)
        if (!minCircle.Contains(pts2[i]))
        {
            LOGI("%d", numPoints);
            for(int j = 0; j < numPoints; ++j)
                LOGI("%s", pts2[j].SerializeToCodeString().c_str());
            break;
        }
#endif

#ifdef DEBUG_MINCIRCLE
    LOGI("Final pts:");
    for(int i = 0; i < numPoints; ++i)
        LOGI("%s", pts[i].SerializeToString().c_str());
#endif
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
