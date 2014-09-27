/* Copyright 2011 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Matrix.inl
	@author Jukka Jylänki
	@brief */
#pragma once

#include <utility>
#include "Quat.h"

MATH_BEGIN_NAMESPACE

/** Sets the top-left 3x3 area of the matrix to the rotation matrix about the X-axis. Elements
	outside the top-left 3x3 area are ignored. This matrix rotates counterclockwise if multiplied
	in the order M*v, and clockwise if rotated in the order v*M.
	@param m The matrix to store the result.
	@param angle The rotation angle in radians. */
template<typename Matrix>
void Set3x3PartRotateX(Matrix &m, float angle)
{
	float sinz, cosz;
	SinCos(angle, sinz, cosz);
	
	m[0][0] = 1.f; m[0][1] =  0.f; m[0][2] =   0.f;
	m[1][0] = 0.f; m[1][1] = cosz; m[1][2] = -sinz;
	m[2][0] = 0.f; m[2][1] = sinz; m[2][2] =  cosz;
}

/** Sets the top-left 3x3 area of the matrix to the rotation matrix about the Y-axis. Elements
	outside the top-left 3x3 area are ignored. This matrix rotates counterclockwise if multiplied
	in the order M*v, and clockwise if rotated in the order v*M.
	@param m The matrix to store the result.
	@param angle The rotation angle in radians. */
template<typename Matrix>
void Set3x3PartRotateY(Matrix &m, float angle)
{
	float sinz, cosz;
	SinCos(angle, sinz, cosz);

	m[0][0] =  cosz; m[0][1] = 0.f; m[0][2] = sinz;
	m[1][0] =   0.f; m[1][1] = 1.f; m[1][2] =  0.f;
	m[2][0] = -sinz; m[2][1] = 0.f; m[2][2] = cosz;
}

/** Sets the top-left 3x3 area of the matrix to the rotation matrix about the Z-axis. Elements
	outside the top-left 3x3 area are ignored. This matrix rotates counterclockwise if multiplied
	in the order M*v, and clockwise if rotated in the order v*M.
	@param m The matrix to store the result.
	@param angle The rotation angle in radians. */
template<typename Matrix>
void Set3x3PartRotateZ(Matrix &m, float angle)
{
	float sinz, cosz;
	SinCos(angle, sinz, cosz);

	m[0][0] = cosz; m[0][1] = -sinz; m[0][2] = 0.f;
	m[1][0] = sinz; m[1][1] =  cosz; m[1][2] = 0.f;
	m[2][0] =  0.f; m[2][1] =   0.f; m[2][2] = 1.f;
}

/** Computes the matrix M = R_x * R_y * R_z, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x, y and z are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerXYZ(Matrix &m, float x, float y, float z)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx, sx, cy, sy, cz, sz;
	SinCos(x, sx, cx);
	SinCos(y, sy, cy);
	SinCos(z, sz, cz);

	m[0][0] =             cy*cz; m[0][1] =         -cy * sz; m[0][2] =     sy;
	m[1][0] =  cz*sx*sy + cx*sz; m[1][1] = cx*cz - sx*sy*sz; m[1][2] = -cy*sx;
	m[2][0] = -cx*cz*sy + sx*sz; m[2][1] = cz*sx + cx*sy*sz; m[2][2] =  cx*cy;
}

/** Decomposes the matrix M to form M = R_x * R_y * R_z (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x, y and z, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerXYZ(Matrix &m, float &x, float &y, float &z)
{
	if (m[0][2] < 1.f)
	{
		if (m[0][2] > -1.f)
		{
			y = asin(m[0][2]);
			x = atan2(-m[1][2], m[2][2]);
			z = atan2(-m[0][1], m[0][0]);
		}
		else
		{
			// Not a unique solution: z - x = atan2(m[1][0], m[1][1]);
			y = -pi/2.f;
			x = -atan2(m[1][0], m[1][1]);
			z = 0.f;
		}
	}
	else
	{
		// Not a unique solution: z + x = atan2(m[1][0], m[1][1]);
		y = pi/2.f;
		x = atan2(m[1][0], m[1][1]);
		z = 0.f;
	}
}

/** Computes the matrix M = R_x * R_z * R_y, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x, y and z are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerXZY(Matrix &m, float &x, float &z, float &y)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx, sx, cy, sy, cz, sz;
	SinCos(x, sx, cx);
	SinCos(y, sy, cy);
	SinCos(z, sz, cz);

	m[0][0] =             cy*cz; m[0][1] =   -sz; m[0][2] =             cz*sy;
	m[1][0] =  sx*sy + cx*cy*sz; m[1][1] = cx*cz; m[1][2] = -cy*sx + cx*sy*sz;
	m[2][0] = -cx*sy + cy*sx*sz; m[2][1] = cz*sx; m[2][2] =  cx*cy + sx*sy*sz;
}

/** Decomposes the matrix M to form M = R_x * R_z * R_y (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x, y and z, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerXZY(Matrix &m, float &x, float &z, float &y)
{
	if (m[0][1] < 1.f)
	{
		if (m[0][1] > -1.f)
		{
			z = asin(-m[0][1]);
			x = atan2(m[2][1], m[1][1]);
			y = atan2(m[0][2], m[0][0]);
		}
		else
		{
			// Not a unique solution: y - x = atan2(-m[2][0], m[2][2]);
			z = pi/2.f;
			x = atan2(-m[2][0], m[2][2]);
			y = 0;
		}
	}
	else
	{
		// Not a unique solution: y + x = atan2(-m[2][0], m[2][2]);
		z = -pi/2.f;
		x = atan2(-m[2][0], m[2][2]);
		y = 0;
	}
}

/** Computes the matrix M = R_y * R_x * R_z, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x, y and z are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerYXZ(Matrix &m, float &y, float &x, float &z)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx, sx, cy, sy, cz, sz;
	SinCos(x, sx, cx);
	SinCos(y, sy, cy);
	SinCos(z, sz, cz);

	m[0][0] =  cy*cz + sx*sy*sz; m[0][1] = cx*sx*sy - cy*sz; m[0][2] = cx*sy;
	m[1][0] =             cx*sz; m[1][1] = cx*cz;            m[1][2] =   -sx;
	m[2][0] = -cz*sy + cy*sx*sz; m[2][1] = cy*cz*sx + sy*sz; m[2][2] = cx*cy;
}

/** Decomposes the matrix M to form M = R_y * R_x * R_z (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x, y and z, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerYXZ(Matrix &m, float &y, float &x, float &z)
{
	if (m[1][2] < 1.f)
	{
		if (m[1][2] > -1.f)
		{
			x = asin(-m[1][2]);
			y = atan2(m[0][2], m[2][2]);
			z = atan2(m[1][0], m[1][1]);
		}
		else
		{
			// Not a unique solution.
			x = pi/2.f;
			y = -atan2(-m[0][1], m[0][0]);
			z = 0;
		}
	}
	else
	{
		// Not a unique solution.
		x = -pi/2.f;
		y = atan2(-m[0][1], m[0][0]);
		z = 0;
	}
}

/** Computes the matrix M = R_y * R_z * R_x, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x, y and z are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerYZX(Matrix &m, float &y, float &z, float &x)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx, sx, cy, sy, cz, sz;
	SinCos(x, sx, cx);
	SinCos(y, sy, cy);
	SinCos(z, sz, cz);

	m[0][0] =  cy*cz; m[0][1] = sx*sy - cx*cy*sz; m[0][2] = cx*sy + cy*sx*sz;
	m[1][0] =     sz; m[1][1] =            cx*cz; m[1][2] =           -cz*sx;
	m[2][0] = -cx*sy; m[2][1] = cy*sx + cx*sy*sz; m[2][2] = cx*cy - sx*sy*sz;
}

/** Decomposes the matrix M to form M = R_y * R_z * R_x (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x, y and z, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerYZX(Matrix &m, float &y, float &z, float &x)
{
	if (m[1][0] < 1.f)
	{
		if (m[1][0] > -1.f)
		{
			z = asin(m[1][0]);
			y = atan2(m[2][0], m[0][0]);
			x = atan2(m[1][2], m[1][1]);
		}
		else
		{
			// Not a unique solution.
			z = -pi/2.f;
			y = -atan2(m[2][1], m[2][2]);
			x = 0;
		}
	}
	else
	{
		// Not a unique solution.
		z = pi/2.f;
		y = atan2(-m[2][1], m[2][2]);
		x = 0;
	}
}

/** Computes the matrix M = R_z * R_x * R_y, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x, y and z are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerZXY(Matrix &m, float &z, float &x, float &y)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx, sx, cy, sy, cz, sz;
	SinCos(x, sx, cx);
	SinCos(y, sy, cy);
	SinCos(z, sz, cz);

	m[0][0] = cy*cz - sx*sy*sz; m[0][1] = -cx*sz; m[0][2] =  cz*sy + cy*sx*sz;
	m[1][0] = cz*sx*sy + cy*sz; m[1][1] =  cx*cz; m[1][2] = -cy*cz*sx + sy*sz;
	m[2][0] =           -cx*sy; m[2][1] =     sx; m[2][2] =             cx*cy;
}

/** Decomposes the matrix M to form M = R_z * R_x * R_y (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x, y and z, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerZXY(const Matrix &m, float &z, float &x, float &y)
{
	if (m[2][1] < 1.f)
	{
		if (m[2][1] > -1.f)
		{
			x = asin(m[2][1]);
			z = atan2(-m[0][1], m[1][1]);
			y = atan2(-m[2][0], m[2][2]);
		}
		else
		{
			// Not a unique solution.
			x = -pi/2.f;
			z = -atan2(-m[0][2], m[0][0]);
			y = 0;
		}
	}
	else
	{
		// Not a unique solution.
		x = pi/2.f;
		z = atan2(m[0][2], m[0][0]);
		y = 0;
	}
}

/** Computes the matrix M = R_z * R_y * R_x, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x, y and z are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerZYX(Matrix &m, float &z, float &y, float &x)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx, sx, cy, sy, cz, sz;
	SinCos(x, sx, cx);
	SinCos(y, sy, cy);
	SinCos(z, sz, cz);

	m[0][0] = cy*cz; m[0][1] = cz*sx*sy - cx*sz; m[0][2] =  cx*cz*sy + sx*sz;
	m[1][0] = cy*sz; m[1][1] = cx*cz + sx*sy*sz; m[1][2] = -cz*sx + cx*sy*sz;
	m[2][0] =   -sy; m[2][1] =            cy*sx; m[2][2] =             cx*cy;
}

/** Decomposes the matrix M to form M = R_z * R_y * R_x (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x, y and z, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerZYX(Matrix &m, float &z, float &y, float &x)
{
	if (m[2][0] < 1.f)
	{
		if (m[2][0] > -1.f)
		{
			y = asin(-m[2][0]);
			z = atan2(m[1][0], m[0][0]);
			x = atan2(m[2][1], m[2][2]);
		}
		else
		{
			// Not a unique solution.
			y = pi/2.f;
			z = -atan2(-m[1][2], m[1][1]);
			x = 0;
		}
	}
	else
	{
		// Not a unique solution.
		y = -pi/2.f;
		z = atan2(-m[1][2], m[1][1]);
		x = 0;
	}
}

/** Computes the matrix M = R_x2 * R_y * R_x1, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x2, y and x1 are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerXYX(Matrix &m, float &x2, float &y, float &x1)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx2, sx2, cy, sy, cx1, sx1;
	SinCos(x2, sx2, cx2);
	SinCos(y, sy, cy);
	SinCos(x1, sx1, cx1);

	m[0][0] =      cy; m[0][1] =               sy*sx1; m[0][2] =                sy*cx1;
	m[1][0] =  sy*sx2; m[1][1] = cx2*cx1 - cy*sx2*sx1; m[1][2] = -cy*cx1*sx2 - cx2*sx1;
	m[2][0] = -sy*cx2; m[2][1] = cx1*sx2 + cy*cx2*sx1; m[2][2] =  cy*cx2*cx1 - sx2*sx1;
}

/** Decomposes the matrix M to form M = R_x2 * R_y * R_x1 (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x2, y and x1, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerXYX(Matrix &m, float &x2, float &y, float &x1)
{
	if (m[0][0] < 1.f)
	{
		if (m[0][0] > -1.f)
		{
			y = acos(m[0][0]);
			x2 = atan2(m[1][0], -m[2][0]);
			x1 = atan2(m[0][1], m[0][2]);
		}
		else
		{
			// Not a unique solution.
			y = pi;
			x2 = -atan2(-m[1][2], m[1][1]);
			x1 = 0;
		}
	}
	else
	{
		// Not a unique solution.
		y = 0;
		x2 = atan2(-m[1][2], m[1][1]);
		x1 = 0;
	}
}

/** Computes the matrix M = R_x2 * R_z * R_x1, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters x2, z and x1 are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerXZX(Matrix &m, float &x2, float &z, float &x1)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cx2, sx2, cz, sz, cx1, sx1;
	SinCos(x2, sx2, cx2);
	SinCos(z, sz, cz);
	SinCos(x1, sx1, cx1);

	m[0][0] =     cz; m[0][1] =              -sz*cx1; m[0][2] =                sz*sx1;
	m[1][0] = sz*cx2; m[1][1] = cz*cx2*cx1 - sx2*sx1; m[1][2] = -cx1*sx2 - cz*cx2*sx1;
	m[2][0] = sz*sx2; m[2][1] = cz*cx1*sx2 + cx2*sx1; m[2][2] =  cx2*cx1 - cz*sx2*sx1;
}

/** Decomposes the matrix M to form M = R_x2 * R_z * R_x1 (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters x2, z and x1, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerXZX(Matrix &m, float &x2, float &z, float &x1)
{
	if (m[0][0] < 1.f)
	{
		if (m[0][0] > -1.f)
		{
			z = acos(m[0][0]);
			x2 = atan2(m[2][0], m[1][0]);
			x1 = atan2(m[0][2], -m[0][1]);
		}
		else
		{
			// Not a unique solution.
			z = pi;
			x2 = -atan2(-m[2][1], m[2][2]);
			x1 = 0;
		}
	}
	else
	{
		// Not a unique solution.
		z = 0;
		x2 = atan2(m[2][1], m[2][2]);
		x1 = 0;
	}
}

/** Computes the matrix M = R_y2 * R_x * R_y1, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters y2, x and y1 are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerYXY(Matrix &m, float &y2, float &x, float &y1)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cy2, sy2, cx, sx, cy1, sy1;
	SinCos(y2, sy2, cy2);
	SinCos(x, sx, cx);
	SinCos(y1, sy1, cy1);

	m[0][0] =  cy2*cy1 - cx*sy2*sy1; m[0][1] = sx*sy2; m[0][2] = cx*cy1*sy2 + cy2*sy1;
	m[1][0] =                sx*sy1; m[1][1] =     cx; m[1][2] =              -sx*cy1;
	m[2][0] = -cy1*sy2 - cx*cy2*sy1; m[2][1] = sx*cy2; m[2][2] = cx*cy2*cy1 - sy2*sy1;
}

/** Decomposes the matrix M to form M = R_y2 * R_x * R_y1 (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters y2, x and y1, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerYXY(Matrix &m, float &y2, float &x, float &y1)
{
	if (m[1][1] < 1.f)
	{
		if (m[1][1] > -1.f)
		{
			x = acos(m[1][1]);
			y2 = atan2(m[0][1], m[2][1]);
			y1 = atan2(m[1][0], -m[1][2]);
		}
		else
		{
			// Not a unique solution.
			x = pi;
			y2 = -atan2(m[0][2], m[0][0]);
			y1 = 0;
		}
	}
	else
	{
		// Not a unique solution.
		x = 0;
		y2 = atan2(m[2][0], m[0][0]);
		y1 = 0;
	}
}

/** Computes the matrix M = R_y2 * R_z * R_y1, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters y2, z and y1 are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerYZY(Matrix &m, float &y2, float &z, float &y1)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cy2, sy2, cz, sz, cy1, sy1;
	SinCos(y2, sy2, cy2);
	SinCos(z, sz, cz);
	SinCos(y1, sy1, cy1);

	m[0][0] =  cz*cy2*cy1 - sy2*sy1; m[0][1] = -sz*cy2; m[0][2] = cy1*sy2 + cz*cy2*sy1;
	m[1][0] =                sz*cy1; m[1][1] =      cz; m[1][2] =               sz*sy1;
	m[2][0] = -cz*cy1*sy2 - cy2*sy1; m[2][1] =  sz*sy2; m[2][2] = cy2*cy1 - cz*sy2*sy1;
}

/** Decomposes the matrix M to form M = R_y2 * R_z * R_y1 (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters y2, z and y1, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerYZY(Matrix &m, float &y2, float &z, float &y1)
{
	if (m[1][1] < 1.f)
	{
		if (m[1][1] > -1.f)
		{
			z = acos(m[1][1]);
			y2 = atan2(m[2][1], -m[0][1]);
			y1 = atan2(m[1][2], m[1][0]);
		}
		else
		{
			// Not a unique solution.
			z = pi;
			y2 = -atan2(-m[2][0], m[2][2]);
			y1 = 0;
		}
	}
	else
	{
		// Not a unique solution.
		z = 0;
		y2 = atan2(-m[2][0], m[2][2]);
		y1 = 0;
	}
}

/** Computes the matrix M = R_z2 * R_x * R_z1, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters z2, x and z1 are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerZXZ(Matrix &m, float &z2, float &x, float &z1)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cz2, sz2, cx, sx, cz1, sz1;
	SinCos(z2, sz2, cz2);
	SinCos(x, sx, cx);
	SinCos(z1, sz1, cz1);

	m[0][0] = cz2*cz1 - cx*sz2*sz1; m[0][1] = -cx*cz1*sz2 - cz2*sz1; m[0][2] =  sx*sz2;
	m[1][0] = cz1*sz2 + cx*cz2*sz1; m[1][1] =  cx*cz2*cz1 - sz2*sz1; m[1][2] = -sx*cz2;
	m[2][0] =               sx*sz1; m[2][1] =                sx*cz1; m[2][2] =      cx;
}

/** Decomposes the matrix M to form M = R_z2 * R_x * R_z1 (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters z2, x and z1, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerZXZ(Matrix &m, float &z2, float &x, float &z1)
{
	if (m[2][2] < 1.f)
	{
		if (m[2][2] > -1.f)
		{
			x = acos(m[2][2]);
			z2 = atan2(m[0][2], -m[1][2]);
			z1 = atan2(m[2][0], m[2][1]);
		}
		else
		{
			// Not a unique solution.
			x = pi;
			z2 = -atan2(-m[0][1], m[0][0]);
			z1 = 0;
		}
	}
	else
	{
		// Not a unique solution.
		x = 0;
		z2 = atan2(-m[0][1], m[0][0]);
		z1 = 0;
	}
}

/** Computes the matrix M = R_z2 * R_y * R_z1, where R_d is the cardinal rotation matrix
	about the axis +d, rotating counterclockwise.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf .
	Parameters z2, y and z1 are the angles of rotation, in radians. */
template<typename Matrix>
void Set3x3PartRotateEulerZYZ(Matrix &m, float &z2, float &y, float &z1)
{
	///\todo Vectorize to compute 4 sines+cosines at one time.
	float cz2, sz2, cy, sy, cz1, sz1;
	SinCos(z2, sz2, cz2);
	SinCos(y, sy, cy);
	SinCos(z1, sz1, cz1);

	m[0][0] = cy*cz2*cz1 - sz2*sz1; m[0][1] = -cz1*sz2 - cy*cz2*sz1; m[0][2] = sy*cz2;
	m[1][0] =           cy*cz1*sz2; m[1][1] =  cz2*cz1 - cy*sz2*sz1; m[1][2] = sy*sz2;
	m[2][0] =              -sy*cz1; m[2][1] =                sy*sz1; m[2][2] =     cy;
}

/** Decomposes the matrix M to form M = R_z2 * R_y * R_z1 (R_d being the cardinal rotation
	matrix about the axis +d, rotating counterclockwise), and outputs the angles of rotation in
	parameters z2, y and z1, in radians.
	This function was adapted from http://www.geometrictools.com/Documentation/EulerAngles.pdf
	The matrix m must be orthonormal. */
template<typename Matrix>
void ExtractEulerZYZ(Matrix &m, float &z2, float &y, float &z1)
{
	if (m[2][2] < 1.f)
	{
		if (m[2][2] > -1.f)
		{
			y = acos(m[2][2]);
			z2 = atan2(m[1][2], m[0][2]);
			z1 = atan2(m[2][1], -m[2][0]);
		}
		else
		{
			// Not a unique solution.
			y = pi;
			z2 = -atan2(-m[1][0], m[1][1]);
			z1 = 0;
		}
	}
	else
	{
		// Not a unique solution.
		y = 0;
		z2 = atan2(m[1][0], m[1][1]);
		z1 = 0;
	}
}

/** Sets the top-left 3x3 area of the matrix to the rotation matrix about an arbitrary axis. Elements
	outside the top-left 3x3 area are not modified.
	@param m The matrix to store the result.
	@param a The axis to rotate about. The axis must be normalized.
	@param angle The rotation angle. */
template<typename Matrix, typename Vector>
void SetRotationAxis3x3(Matrix &m, const Vector &a, float angle)
{
	assume(a.IsNormalized());
	assume(MATH_NS::IsFinite(angle));

	float s, c;
	SinCos(angle, s, c);

	const float c1 = 1.f - c;

	m[0][0] = c+c1*a.x*a.x;
	m[1][0] = c1*a.x*a.y+s*a.z;
	m[2][0] = c1*a.x*a.z-s*a.y;

	m[0][1] = c1*a.x*a.y-s*a.z;
	m[1][1] = c+c1*a.y*a.y;
	m[2][1] = c1*a.y*a.z+s*a.x;

	m[0][2] = c1*a.x*a.z+s*a.y;
	m[1][2] = c1*a.y*a.z-s*a.x;
	m[2][2] = c+c1*a.z*a.z;
}

template<typename Matrix>
void SetIdentity3x3(Matrix &m)
{
	m[0][0] = 1.f; m[0][1] = 0.f; m[0][2] = 0.f;
	m[1][0] = 0.f; m[1][1] = 1.f; m[1][2] = 0.f;
	m[2][0] = 0.f; m[2][1] = 0.f; m[2][2] = 1.f;
}

template<typename Matrix>
void InverseAffineMatrixNoScale(Matrix &mat)
{
	Swap(mat[0][1], mat[1][0]);
	Swap(mat[0][2], mat[2][0]);
	Swap(mat[1][2], mat[2][1]);
	mat.SetTranslatePart(mat.TransformDir(-mat[0][3], -mat[1][3], -mat[2][3]));
}

template<typename Matrix>
void InverseAffineMatrixUniformScale(Matrix &mat)
{
	Swap(mat[0][1], mat[1][0]);
	Swap(mat[0][2], mat[2][0]);
	Swap(mat[1][2], mat[2][1]);
	float scale = sqrtf(1.f / float3(mat[0][0], mat[0][1], mat[0][2]).LengthSq());

	mat[0][0] *= scale; mat[0][1] *= scale; mat[0][2] *= scale;
	mat[1][0] *= scale; mat[1][1] *= scale; mat[1][2] *= scale;
	mat[2][0] *= scale; mat[2][1] *= scale; mat[2][2] *= scale;

	mat.SetTranslatePart(mat.TransformDir(-mat[0][3], -mat[1][3], -mat[2][3]));
}

template<typename Matrix>
void InverseAffineMatrixNonuniformScale(Matrix &mat)
{
	Swap(mat[0][1], mat[1][0]);
	Swap(mat[0][2], mat[2][0]);
	Swap(mat[1][2], mat[2][1]);
	for(int i = 0; i < 3; ++i)
	{
		float scale = sqrtf(1.f / float3(mat[i][0], mat[i][1], mat[i][2]).LengthSq());
		mat[i][0] *= scale;
		mat[i][1] *= scale;
		mat[i][2] *= scale;
	}
	mat.SetTranslatePart(mat.TransformDir(-mat[0][3], -mat[1][3], -mat[2][3]));
}

template<typename Matrix>
bool InverseMatrix(Matrix &mat, float epsilon)
{
	Matrix inversed = Matrix::identity; // will contain the inverse matrix

	const int nc = Min<int>(Matrix::Rows, Matrix::Cols);
	for(int column = 0; column < nc; ++column)
	{
		// find the row i with i >= j such that M has the largest absolute value.
		int greatest = column;
		float greatestVal = Abs(mat[greatest][column]);
		for(int i = column+1; i < Matrix::Rows; i++)
		{
			float val = Abs(mat[i][column]);
			if (val > greatestVal)
			{
				greatest = i;
				greatestVal = val;
			}
		}

		if (greatestVal < epsilon)
		{
			mat = inversed;
			return false;
		}

		// exchange rows
		if (greatest != column)
		{
			inversed.SwapRows(greatest, column);
			mat.SwapRows(greatest, column);
		}
		
		// multiply rows
		assume(!EqualAbs(mat[column][column], 0.f, epsilon));
		float scale = 1.f / mat[column][column];
		inversed.ScaleRow(column, scale);
		mat.ScaleRow(column, scale);
		
		// add rows
		for(int i = 0; i < column; i++)
		{
			inversed.SetRow(i, inversed.Row(i) - inversed.Row(column) * mat[i][column]);
			mat.SetRow(i, mat.Row(i) - mat.Row(column) * mat[i][column]);
		}
		for(int i = column+1; i < Matrix::Rows; i++)
		{
			inversed.SetRow(i, inversed.Row(i) - inversed.Row(column) * mat[i][column]);
			mat.SetRow(i, mat.Row(i) - mat.Row(column) * mat[i][column]);
		}
	}
	mat = inversed;

	return true;
}

/** Computes the LU-decomposition on the given square matrix.
	@return True if the decomposition was successful, false otherwise. If the return value is false, the
		contents of the output matrix are unspecified.*/
template<typename Matrix>
bool LUDecomposeMatrix(const Matrix &mat, Matrix &lower, Matrix &upper)
{
	lower = Matrix::identity;
	upper = Matrix::zero;

	for(int i = 0; i < Matrix::Rows; ++i)
	{
		for(int col = i; col < Matrix::Cols; ++col)
		{
			upper[i][col] = mat[i][col];
			for(int k = 0; k < i; ++k)
				upper[i][col] -= lower[i][k]*upper[k][col];
		}
		for(int row = i+1; row < Matrix::Rows; ++row)
		{
			lower[row][i] = mat[row][i];
			for(int k = 0; k < i; ++k)
				lower[row][i] -= lower[row][k]*upper[k][i];
			if (EqualAbs(upper[i][i], 0.f))
				return false;
			lower[row][i] /= upper[i][i];
		}
	}
	return true;
}

/** Computes the Cholesky decomposition on the given square matrix *on the real domain*.
	@return True if succesful, false otherwise. If the return value is false, the contents of the output
		matrix are unspecified. */
template<typename Matrix>
bool CholeskyDecomposeMatrix(const Matrix &mat, Matrix &lower)
{
	lower = Matrix::zero;
	for(int i = 0; i < Matrix::Rows; ++i)
	{
		for(int j = 0; j < i; ++j)
		{
			lower[i][j] = mat[i][j];
			for(int k = 0; k < j; ++k)
				lower[i][j] -= lower[i][k]*lower[j][k];
			if (EqualAbs(lower[j][j], 0.f))
				return false;
			lower[i][j] /= lower[j][j];
		}

		lower[i][i] = mat[i][i];
		if (lower[i][i])
			return false;
		for(int k = 0; k < i; ++k)
			lower[i][i] -= lower[i][k]*lower[i][k];
		lower[i][i] = Sqrt(lower[i][i]);
	}
	return true;
}

template<typename Matrix>
void SetMatrixRotatePart(Matrix &m, const Quat &q)
{
	// See e.g. http://www.geometrictools.com/Documentation/LinearAlgebraicQuaternions.pdf .

	assume2(q.IsNormalized(1e-3f), q.ToString(), q.LengthSq());
	const float x = q.x; const float y = q.y; const float z = q.z; const float w = q.w;
	m[0][0] = 1 - 2*(y*y + z*z); m[0][1] =     2*(x*y - z*w); m[0][2] =     2*(x*z + y*w);
	m[1][0] =     2*(x*y + z*w); m[1][1] = 1 - 2*(x*x + z*z); m[1][2] =     2*(y*z - x*w);
	m[2][0] =     2*(x*z - y*w); m[2][1] =     2*(y*z + x*w); m[2][2] = 1 - 2*(x*x + y*y);
}

/// See e.g. http://en.wikipedia.org/wiki/Transformation_matrix#reflection
template<typename Matrix>
void SetMatrix3x3LinearPlaneMirror(Matrix &m, float x, float y, float z)
{
	assume(float3(x,y,z).IsNormalized());
	m[0][0] = 1.f - 2.f*x*x; m[0][1] =      -2.f*y*x; m[0][2] =      -2.f*z*x;
	m[1][0] =      -2.f*x*y; m[1][1] = 1.f - 2.f*y*y; m[1][2] =      -2.f*y*z;
	m[2][0] =      -2.f*x*z; m[2][1] =      -2.f*y*z; m[2][2] = 1.f - 2.f*z*z;
}

template<typename Matrix>
void SetMatrix3x4AffinePlaneMirror(Matrix &m, float x, float y, float z, float d)
{
	SetMatrix3x3LinearPlaneMirror(m, x, y, z);
	m[0][3] = 2.f * d * x;
	m[1][3] = 2.f * d * y;
	m[2][3] = 2.f * d * z;
}

template<typename Matrix>
void SetMatrix3x3LinearPlaneProject(Matrix &m, float x, float y, float z)
{
	assume(float3(x,y,z).IsNormalized());
	m[0][0] = 1.f - x*x; m[0][1] =      -y*x; m[0][2] =      -z*x;
	m[1][0] =      -x*y; m[1][1] = 1.f - y*y; m[1][2] =      -y*z;
	m[2][0] =      -x*z; m[2][1] =      -y*z; m[2][2] = 1.f - z*z;
}

template<typename Matrix>
void SetMatrix3x4AffinePlaneProject(Matrix &m, float x, float y, float z, float d)
{
	SetMatrix3x3LinearPlaneProject(m, x, y, z);
	m[0][3] = d * x;
	m[1][3] = d * y;
	m[2][3] = d * z;
}

MATH_END_NAMESPACE
