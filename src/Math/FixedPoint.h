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

/** @file FixedPoint.h
	@author Jukka Jylänki
	@brief FixedPoint is a templatized structure for representing fixed-point decimal numbers. */
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

/** @brief Fixed-precision decimal number.

	BaseT is the base type that is used to hold the value (it should be able to represent negative
	numbers, so consider using a signed integer with either 8,16,32 or 64 bits. FracSize is the number
	of bits to use for the fractional part of the value. */
template<typename BaseT, int FracSize>
class FixedPoint
{
public:
	enum
	{
		One = 1 << FracSize,
		IntBits = sizeof(BaseT)*8 - FracSize - 1, // -1 to take the sign into account.
		FracBits = FracSize,
		MaxVal = (1 << (IntBits))-1, // IntBits-1 to take the sign into account.
		MaxFrac = (1 << FracBits)-1,
		Epsilon = 1
	};

	/// Stores the fixed-point value.
	BaseT value;

	FixedPoint()
	{
	}

	/** Converts an integer value to fixed-point value. */
	FixedPoint(const BaseT &v)
	:value(v << FracBits)
	{
	}

	/** Converts a whole/fractional pair to fixed-point. */
	FixedPoint(const BaseT &whole, const BaseT &frac)
	:value((whole << FracBits) + frac)
	{
	}

	/** Converts a whole, (nomin/denom) combination to fixed-point.
		For example, FixedPoint(2, 1, 3) would produce a fixed-point that corresponds to a rational 2 1/3, or 7/3. */
	FixedPoint(const BaseT &whole, const BaseT &nomin, const BaseT &denom)
	:value((whole << FracBits) + (nomin << FracBits) / denom)
	{
		assert(denom != 0);
	}

	operator double() const
	{
		return value/(double)One;
	}

	operator float() const
	{
		return value/(float)One;
	}

	/// Returns the truncated integer part.
	BaseT Int() const { return value >> FracBits; }

	BaseT Frac() const { return value & MaxFrac; }
};

template<typename T, int F, int F2>
void Add(FixedPoint<T, F> &dst, const FixedPoint<T, F2> &src)
{
	if (F > F2)
		dst.value += src.value << (F - F2);
	else if (F < F2)
		dst.value += src.value >> (F2 - F);
	else
		dst.value += src.value;
}

template<typename T, int F>
void Add(FixedPoint<T, F> &dst, const T &src)
{
	dst.value += src << FixedPoint<T, F>::FracBits;
}

template<typename T, int F, int F2>
void Sub(FixedPoint<T, F> &dst, const FixedPoint<T, F2> &src)
{
	if (F > F2)
		dst.value -= src.value << (F - F2);
	else if (F < F2)
		dst.value -= src.value >> (F2 - F);
	else
		dst.value -= src.value;
}

template<typename T, int F>
void Sub(FixedPoint<T, F> &dst, const T &src)
{
	dst.value -= src.value << FixedPoint<T, F>::FracBits;
}

template<typename T, int F>
void Mul(FixedPoint<T, F> &dst, const T &src)
{
	dst.value *= src;
}

/** The most straightforward (a*b)>>F, but the possible range for a & b is very limited, since
	the intermediate result a*b is so large. */
template<typename T, int F, int F2>
void MulExtraFast(FixedPoint<T, F> &a, const FixedPoint<T, F2> &b)
{
	a.value = (a.value * b.value) >> FixedPoint<T, F2>::FracBits;
}

/** Computes (a >> F/2)*(b >> F/2), which is a lot better than above, but suffers from a really
	small precision error. */
template<typename T, int F, int F2>
void MulFast(FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
// Share the shifting between the two multiplicands: (Causes a slight precision error)
	a.value = (a.value >> ((FixedPoint<T, F2>::FracBits+1)/2)) * (b.value >> (FixedPoint<T, F2>::FracBits/2));
}

/** Precisely multiplies two fixed-point numbers by breaking up the whole calculation into sums so that overflowing
	in the intermediate range is not possible. */
template<typename T, int F>
void MulPrecise(FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	a.value = ((a.Int() * b.Int()) << FixedPoint<T, F>::FracBits) +
		(a.Int() * b.Frac() + a.Frac() * b.Int()) +
		((a.Frac() * b.Frac()) >> (FixedPoint<T, F>::FracBits));
}
/*
template<int F>
void MulPrecise(FixedPoint<__int32, F> &a, const FixedPoint<__int32, F> &b)
{
	const __int32 dst = a.value;
	const __int32 src = b.value;
	if (F >= 32) // the result is wholly in EDX.
	{
		const int shift = F-32;
		__asm {
			mov eax, dst
			imul src
			mov ecx, shift
			sar edx, cl
			mov dst, edx
		};
	}
	else // the result overlaps EDX:EAX.
	{
		const int shift1 = 32 - F;
		const int shift2 = F;
		__asm {
			// Multiply
			mov eax, dst
			imul src

			// Shift bits in edx to proper place
			mov ecx, shift1
			sal edx, cl

			// Shift bits in eax to proper place
			mov ecx, shift2			
			shr eax, cl

			// Combine edx and eax and store the value.
			or edx, eax
			mov dst, edx
		};
	}
	a.value = dst;
}
*/
template<typename T, int F>
void Div(FixedPoint<T, F> &a, const T &b)
{
	a.value /= b;
}

template<typename T, int F, int F2>
void DivExtraFast(FixedPoint<T, F> &a, const FixedPoint<T, F2> &b)
{
	a.value = (a.value << FixedPoint<T, F2>::FracBits) / b.value;
}

template<typename T, int F>
const FixedPoint<T, F> &operator+=(FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	Add(a, b);
	return a;
}

template<typename T, int F>
const FixedPoint<T, F> &operator+=(FixedPoint<T, F> &a, const T &b)
{
	Add(a, b);
	return a;
}

template<typename T, int F>
const FixedPoint<T, F> operator+(const FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	FixedPoint<T, F> t(a);
	t += b;
	return t;
}

template<typename T, int F>
const FixedPoint<T, F> operator+(const FixedPoint<T, F> &a, const T &b)
{
	FixedPoint<T, F> t(a);
	t += b;
	return t;
}

template<typename T, int F>
const FixedPoint<T, F> &operator-=(FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	Sub(a, b);
	return a;
}

template<typename T, int F>
const FixedPoint<T, F> operator-(const FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	FixedPoint<T, F> t(a);
	t -= b;
	return t;
}

template<typename T, int F>
const FixedPoint<T, F> &operator*=(FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	MulPrecise(a, b);
	return a;
}

template<typename T, int F>
const FixedPoint<T, F> operator*(const FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	FixedPoint<T, F> t(a);
	t *= b;
	return t;
}

template<typename T, int F>
bool operator<(const FixedPoint<T, F> &a, const T &b)
{
	return a.Int() < b;
}

template<typename T, int F>
bool operator<(const FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	return a.value < b.value;
}

template<typename T, int F>
bool operator==(const FixedPoint<T, F> &a, const T &b)
{
	return a.value == b << FixedPoint<T, F>::FracBits;
}

template<typename T, int F>
bool operator==(const FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	return a.value == b.value;
}

template<typename T, int F>
bool operator!=(const FixedPoint<T, F> &a, const FixedPoint<T, F> &b)
{
	return !(a == b);
}

#ifdef MATH_ENABLE_STL_SUPPORT
template<typename T, int F>
std::ostream &operator<<(std::ostream &out, const FixedPoint<T, F> &f)
{
//	out << (double)f.value/(double)FixedPoint<T, F>::One;
	if (f < 0 && f.Frac() != 0)
		out << f.Int()+1 << "." << (FixedPoint<T, F>::One-f.Frac()) * 10000 / FixedPoint<T, F>::One;
	else
		out << f.Int() << "." << f.Frac() * 10000 / FixedPoint<T, F>::One;
	return out;
}
#endif

MATH_END_NAMESPACE
