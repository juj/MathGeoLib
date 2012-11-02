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

/** @file Complex.h
	@author Jukka Jylänki
	@brief */
#pragma once

MATH_BEGIN_NAMESPACE

/// A complex value of form a + bi.
class Complex
{
public:
	/// The default ctor does not initialize the Circle to any value.
	Complex() {}
	Complex(float real, float imaginary);

	float r;
	float i;

	Complex Conjugate() const;
	void Normalize();
	Complex Normalized() const;
	float Length() const;
	float LengthSq() const;

	Complex operator +(float real) const;
	Complex operator +(const Complex &c) const;
	Complex operator -(float real) const;
	Complex operator -(const Complex &c) const;
	Complex operator *(float real) const;
	Complex operator *(const Complex &c) const;
	Complex operator /(float real) const;
	Complex operator /(const Complex &c) const;

	Complex &operator +=(float real);
	Complex &operator +=(const Complex &c);
	Complex &operator -=(float real);
	Complex &operator -=(const Complex &c);
	Complex &operator *=(float real);
	Complex &operator *=(const Complex &c);
	Complex &operator /=(float real);
	Complex &operator /=(const Complex &c);

	static const Complex zero;
	static const Complex unitOne;
	static const Complex unitI;
};

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Complex)
Q_DECLARE_METATYPE(Complex*)
#endif

MATH_END_NAMESPACE
