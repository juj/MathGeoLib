/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

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

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Complex)
Q_DECLARE_METATYPE(Complex*)
#endif
