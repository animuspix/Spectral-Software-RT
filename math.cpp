#include "math.h"

math::vec<3> math::operator+(math::vec<3> lhs, math::vec<3> rhs)
{
   return math::vec<3>(lhs.e[0] + rhs.e[0],
                       lhs.e[1] + rhs.e[1],
                       lhs.e[2] + rhs.e[2]);
}

const math::vec<3> math::operator+=(math::vec<3> lhs, math::vec<3> rhs)
{
   return vec<3>(lhs.e[0] + rhs.e[0],
             lhs.e[1] + rhs.e[1],
             lhs.e[2] + rhs.e[2]);
}

math::vec<3> math::operator-(math::vec<3> lhs, math::vec<3> rhs)
{
   return vec<3>(lhs.e[0] - rhs.e[0],
             lhs.e[1] - rhs.e[1],
             lhs.e[2] - rhs.e[2]);
}
const math::vec<3> math::operator-=(math::vec<3> lhs, math::vec<3> rhs)
{
   return lhs - rhs;
}
const math::vec<3> math::operator*(math::vec<3> lhs, math::vec<3> rhs)
{
   return vec<3>(lhs.e[0] * rhs.e[0],
                 lhs.e[1] * rhs.e[1],
                 lhs.e[2] * rhs.e[2]);
}
const math::vec<3> math::operator*=(math::vec<3> lhs, math::vec<3> rhs)
{
   return lhs * rhs;
}

const math::vec<3> math::operator*(vec<3> lhs, float rhs)
{
    return math::vec<3>(lhs.e[0] * rhs,
                    lhs.e[1] * rhs,
                    lhs.e[2] * rhs);
}

const math::vec<3> math::operator*=(vec<3> lhs, float rhs)
{
    return lhs * rhs;
}

const math::vec<3> math::operator/(vec<3> lhs, vec<3> rhs)
{
    return math::vec<3>(lhs.e[0] / rhs.e[0],
                        lhs.e[1] / rhs.e[1],
                        lhs.e[2] / rhs.e[2]);
}

const math::vec<3> math::operator/=(vec<3> lhs, vec<3> rhs)
{
    return lhs / rhs;
}

const math::vec<3> math::operator/(vec<3> lhs, float rhs)
{
    return math::vec<3>(lhs.e[0] / rhs,
                        lhs.e[1] / rhs,
                        lhs.e[2] / rhs);
}

const math::vec<3> math::operator/=(vec<3> lhs, float rhs)
{
    return lhs / rhs;
}

float math::polyMulBinomial(float x, float a, float b)
{
   // (ax [operator] b)^2
   // quick, lazy FOIL polynomial expansion
   float ax = a * x;
   float axSqr = ax * ax;
   float axb_2 = (ax * b) * 2.0f;
   float bSqr = b * b;

   // (ax - b)^2 should expand to
   // ax^2 - 2axb + b^2
   return axSqr - axb_2 + bSqr;
}

float math::quadratic(float x, float a, float b, float c, bool inv)
{
   // Solve inner multiply
   float polyNum = polyMulBinomial(x, a, b);

   // Invert, apply translation, return :)
   return inv ? -polyNum + c :
                 polyNum + c;
};

float math::gaussian(float x, float a, float b, float c, float d)
{
   // Solve inner polynomial
   float polyNum = math::polyMulBinomial(x, 1.0f, b);

   // Central fraction (stretch the polynomial + invert it)
   float frac = -(polyNum / (2.0f * c * c));

   // Unit gaussian
   float gauss = std::exp(frac);

   // Scale + translation
   return (a * gauss) - d;
}

