#include "math.h"

const math::v3 math::operator+(math::v3 lhs, math::v3 rhs)
{
   return math::v3(lhs.x + rhs.x,
                   lhs.y + rhs.y,
                   lhs.z + rhs.z);
}

const math::v3 math::operator+=(math::v3 lhs, math::v3 rhs)
{
   return v3(lhs.x + rhs.x,
             lhs.y + rhs.y,
             lhs.z + rhs.z);
}

const math::v3 math::operator-(math::v3 lhs, math::v3 rhs)
{
   return v3(lhs.x - rhs.x,
             lhs.y - rhs.y,
             lhs.z - rhs.z);
}
const math::v3 math::operator-=(math::v3 lhs, math::v3 rhs)
{
   return lhs - rhs;
}
const math::v3 math::operator*(math::v3 lhs, math::v3 rhs)
{
   return v3(lhs.x * rhs.x,
             lhs.y * rhs.y,
             lhs.z * rhs.z);
}
const math::v3 math::operator*=(math::v3 lhs, math::v3 rhs)
{
   return lhs * rhs;
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

