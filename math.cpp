#include "math.h"

const math::vec<3> math::operator+(math::vec<3> lhs, math::vec<3> rhs)
{
    return math::vec<3>(lhs.e[0] + rhs.e[0],
                        lhs.e[1] + rhs.e[1],
                        lhs.e[2] + rhs.e[2]);
}

const math::vec<3> math::operator+=(math::vec<3>& lhs, math::vec<3> rhs)
{
    lhs = vec<3>(lhs.e[0] + rhs.e[0],
                 lhs.e[1] + rhs.e[1],
                 lhs.e[2] + rhs.e[2]);
    return lhs;
}

const math::vec<3> math::operator-(math::vec<3> lhs, math::vec<3> rhs)
{
   return vec<3>(lhs.e[0] - rhs.e[0],
             lhs.e[1] - rhs.e[1],
             lhs.e[2] - rhs.e[2]);
}
const math::vec<3> math::operator-=(math::vec<3>& lhs, math::vec<3> rhs)
{
    lhs = lhs - rhs;
    return lhs;
}
const math::vec<3> math::operator*(math::vec<3> lhs, math::vec<3> rhs)
{
   return vec<3>(lhs.e[0] * rhs.e[0],
                 lhs.e[1] * rhs.e[1],
                 lhs.e[2] * rhs.e[2]);
}
const math::vec<3> math::operator*=(math::vec<3>& lhs, math::vec<3> rhs)
{
    lhs = lhs * rhs;
    return lhs;
}

const math::vec<3> math::operator*(vec<3> lhs, float rhs)
{
    return math::vec<3>(lhs.e[0] * rhs,
                    lhs.e[1] * rhs,
                    lhs.e[2] * rhs);
}

const math::vec<3> math::operator*=(vec<3>& lhs, float rhs)
{
    lhs = lhs * rhs;
    return lhs;
}

const math::vec<3> math::operator/(vec<3> lhs, vec<3> rhs)
{
    return math::vec<3>(lhs.e[0] / rhs.e[0],
                        lhs.e[1] / rhs.e[1],
                        lhs.e[2] / rhs.e[2]);
}

const math::vec<3> math::operator/=(vec<3>& lhs, vec<3> rhs)
{
    lhs = lhs / rhs;
    return lhs;
}

const math::vec<3> math::operator/(vec<3> lhs, float rhs)
{
    return math::vec<3>(lhs.e[0] / rhs,
                        lhs.e[1] / rhs,
                        lhs.e[2] / rhs);
}

const math::vec<3> math::operator/=(vec<3>& lhs, float rhs)
{
    lhs = lhs / rhs;
    return lhs;
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

math::m3 math::normalSpace(vec<3> n)
{
    // Each possible solution for the [x-basis] assumes some direction is zero and uses that
     // to solve the plane equation exactly orthogonal to the normal direction
     // I.e. given (n.x)x + (n.y)y + (n.z)z = 0 we can resolve the x-basis in three different ways
     // depending on which axis we decide to zero:
     // y = 0 >> (n.x)x + (n.z)z = 0 >> (n.x)x = 0 - (n.z)z >> n.x(x) = -n.z(z) >> (x = n.z, z = -n.x)
     // x = 0 >> (n.y)y + (n.z)z = 0 >> (n.y)y = 0 - (n.z)z >> n.y(y) = -n.z(z) >> (y = n.z, z = -n.y)
     // z = 0 >> (n.x)x + (n.y)y = 0 >> (n.x)x = 0 - (n.y)y >> n.x(x) = -n.y(y) >> (x = n.z, y = -n.x)
     // This assumption becomes less reliable as the direction chosen by any of the solutions gets further
     // and further away from zero; the most effective way to minimize that error is to use the solution
     // corresponding to the smallest direction
     // In other words...
     // When x = min(x, y, z), solve for x = 0
     // When y = min(x, y, z), solve for y = 0
     // When z = min(x, y, z), solve for z = 0
    vec<3> absNormal = vec<3>(std::abs(n.x()), std::abs(n.y()), std::abs(n.z()));
    float minAxis = std::min(std::min(absNormal.x(), absNormal.y()),
                                      absNormal.z());

    // Scratchapixel's minimum-selection function
    vec<3> basisX = vec<3>(0,0,0);
    if (absNormal.x() > absNormal.y())
    {
        basisX = vec<3>(n.z(), 0.0f, n.x() * -1.0f);
    }
    else
    {
        basisX = vec<3>(0.0f, n.z(), n.y() * -1.0f);
    }

    // Build the normal-space from the normal vector and the x-basis we generated before,
    // then return the result
    return m3(basisX.normalized(),
              n,
              n.cross(basisX).normalized());
}

math::vec<3> math::floor(math::vec<3> v)
{
    return math::vec<3>(std::floor(v.e[0]),
                        std::floor(v.e[1]),
                        std::floor(v.e[2]));
}

math::vec<3> math::ceil(math::vec<3> v)
{
    return math::vec<3>(std::ceil(v.e[0]),
                        std::ceil(v.e[1]),
                        std::ceil(v.e[2]));
}

math::vec<3> math::max(math::vec<3> v, math::vec<3> u)
{
    return math::vec<3>(std::max(v.e[0], u.e[0]),
                        std::max(v.e[1], u.e[1]),
                        std::max(v.e[2], u.e[2]));
}

math::vec<3> math::min(math::vec<3> v, math::vec<3> u)
{
    return math::vec<3>(std::min(v.e[0], u.e[0]),
                        std::min(v.e[1], u.e[1]),
                        std::min(v.e[2], u.e[2]));
}

math::vec<3> math::abs(math::vec<3> v)
{
    return math::vec<3>(std::abs(v.e[0]),
                        std::abs(v.e[1]),
                        std::abs(v.e[2]));
}

float math::sgn(float f)
{
    return std::signbit(f) ? -1.0f : 1.0f;
}

bool math::eps_equality(float x, float y)
{
    return x >= (y - eps) &&
           x <= (y + eps);
}
