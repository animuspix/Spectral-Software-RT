
#include "vmath.h"
#include <cmath>

vmath::m3 vmath::normalSpace(vmath::vec<3> n)
{
    // Tangent normal function from:
    // https://www.shadertoy.com/view/ltKyzm
    // Originally from Scratchapixel
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/global-illumination-path-tracing/global-illumination-path-tracing-practical-implementation

    // Generate a basis-x direction
    // Different functions will be more/less appropriate
    // depending on the normal's x/y-tendency
    vmath::vec<3> absNormal = vmath::vec<3>(fabs(n.x()), fabs(n.y()), fabs(n.z()));
    vmath::vec<3> basisX = vmath::vec<3>(0, 0, 0);
    if (absNormal.x() >= absNormal.y())
    {
        basisX = vmath::vec<3>(n.z(), 0.0f, n.x() * -1.0f);
    }
    else
    {
        basisX = vmath::vec<3>(0.0f, n.z(), n.y() * -1.0f);
    }

    // Build the normal-space from the normal vector and the x-basis we generated before,
    // then return the result
    return vmath::m3(basisX,
                     n,
                     basisX.cross(n));
}

float vmath::polyMulBinomial(float x, float a, float b)
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

float vmath::quadratic(float x, float a, float b, float c, bool inv)
{
    // Solve inner multiply
    float polyNum = polyMulBinomial(x, a, b);

    // Invert, apply translation, return :)
    return inv ? -polyNum + c :
        polyNum + c;
};

float vmath::gaussian(float x, float a, float b, float c, float d)
{
    // Solve inner polynomial
    float polyNum = polyMulBinomial(x, 1.0f, b);

    // Central fraction (stretch the polynomial + invert it)
    float frac = -(polyNum / (2.0f * c * c));

    // Unit gaussian
    float gauss = fexp(frac);

    // Scale + translation
    return (a * gauss) - d;
}

float vmath::fsin(float a)
{
	return std::sin(a);
}

float vmath::fcos(float a)
{
	return std::cos(a);
}

float vmath::ftan(float a)
{
	return std::tan(a);
}

float vmath::fexp(float f)
{
	return std::exp(f);
}

float vmath::fabs(float a)
{
	return std::abs(a);
}

float vmath::ffloor(float f)
{
	return std::floor(f);
}

float vmath::ffrac(float f)
{
	return f - int(f);
}

float vmath::fceil(float f)
{
	return std::ceil(f);
}

float vmath::fmodf(float fval, float finterval)
{
	return std::fmodf(fval, finterval);
}

float vmath::fsqrt(float fval)
{
	return std::sqrt(fval);
}

