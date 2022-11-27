
#include "vmath.h"
#include <cmath>

namespace vmath
{
    struct m3
    {
        m3(vec<3> _X, vec<3> _Y, vec<3> _Z) : X(_X), Y(_Y), Z(_Z) {}
        m3(float x0, float y0, float z0,
            float x1, float y1, float z1,
            float x2, float y2, float z2) :
            X(vec<3>(x0, y0, z0)),
            Y(vec<3>(x1, y1, z1)),
            Z(vec<3>(x2, y2, z2)) {}
        vec<3> X, Y, Z;
        m3 transposed()
        {
            vec<3> _X(X.e[0], Y.e[0], Z.e[0]);
            vec<3> _Y(X.e[1], Y.e[1], Z.e[1]);
            vec<3> _Z(X.e[2], Y.e[2], Z.e[2]);
            return m3(_X, _Y, _Z);
        }
        vec<3> apply(vec<3> v) // matrix x vector product
        {
            const vec<3> xT(X.x(), Y.x(), Z.x()); // Implicit transpose, we have to be computing rows x columns and not rows x rows
            const vec<3> yT(X.y(), Y.y(), Z.y());
            const vec<3> zT(X.z(), Y.z(), Z.z());
            return vec<3>(v.dot(xT),
                v.dot(yT),
                v.dot(zT));
        }
        m3 chain_with(m3 m) // matrix x matrix proeduct for chained transformations
        {
            m3 rhsT = m.transposed();
            return m3(vec<3>(X.dot(rhsT.X),
                X.dot(rhsT.Y),
                X.dot(rhsT.Z)),
                vec<3>(Y.dot(rhsT.X),
                    Y.dot(rhsT.Y),
                    Y.dot(rhsT.Z)),
                vec<3>(Z.dot(rhsT.X),
                    Z.dot(rhsT.Y),
                    Z.dot(rhsT.Z)));
        }
    };
};

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

