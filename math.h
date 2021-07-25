#pragma once

#ifdef _DEBUG
#include <assert.h>
#endif

#define _USE_MATH_DEFINES
#include <cmath>
#include <concepts>
#include <ranges>
#include "utils.h"

namespace math
{
    // Linear algebra classes
    template<int dim> requires (dim < 5 && dim > 1)
        struct vec
    {
        vec<dim>()
        {
            memset(e, 0x0, sizeof(float) * dim);
        }
        vec<dim>(float _x, float _y)
        {
            static_assert(dim == 2, "two-dimensional constructor used with higher-dimensional vector");
            e[0] = _x;
            e[1] = _y;
        }
        vec<dim>(float _x, float _y, float _z)
        {
            static_assert(dim == 3, "three-dimensional constructor used with differently-dimensional vector");
            e[0] = _x;
            e[1] = _y;
            e[2] = _z;
        }
        vec<dim>(float _x, float _y, float _z, float _w)
        {
            static_assert(dim == 4, "four-dimensional constructor used with lower-dimensional vector");
            e[0] = _x;
            e[1] = _y;
            e[2] = _z;
            e[3] = _w;
        }
        float e[dim] = {};
        float dot(vec<dim> u)
        {
            float ret = 0;
            for (uint32_t i : countRange(0, dim))
            {
                ret += u.e[i] * e[i];
            }
            return ret;
        }
        vec<3> cross(vec<3> u)
        {
            static_assert(dim == 3, "The cross product is only meaningful in three dimensions - the wedge product is interesting & more general but i haven't found a use-case for it here yet");
            vec<3> leftMuls(e[0] * u.e[1],
                e[1] * u.e[2],
                e[2] * u.e[0]);
            vec<3> rightMuls(e[1] * u.e[0],
                e[2] * u.e[1],
                e[0] * u.e[2]);
            return vec<3>(leftMuls.e[0] - rightMuls.e[0],
                leftMuls.e[1] - rightMuls.e[1],
                leftMuls.e[2] - rightMuls.e[2]);
        }
        const float magnitude() const
        {
            float acc = 0;
            for (uint32_t i : countRange(0, dim))
            {
                acc += e[i] * e[i];
            }
            return sqrt(acc);
        }
        vec<dim> normalized()
        {
            float len = magnitude();
            vec<dim> d = *this;
            for (uint32_t i : countRange(0, dim)) d.e[i] /= len;
            return d;
        }
        const float x() const
        {
            return e[0];
        }
        const float y() const
        {
            return e[1];
        }
        const float z() const
        {
            static_assert(dim >= 3, "no z-coordinate available");
            return e[2];
        }
        const float w() const
        {
            static_assert(dim == 4, "no w-coordinate available");
            return e[3];
        }
        const vec<2> xy() const
        {
            return vec<2>(e[0], e[1]);
        }
        const vec<3> xyz() const
        {
            static_assert(dim >= 3, "Not enough components for XYZ swizzle");
            return vec<3>(e[0], e[1], e[2]);
        }
    };
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
            return vec<3>(v.dot(X),
                          v.dot(Y),
                          v.dot(Z));
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

    // Linear algebra operators
    const vec<3> operator+(vec<3> lhs, vec<3> rhs);
    const vec<3> operator+=(vec<3>& lhs, vec<3> rhs);
    const vec<3> operator-(vec<3> lhs, vec<3> rhs);
    const vec<3> operator-=(vec<3>& lhs, vec<3> rhs);
    const vec<3> operator*(vec<3> lhs, vec<3> rhs);
    const vec<3> operator*=(vec<3>& lhs, vec<3> rhs);
    const vec<3> operator*(vec<3> lhs, float rhs);
    const vec<3> operator*=(vec<3>& lhs, float rhs);
    const vec<3> operator/(vec<3> lhs, vec<3> rhs);
    const vec<3> operator/=(vec<3>& lhs, vec<3> rhs);
    const vec<3> operator/(vec<3> lhs, float rhs);
    const vec<3> operator/=(vec<3>& lhs, float rhs);

    // General functions
    float polyMulBinomial(float x, float a, float b);
    float quadratic(float x, float a, float b, float c, bool inv = true); // All quadratics are assumed to have the form (a - b)^2; additions can be faked by passing negative values for [b]
    float gaussian(float x, float a, float b, float c, float d);
    m3 normalSpace(vec<3> n); // Generates the matrix mapping the space at a point (where Y is aligned with the normal [n]) into world-space
    math::vec<3> floor(math::vec<3> v);
    math::vec<3> ceil(math::vec<3> v);
    math::vec<3> max(math::vec<3> u, math::vec<3> v);
    math::vec<3> min(math::vec<3> u, math::vec<3> v);
    math::vec<3> abs(math::vec<3> v);
    float sgn(float f);

    // Useful constants
    static constexpr float pi = 3.14159265358979323846f;
    static constexpr float inv_pi = 1.0f / pi;
    static constexpr float half_pi = 0.5f * pi;
    static constexpr float quarter_pi = 0.25f * pi;
    static constexpr float pi_2 = 2.0f * pi;
    static constexpr float eps = 0.000001f;

    // Type constraint for rt-soft-render maths functions
    template<typename ret>
    concept maths_type = std::is_floating_point_v<ret> ||
        std::is_same_v<ret, vec<2>> ||
        std::is_same_v<ret, vec<3>> ||
        std::is_same_v<ret, vec<4>>;

    // Higher-level function object/functor; not super fancy but better than std::function (spooky hidden heap allocations) and more readable than c-style function pointers
    template<int dim, maths_type ret>
    struct fn
    {
    private:
        ret(*ptr)(vec<dim>);
    public:
        fn() : ptr(NULL) {};
        fn(decltype(ptr) fnPtr) : ptr(fnPtr) {}
        ret invoke(vec<dim> param) // User-friendly calling interface
        {
            return ptr(param);
        }
    };

    // Extension of [fn] with captured state
    template<int dim, maths_type ret>
    struct fn_closure
    {
    private:
        fn<dim, ret> operation;
    public:
        vec<dim> param_baked;
        fn_closure(vec<dim> _param_baked) : param_baked(_param_baked) {}
        ret invoke()
        {
            return operation(param_baked);
        }
    };
}