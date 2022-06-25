export module vmath;

import platform;
import <concepts>;

//#define MATH_DBG
#ifdef MATH_DBG
#pragma optimize("", off)
#endif
export namespace vmath
{
    // Useful constants
    constexpr float pi = 3.14159265358979323846f;
    constexpr float inv_pi = 1.0f / pi;
    constexpr float half_pi = 0.5f * pi;
    constexpr float quarter_pi = 0.25f * pi;
    constexpr float pi_2 = 2.0f * pi;
    constexpr float eps = 0.000001f;

    // Linear algebra classes
    template<int dim> requires (dim < 5 && dim > 1)
    struct vec
    {
        vec<dim>()
        {
            for (int i = 0; i < dim; i++)
            {
                e[i] = 0;
            }
        }
        vec<dim>(float _x)
        {
            // Valid for all dimensions - behaves like a broadcast
            for (int i = 0; i < dim; i++)
            {
                e[i] = _x;
            }
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
            for (int i = 0; i < dim; i++)
            {
                ret += u.e[i] * e[i];
            }
            return ret;
        }
        vec<3> cross(vec<3> u)
        {
            static_assert(dim == 3, "The cross product is only meaningful in three dimensions - the wedge product is interesting & more general but i haven't found a use-case for it here yet");
            vec<3> leftMuls(e[1] * u.e[2],
                            e[2] * u.e[0],
                            e[0] * u.e[1]);
            vec<3> rightMuls(e[2] * u.e[1],
                             e[0] * u.e[2],
                             e[1] * u.e[0]);
            return vec<3>(leftMuls.e[0] - rightMuls.e[0],
                          leftMuls.e[1] - rightMuls.e[1],
                          leftMuls.e[2] - rightMuls.e[2]);
        }

        // Need to refresh my memory of how this math works...
        vec<4> qtn_rotation_concat(vec<4> q)
        {
            static_assert(dim == 4, "Quaternionic vectors must have four axes");
            //platform::osAssertion(q.sqr_magnitude() == 1.0f && sqr_magnitude() == 1.0f); // Rotation quaternions should have unit magnitude
            vmath::vec<3> v = xyz();
            vec<3> a = w() * q.xyz();
            vec<3> b = q.w() * v;
            vec<3> ortho = a.cross(b);
            vec<3> u = a + b + ortho;
            return vec<4>(u.x(), u.y(), u.z(),
                          (w() * q.w()) - v.dot(q.xyz()));
        }
        vec<3> qtn_rotation_apply(vec<4> q)
        {
            static_assert(dim == 3, "This function is for applying quaternionic rotations to 3D vectors - 2D vectors can be rotated with regular sin/cos, and quaternion rotations can be concatenated using [qtn_rotation_concat] (see above)");
            //platform::osAssertion(q.sqr_magnitude() == 1.0f); // Rotation quaternions should have unit magnitude
            vec<4> p = vec<4>(x(), y(), z(), 0.0);
            vec<4> qv = p.qtn_rotation_concat(q);
            vec<4> q_inv = vec<4>(qv.x() * -1.0f,
                                  qv.y() * -1.0f,
                                  qv.z() * -1.0f, q.w());
            return qv.qtn_rotation_concat(q_inv).xyz();
        }
        const float sqr_magnitude() const
        {
            float acc = 0;
            for (int i = 0; i < dim; i++)
            {
                acc += e[i] * e[i];
            }
            return acc;
        }
        const float magnitude() const
        {
            float acc = 0;
            for (int i = 0; i < dim; i++)
            {
                acc += e[i] * e[i];
            }
            return static_cast<float>(sqrt(acc));
        }
        vec<dim> normalized() const
        {
            float len = magnitude();
            vec<dim> d = *this;
            for (int i = 0; i < dim; i++) d.e[i] /= len;
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
        const vec<2> xz() const
        {
            static_assert(dim >= 3, "no z-coordinate available");
            return vec<2>(e[0], e[2]);
        }
        const vec<2> zw() const
        {
            static_assert(dim == 4, "no w-coordinate available");
            return vec<2>(e[2], e[3]);
        }
        const vec<2> yw() const
        {
            static_assert(dim == 4, "no w-coordinate available");
            return vec<2>(e[1], e[3]);
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

    // Type constraint for vox_sculpt maths functions
    template<typename numeric_t>
    concept maths_type = std::is_floating_point_v<numeric_t> ||
                         std::is_same_v<numeric_t, vec<2>> ||
                         std::is_same_v<numeric_t, vec<3>> ||
                         std::is_same_v<numeric_t, vec<4>>;

    // Linear algebra operators
    // 3D
    const vec<3> operator+(vec<3> lhs, vec<3> rhs)
    {
        return vec<3>(lhs.e[0] + rhs.e[0],
            lhs.e[1] + rhs.e[1],
            lhs.e[2] + rhs.e[2]);
    }

    const vec<3> operator+=(vec<3>& lhs, vec<3> rhs)
    {
        lhs = vec<3>(lhs.e[0] + rhs.e[0],
            lhs.e[1] + rhs.e[1],
            lhs.e[2] + rhs.e[2]);
        return lhs;
    }

    const vec<3> operator-(vec<3> lhs, vec<3> rhs)
    {
        return vec<3>(lhs.e[0] - rhs.e[0],
            lhs.e[1] - rhs.e[1],
            lhs.e[2] - rhs.e[2]);
    }
    const vec<3> operator-=(vec<3>& lhs, vec<3> rhs)
    {
        lhs = lhs - rhs;
        return lhs;
    }
    const vec<3> operator*(vec<3> lhs, vec<3> rhs)
    {
        return vec<3>(lhs.e[0] * rhs.e[0],
            lhs.e[1] * rhs.e[1],
            lhs.e[2] * rhs.e[2]);
    }
    const vec<3> operator*=(vec<3>& lhs, vec<3> rhs)
    {
        lhs = lhs * rhs;
        return lhs;
    }

    const vec<3> operator*(vec<3> lhs, float rhs)
    {
        return vec<3>(lhs.e[0] * rhs,
            lhs.e[1] * rhs,
            lhs.e[2] * rhs);
    }

    const vec<3> operator*=(vec<3>& lhs, float rhs)
    {
        lhs = lhs * rhs;
        return lhs;
    }

    const vec<3> operator/(vec<3> lhs, vec<3> rhs)
    {
        return vec<3>(lhs.e[0] / rhs.e[0],
                      lhs.e[1] / rhs.e[1],
                      lhs.e[2] / rhs.e[2]);
    }

    const vec<3> operator/=(vec<3>& lhs, vec<3> rhs)
    {
        lhs = lhs / rhs;
        return lhs;
    }

    const vec<3> operator/(vec<3> lhs, float rhs)
    {
        return vec<3>(lhs.e[0] / rhs,
                      lhs.e[1] / rhs,
                      lhs.e[2] / rhs);
    }

    const vec<3> operator/=(vec<3>& lhs, float rhs)
    {
        lhs = lhs / rhs;
        return lhs;
    }

    const bool anyGreater(vec<3> lhs, vec<3> rhs)
    {
        return lhs.e[0] > rhs.e[0] ||
            lhs.e[1] > rhs.e[1] ||
            lhs.e[2] > rhs.e[2];
    }

    const bool anyLesser(vec<3> lhs, vec<3> rhs)
    {
        return lhs.e[0] < rhs.e[0] ||
            lhs.e[1] < rhs.e[1] ||
            lhs.e[2] < rhs.e[2];
    }

    const bool anyEqual(vec<3> lhs, vec<3> rhs)
    {
        return lhs.e[0] == rhs.e[0] ||
               lhs.e[1] == rhs.e[1] ||
               lhs.e[2] == rhs.e[2];
    }

    const bool allGreater(vec<3> lhs, vec<3> rhs)
    {
        return lhs.e[0] > rhs.e[0] &&
               lhs.e[1] > rhs.e[1] &&
               lhs.e[2] > rhs.e[2];
    }

    const bool allLesser(vec<3> lhs, vec<3> rhs)
    {
        return lhs.e[0] < rhs.e[0] &&
            lhs.e[1] < rhs.e[1] &&
            lhs.e[2] < rhs.e[2];
    }

    const bool allEqual(vec<3> lhs, vec<3> rhs)
    {
        return lhs.e[0] == rhs.e[0] &&
            lhs.e[1] == rhs.e[1] &&
            lhs.e[2] == rhs.e[2];
    }

    const bool anyGreater(vec<3> lhs, float rhs)
    {
        return lhs.e[0] > rhs ||
            lhs.e[1] > rhs ||
            lhs.e[2] > rhs;
    }
    const bool anyLesser(vec<3> lhs, float rhs)
    {
        return lhs.e[0] < rhs ||
            lhs.e[1] < rhs ||
            lhs.e[2] < rhs;
    }
    const bool anyEqual(vec<3> lhs, float rhs)
    {
        return lhs.e[0] == rhs ||
               lhs.e[1] == rhs ||
               lhs.e[2] == rhs;
    }
    const bool allGreater(vec<3> lhs, float rhs)
    {
        return lhs.e[0] > rhs &&
               lhs.e[1] > rhs &&
               lhs.e[2] > rhs;
    }
    const bool allLesser(vec<3> lhs, float rhs)
    {
        return lhs.e[0] < rhs &&
               lhs.e[1] < rhs &&
               lhs.e[2] < rhs;
    }
    const bool allEqual(vec<3> lhs, float rhs)
    {
        return lhs.e[0] == rhs &&
               lhs.e[1] == rhs &&
               lhs.e[2] == rhs;
    }

    // 2D
    const vec<2> operator+(vec<2> lhs, vec<2> rhs)
    {
        return vec<2>(lhs.e[0] + rhs.e[0],
            lhs.e[1] + rhs.e[1]);
    }

    const vec<2> operator+=(vec<2>& lhs, vec<2> rhs)
    {
        lhs = vec<2>(lhs.e[0] + rhs.e[0],
            lhs.e[1] + rhs.e[1]);
        return lhs;
    }

    const vec<2> operator-(vec<2> lhs, vec<2> rhs)
    {
        return vec<2>(lhs.e[0] - rhs.e[0],
            lhs.e[1] - rhs.e[1]);
    }
    const vec<2> operator-=(vec<2>& lhs, vec<2> rhs)
    {
        lhs = lhs - rhs;
        return lhs;
    }
    const vec<2> operator*(vec<2> lhs, vec<2> rhs)
    {
        return vec<2>(lhs.e[0] * rhs.e[0],
            lhs.e[1] * rhs.e[1]);
    }
    const vec<2> operator*=(vec<2>& lhs, vec<2> rhs)
    {
        lhs = lhs * rhs;
        return lhs;
    }

    const vec<2> operator*(vec<2> lhs, float rhs)
    {
        return vec<2>(lhs.e[0] * rhs,
            lhs.e[1] * rhs);
    }

    const vec<2> operator*=(vec<2>& lhs, float rhs)
    {
        lhs = lhs * rhs;
        return lhs;
    }

    const vec<2> operator/(vec<2> lhs, vec<2> rhs)
    {
        return vec<2>(lhs.e[0] / rhs.e[0],
                      lhs.e[1] / rhs.e[1]);
    }

    const vec<2> operator/=(vec<2>& lhs, vec<2> rhs)
    {
        lhs = lhs / rhs;
        return lhs;
    }

    const vec<2> operator/(vec<2> lhs, float rhs)
    {
        return vec<2>(lhs.e[0] / rhs,
            lhs.e[1] / rhs);
    }

    const vec<2> operator/=(vec<2>& lhs, float rhs)
    {
        lhs = lhs / rhs;
        return lhs;
    }

    const bool anyGreater(vec<2> lhs, vec<2> rhs)
    {
        return lhs.e[0] > rhs.e[0] ||
            lhs.e[1] > rhs.e[1];
    }

    const bool anyLesser(vec<2> lhs, vec<2> rhs)
    {
        return lhs.e[0] < rhs.e[0] ||
            lhs.e[1] < rhs.e[1];
    }

    const bool anyEqual(vec<2> lhs, vec<2> rhs)
    {
        return lhs.e[0] == rhs.e[0] ||
            lhs.e[1] == rhs.e[1];
    }

    const bool allGreater(vec<2> lhs, vec<2> rhs)
    {
        return lhs.e[0] > rhs.e[0] &&
            lhs.e[1] > rhs.e[1];
    }

    const bool allLesser(vec<2> lhs, vec<2> rhs)
    {
        return lhs.e[0] < rhs.e[0] &&
            lhs.e[1] < rhs.e[1];
    }

    const bool allEqual(vec<2> lhs, vec<2> rhs)
    {
        return lhs.e[0] == rhs.e[0] &&
            lhs.e[1] == rhs.e[1];
    }

    const bool anyGreater(vec<2> lhs, float rhs)
    {
        return lhs.e[0] > rhs &&
            lhs.e[1] > rhs;
    }
    const bool anyLesser(vec<2> lhs, float rhs)
    {
        return lhs.e[0] < rhs &&
            lhs.e[1] < rhs;
    }
    const bool anyEqual(vec<2> lhs, float rhs)
    {
        return lhs.e[0] == rhs &&
            lhs.e[1] == rhs;
    }
    const bool allGreater(vec<2> lhs, float rhs)
    {
        return lhs.e[0] > rhs &&
            lhs.e[1] > rhs;
    }
    const bool allLesser(vec<2> lhs, float rhs)
    {
        return lhs.e[0] < rhs &&
            lhs.e[1] < rhs;
    }
    const bool allEqual(vec<2> lhs, float rhs)
    {
        return lhs.e[0] == rhs &&
               lhs.e[1] == rhs;
    }

    float fsin(float a);
    vec<2> sin(vec<2> v)
    {
        return vec<2>(fsin(v.x()),
                      fsin(v.y()));
    }

    vec<3> sin(vec<3> v)
    {
        return vec<3>(fsin(v.x()),
                      fsin(v.y()),
                      fsin(v.z()));
    }

    float fcos(float a);
    vec<2> cos(vec<2> v)
    {
        return vec<2>(fcos(v.x()),
                      fcos(v.y()));
    }

    vec<3> cos(vec<3> v)
    {
        return vec<3>(fcos(v.x()),
                      fcos(v.y()),
                      fcos(v.z()));
    }

    float ftan(float a);
    vec<2> tan(vec<2> v)
    {
        return vec<2>(ftan(v.x()),
                      ftan(v.y()));
    }

    vec<3> tan(vec<3> v)
    {
        return vec<3>(ftan(v.x()),
                      ftan(v.y()),
                      ftan(v.z()));
    }

    // General functions
    float polyMulBinomial(float x, float a, float b)
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

    float quadratic(float x, float a, float b, float c, bool inv = true)
    {
        // Solve inner multiply
        float polyNum = polyMulBinomial(x, a, b);

        // Invert, apply translation, return :)
        return inv ? -polyNum + c :
                      polyNum + c;
    };

    float fexp(float f);
    vmath::vec<2> vexp(vmath::vec<2> v)
    {
        return vmath::vec<2>(vmath::fexp(v.x()),
                             vmath::fexp(v.y()));
    }
    vmath::vec<3> vexp(vmath::vec<3> v)
    {
        return vmath::vec<3>(vmath::fexp(v.x()),
                             vmath::fexp(v.y()),
                             vmath::fexp(v.z()));
    }

    float gaussian(float x, float a, float b, float c, float d)
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

    float fabs(float a);
    m3 normalSpace(vec<3> n)
    {
        // Tangent normal function from:
        // https://www.shadertoy.com/view/ltKyzm
        // Originally from Scratchapixel
        // https://www.scratchapixel.com/lessons/3d-basic-rendering/global-illumination-path-tracing/global-illumination-path-tracing-practical-implementation

        // Generate a basis-x direction
        // Different functions will be more/less appropriate
        // depending on the normal's x/y-tendency
        vec<3> absNormal = vec<3>(fabs(n.x()), fabs(n.y()), fabs(n.z()));
        vec<3> basisX = vec<3>(0,0,0);
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
        return m3(basisX,
                  n,
                  basisX.cross(n));
    }

    float ffloor(float f);
    vec<3> vfloor(vec<3> v)
    {
        return vec<3>(ffloor(v.e[0]),
                      ffloor(v.e[1]),
                      ffloor(v.e[2]));
    }

    float ffrac(float f);
    vec<3> vfrac(vec<3> v)
    {
        return vec<3>(ffrac(v.e[0]),
                      ffrac(v.e[1]),
                      ffrac(v.e[2]));
    }
    vec<2> vfrac(vec<2> v)
    {
        return vec<2>(ffrac(v.e[0]),
                      ffrac(v.e[1]));
    }

    float fceil(float f);
    vec<3> vceil(vec<3> v)
    {
        return vec<3>(fceil(v.e[0]),
                      fceil(v.e[1]),
                      fceil(v.e[2]));
    }

    template<maths_type lerp_type>
    lerp_type lerp(lerp_type a, lerp_type b, lerp_type t) // Because I keep forgetting how these work: https://en.wikipedia.org/wiki/Linear_interpolation
    {
        return a + t * (a - b);
    }

    float fmax(float a, float b)
    {
        return a > b ? a : b;
    }
    float fmin(float a, float b)
    {
        return a > b ? b : a;
    }
    unsigned int umax(unsigned int a, unsigned int b)
    {
        return a > b ? a : b;
    }
    unsigned int umin(unsigned int a, unsigned int b)
    {
        return a > b ? b : a;
    }

    vec<3> vmax(vec<3> v, vec<3> u)
    {
        return vec<3>(fmax(v.e[0], u.e[0]),
                      fmax(v.e[1], u.e[1]),
                      fmax(v.e[2], u.e[2]));
    }

    vec<3> vmin(vec<3> v, vec<3> u)
    {
        return vec<3>(fmin(v.e[0], u.e[0]),
                      fmin(v.e[1], u.e[1]),
                      fmin(v.e[2], u.e[2]));
    }

    template<maths_type clamp_type>
    clamp_type clamp(clamp_type a, clamp_type clampMin, clamp_type clampMax)
    {
        if constexpr (std::is_same<clamp_type, float>::value)
        {
            return fmin(fmax(a, clampMin), clampMax);
        }
        else if constexpr (!std::is_same<clamp_type, float>::value)
        {
            return vmin(vmax(a, clampMin), clampMax);
        }
        platform::osAssertion(false);
        return a; // Failure case; assert false & return the value unclamped
    }

    vec<2> vabs(vec<2> v)
    {
        return vec<2>(fabs(v.e[0]),
                      fabs(v.e[1]));
    }

    vec<3> vabs(vec<3> v)
    {
        return vec<3>(fabs(v.e[0]),
                      fabs(v.e[1]),
                      fabs(v.e[2]));
    }

    float fmodf(float fval, float finterval);
    vec<3> vmodf(vec<3> u, vec<3> v)
    {
        return vec<3>(fmodf(u.e[0], v.e[0]),
                      fmodf(u.e[1], v.e[1]),
                      fmodf(u.e[2], v.e[2]));
    }

    float fsqrt(float fval);
    vec<3> vsqrt(vec<3> u, vec<3> v)
    {
        return vec<3>(fsqrt(u.e[0]),
                      fsqrt(u.e[1]),
                      fsqrt(u.e[2]));
    }

    float fsgn(float f)
    {
        unsigned int u = *reinterpret_cast<unsigned int*>(&f);
        const bool bitset = u & (1 << 31);
        return bitset ? -1.0f : 1.0f;
    }

    vec<3> vsgn(vec<3> v)
    {
        return vec<3>(fsgn(v.x()),
                      fsgn(v.y()),
                      fsgn(v.z()));
    }

    bool eps_equality(float x, float y)
    {
        return x >= (y - eps) &&
               x <= (y + eps);
    }

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

#ifdef MATH_DBG
#pragma optimize("", on)
#endif