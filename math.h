#pragma once

#ifdef _DEBUG
#include <assert.h>
#endif

#define _USE_MATH_DEFINES
#include <cmath>

namespace math
{
   // Linear algebra classes
   struct v3
   {
      v3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
      float x;
      float y;
      float z;
      float w = 0.0f; // Ghost type, for nicer padding; ignored by functions & operators, possibly useful for stashing data between shading stages
      float dot(v3 u)
      {
         return x * u.x +
                y * u.y +
                z * u.z;
      }
      v3 cross(v3 u)
      {
         v3 leftMuls(x * u.y,
                     y * u.z,
                     z * u.x);
         v3 rightMuls(y * u.x,
                      z * u.y,
                      x * u.z);
         return math::v3(leftMuls.x - rightMuls.x,
                         leftMuls.y - rightMuls.y,
                         leftMuls.z - rightMuls.z);
      }
      float magnitude()
      {
         return sqrt(x * x + y * y + z * z);
      }
      v3 normalized()
      {
         float len = magnitude();
         return v3(x / len,
                   y / len,
                   z / len);
      }
   };
   struct m3
   {
      m3(v3 _X, v3 _Y, v3 _Z) : X(_X), Y(_Y), Z(_Z) {}
      m3(float x0, float y0, float z0,
         float x1, float y1, float z1,
         float x2, float y2, float z2) :
         X(v3(x0, y0, z0)),
         Y(v3(x1, y1, z1)),
         Z(v3(x2, y2, z2)) {}
      v3 X, Y, Z;
      m3 transposed()
      {
         v3 _X(X.x, Y.x, Z.x);
         v3 _Y(X.y, Y.y, Z.y);
         v3 _Z(X.z, Y.z, Z.z);
         return m3(_X, _Y, _Z);
      }
      v3 apply(v3 v) // matrix x vector product
      {
         return v3(v.dot(X),
                   v.dot(Y),
                   v.dot(Z));
      }
      m3 chain_with(m3 m) // matrix x matrix proeduct for chained transformations
      {
         m3 rhsT = m.transposed();
         return m3(v3(X.dot(rhsT.X),
                      X.dot(rhsT.Y),
                      X.dot(rhsT.Z)),
                   v3(Y.dot(rhsT.X),
                      Y.dot(rhsT.Y),
                      Y.dot(rhsT.Z)),
                   v3(Z.dot(rhsT.X),
                      Z.dot(rhsT.Y),
                      Z.dot(rhsT.Z)));
      }
   };

   // Linear algebra operators
   const v3 operator+(v3 lhs, v3 rhs);
   const v3 operator+=(v3 lhs, v3 rhs);
   const v3 operator-(v3 lhs, v3 rhs);
   const v3 operator-=(v3 lhs, v3 rhs);
   const v3 operator*(v3 lhs, v3 rhs);
   const v3 operator*=(v3 lhs, v3 rhs);
   const v3 operator*(v3 lhs, float rhs);

   // General functions
   float polyMulBinomial(float x, float a, float b);
   float quadratic(float x, float a, float b, float c, bool inv = true); // All quadratics are assumed to have the form (a - b)^2; additions can be faked by passing negative values for [b]
   float gaussian(float x, float a, float b, float c, float d);

   // Useful constants
   static constexpr float pi = 3.14159265358979323846f;
   static constexpr float inv_pi = 1.0f / pi;
   static constexpr float half_pi = 0.5f * pi;
   static constexpr float quarter_pi = 0.25f * pi;
   static constexpr float pi_2 = 2.0f * pi;
   static constexpr float eps = 0.000001f;
}