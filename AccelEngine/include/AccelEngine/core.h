#pragma once

#include "precision.h"

namespace AccelEngine
{

    class Vector2
    {
    public:
        // Constructors
        Vector2() : x(0), y(0) {}
        Vector2(const real x, const real y) : x(x), y(y) {}

        // Invert vector
        void invert()
        {
            x = -x;
            y = -y;
        }

        // Magnitude and normalization
        real magnitude() const
        {
            return sqrt(x * x + y * y);
        }

        real squareMagnitude() const
        {
            return x * x + y * y;
        }

        void normalize()
        {
            real l = magnitude();
            if (l > 0)
            {
                (*this) *= ((real)1) / l;
            }
        }

        // Scaling
        void operator*=(const real value)
        {
            x *= value;
            y *= value;
        }

        Vector2 operator*(const real value) const
        {
            return Vector2(x * value, y * value);
        }

        // Addition / subtraction
        void operator+=(const Vector2 &v)
        {
            x += v.x;
            y += v.y;
        }

        Vector2 operator+(const Vector2 &v) const
        {
            return Vector2(x + v.x, y + v.y);
        }

        void operator-=(const Vector2 &v)
        {
            x -= v.x;
            y -= v.y;
        }

        Vector2 operator-(const Vector2 &v) const
        {
            return Vector2(x - v.x, y - v.y);
        }

        // Add scaled vector
        void addScaledVector(const Vector2 &v, real scale)
        {
            x += v.x * scale;
            y += v.y * scale;
        }

        // Component-wise operations
        Vector2 componentProduct(const Vector2 &v) const
        {
            return Vector2(x * v.x, y * v.y);
        }

        void componentProductUpdate(const Vector2 &v)
        {
            x *= v.x;
            y *= v.y;
        }

        // Dot product
        real scalarProduct(const Vector2 &v) const
        {
            return x * v.x + y * v.y;
        }

        real operator*(const Vector2 &v) const
        {
            return x * v.x + y * v.y;
        }

        // 2D "cross product" helper (returns scalar)
        real cross(const Vector2 &v) const
        {
            return x * v.y - y * v.x;
        }

        // Perpendicular vector (useful for physics)
        Vector2 perpendicular() const
        {
            return Vector2(-y, x);
        }

        static real distance(const Vector2 &a, const Vector2 &b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return sqrtf(dx * dx + dy * dy);
        }

        // Clear to zero
        void clear()
        {
            x = y = 0;
        }

        // Components
        real x, y;
    };

    class Vector3
    {
    public:
        // constructors
        Vector3() : x(0), y(0), z(0) {}
        Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

        // methods and helping fucntions
        void invert()
        {
            x = -x;
            y = -y;
            z = -z;
        }

        real magnitude()
        {
            return sqrt(x * x + y * y + z * z);
        }

        real squareMagnitude()
        {
            return x * x + y * y + z * z;
        }

        void normalize()
        {
            real l = magnitude();
            if (l > 0)
            {
                (*this) *= ((real)1) / l;
            }
        }

        // operations
        void operator*=(const real value)
        {
            x *= value;
            y *= value;
            z *= value;
        }

        Vector3 operator*(const real value) const
        {
            return Vector3(x * value, y * value, z * value);
        }

        void operator+=(const Vector3 &v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        Vector3 operator+(const Vector3 &v) const
        {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        void operator-=(const Vector3 &v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        Vector3 operator-(const Vector3 &v) const
        {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        void addScaledVector(const Vector3 &v, real scale)
        {
            x += v.x * scale;
            y += v.y * scale;
            z += v.z * scale;
        }

        Vector3 componentProduct(const Vector3 &v) const
        {
            return Vector3(x * v.x, y * v.y, z * v.z);
        }

        void componentProductUpdate(const Vector3 &v)
        {
            x *= v.x;
            y *= v.y;
            z *= v.z;
        }

        real scalarProduct(const Vector3 &v) const
        {
            return x * v.x + y * v.y + z * v.z;
        }

        real operator*(const Vector3 &v) const
        {
            return x * v.x + y * v.y + z * v.z;
        }

        Vector3 vectorProduct(const Vector3 &v) const
        {
            return Vector3(y * v.z - z * v.y,
                           z * v.x - x * v.z,
                           x * v.y - y * v.x);
        }

        void operator%=(const Vector3 &v)
        {
            *this = vectorProduct(v);
        }

        Vector3 operator%(const Vector3 &v) const
        {
            return Vector3(y * v.z - z * v.y,
                           z * v.x - x * v.z,
                           x * v.y - y * v.x);
        }

        void clear()
        {
            x = y = z = 0;
        }
        // actual components
        real x, y, z;

    private:
        real pad;
    };

}