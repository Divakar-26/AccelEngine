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

        Vector2 normalized() const
        {
            real l = magnitude();
            if (l > 0)
            {
                return Vector2(x / l, y / l);
            }
            return Vector2(0, 0);
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

        Vector2 operator/(const real value) const
        {
            return Vector2(x / value, y / value);
        }

        Vector2 &operator/=(const real value)
        {
            x /= value;
            y /= value;
            return *this;
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

    class Matrix2
    {
    public:
        /**
         * Matrix elements in array form.
         * 0 1
         * 2 3
         */
        real data[4];

        Matrix2()
        {
            data[0] = data[3] = (real)1;
            data[1] = data[2] = (real)0;
        }

        Matrix2(real m00, real m01, real m10, real m11)
        {
            data[0] = m00;
            data[1] = m01;
            data[2] = m10;
            data[3] = m11;
        }

        void setIdentity()
        {
            data[0] = data[3] = (real)1;
            data[1] = data[2] = (real)0;
        }

        void set(real m00, real m01, real m10, real m11)
        {
            data[0] = m00;
            data[1] = m01;
            data[2] = m10;
            data[3] = m11;
        }

        void setOrientation(real radians)
        {
            real c = (real)std::cos(radians);
            real s = (real)std::sin(radians);
            data[0] = c;
            data[1] = -s;
            data[2] = s;
            data[3] = c;
        }

        Vector2 operator*(const Vector2 &v) const
        {
            return Vector2(
                data[0] * v.x + data[1] * v.y,
                data[2] * v.x + data[3] * v.y);
        }

        Matrix2 operator*(const Matrix2 &o) const
        {
            return Matrix2(
                data[0] * o.data[0] + data[1] * o.data[2],
                data[0] * o.data[1] + data[1] * o.data[3],
                data[2] * o.data[0] + data[3] * o.data[2],
                data[2] * o.data[1] + data[3] * o.data[3]);
        }

        Matrix2 operator*(real scalar) const
        {
            return Matrix2(
                data[0] * scalar, data[1] * scalar,
                data[2] * scalar, data[3] * scalar);
        }

        Vector2 transformTranspose(const Vector2 &v) const
        {
            return Vector2(
                data[0] * v.x + data[2] * v.y,
                data[1] * v.x + data[3] * v.y);
        }

        void invert()
        {
            real det = data[0] * data[3] - data[1] * data[2];
            if (det == (real)0.0f)
                return;

            real invDet = ((real)1.0f) / det;
            Matrix2 tmp = *this;

            data[0] = tmp.data[3] * invDet;
            data[1] = -tmp.data[1] * invDet;
            data[2] = -tmp.data[2] * invDet;
            data[3] = tmp.data[0] * invDet;
        }

        real getDeterminant() const
        {
            return data[0] * data[3] - data[1] * data[2];
        }

        void setInverse(const Matrix2 &m)
        {
            real det = m.getDeterminant();
            if (det == (real)0.0f)
            {
                setIdentity();
                return;
            }

            real invDet = ((real)1.0f) / det;
            data[0] = m.data[3] * invDet;
            data[1] = -m.data[1] * invDet;
            data[2] = -m.data[2] * invDet;
            data[3] = m.data[0] * invDet;
        }

        void transpose()
        {
            std::swap(data[1], data[2]);
        }

        Matrix2 &operator*=(const Matrix2 &o)
        {
            *this = *this * o;
            return *this;
        }

        Vector2 transform(const Vector2 &v) const
        {
            return Vector2(
                data[0] * v.x + data[1] * v.y,
                data[2] * v.x + data[3] * v.y);
        }
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