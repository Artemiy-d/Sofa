#ifndef VECTOR2D_H
#define VECTOR2D_H

class Vector2D
{
public:
    Vector2D(double x = 0, double y = 0) : _x(x), _y(y)
    {

    }

    Vector2D& operator -= (const Vector2D& other)
    {
        _x -= other._x;
        _y -= other._y;

        return *this;
    }

    Vector2D& operator += (const Vector2D& other)
    {
        _x += other._x;
        _y += other._y;

        return *this;
    }

    Vector2D& operator *= (const double c)
    {
        _x *= c;
        _y *= c;

        return *this;
    }

    Vector2D operator - (const Vector2D& other) const
    {
        return {_x - other._x, _y - other._y};
    }

    Vector2D operator + (const Vector2D& other) const
    {
        return {_x + other._x, _y + other._y};
    }

    friend Vector2D operator * (double c, const Vector2D& v)
    {
        return v * c;
    }

    Vector2D operator * (double c) const
    {
        return {_x * c, _y * c};
    }

    double len() const
    {
        return sqrt(_x * _x + _y * _y);
    }

    Vector2D normalized() const
    {
        auto c = 1. / len();
        return {_x * c, _y * c};
    }

    static double dotProduct(const Vector2D& a, const Vector2D& b)
    {
        return a._x * b._x + a._y * b._y;
    }



    double x() const
    {
        return _x;
    }

    double y() const
    {
        return _y;
    }

    void setX(double x)
    {
        _x = x;
    }

    void setY(double y)
    {
        _y = y;
    }

private:
    double _x, _y;
};

#endif // VECTOR2D_H
