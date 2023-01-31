#include "architecture_measurement/impl/types.hpp"

namespace am
{

/**** Point3d ****/
Point3d::Point3d()
{

}

Point3d::Point3d(const double& _x,const double& _y,const double& _z) : x(_x),y(_y),z(_z)
{
    point << x,y,z;
}

Point3d::Point3d(const Eigen::Vector3d& _point) :  point(_point)
{
    x = point(0);
    y = point(1);
    z = point(2);
}

Point3d::Point3d(const Point3d& _point)
{
    x = _point.x;
    y = _point.y;
    z = _point.z;
    point = _point.point;
}

Point3d::~Point3d()
{

}

Point3d& Point3d::operator=(const Point3d& _point)
{
    if(this == &_point)
        return *this;

    x = _point.x;
    y = _point.y;
    z = _point.z;
    point = _point.point;

    return *this;
}

std::ostream& operator<<(std::ostream& os, const Point3d& point)
{
    os << "[" << point.x << ", " << point.y << ", " << point.z << "]";
    return os;
}

/**** Vector3d ****/
Vector3d::Vector3d()
{

}

Vector3d::Vector3d(const double& _x,const double& _y,const double& _z) : x(_x), y(_y), z(_z) 
{
    vector << x, y, z;
}

Vector3d::Vector3d(const Eigen::Vector3d& _vector) : vector(_vector)
{
    x = vector(0);
    y = vector(1);
    z = vector(2);
}

Vector3d::Vector3d(const Vector3d& _vector)
{
    x = _vector.x;
    y = _vector.y;
    z = _vector.z;
    vector = _vector.vector;
}

Vector3d::~Vector3d()
{

}

Vector3d& Vector3d::operator=(const Vector3d& _vector)
{
    if(this == &_vector)
        return *this;

    x = _vector.x;
    y = _vector.y;
    z = _vector.z;
    vector = _vector.vector;

    return *this;
}

std::ostream& operator<<(std::ostream& os, const Vector3d& vector)
{
    os << "[" << vector.x << ", " << vector.y << ", " << vector.z << "]";
    return os;
}

void Vector3d::Normalization(const bool& normlize)
{
    if(normlize)
    {
        vector = vector/vector.norm();
        x = vector(0);
        y = vector(1);
        z = vector(2);
    }
}

/**** Line3d ****/
Line3d::Line3d()
{

}

Line3d::Line3d(const Point3d& _point, const Vector3d& _vector) : point(_point), vector(_vector) 
{
    vector.Normalization(true);
}

Line3d::Line3d(const Line3d& _line)
{
    point = _line.point;
    vector = _line.vector;
}

Line3d::~Line3d()
{

}

Line3d& Line3d::operator=(const Line3d& _line)
{
    if(this == &_line)
        return *this;

    point = _line.point;
    vector = _line.vector;

    return *this;
}

std::ostream& operator<<(std::ostream& os, const Line3d& line)
{
    os << line.point << "  " << line.vector;
    return os;
}

/**** Plane3d ****/
Plane3d::Plane3d()
{

}

Plane3d::Plane3d(const double& _a, const double& _b, const double& _c, const double& _d) : a(_a), b(_b), c(_c), d(_d)
{
    Eigen::Vector3d normVector;
    normVector << a, b, c;
    Eigen::Vector3d normVectorNormalization = normVector/normVector.norm();
    d = d/normVector.norm();
    plane << normVectorNormalization(0), normVectorNormalization(1), normVectorNormalization(2), d;
    a = plane(0);
    b = plane(1);
    c = plane(2);
    d = plane(3);
}

Plane3d::Plane3d(const Plane3d& _plane)
{
    a = _plane.a;
    b = _plane.b;
    c = _plane.c;
    d = _plane.d;
    plane = _plane.plane;
}

Plane3d::~Plane3d()
{

}

Plane3d& Plane3d::operator=(const Plane3d& _plane)
{
    if(this == &_plane)
        return *this;

    a = _plane.a;
    b = _plane.b;
    c = _plane.c;
    d = _plane.d;
    plane = _plane.plane;

    return *this;
}

std::ostream& operator<<(std::ostream& os, const Plane3d& plane)
{
    os << "[" << plane.a << ", " << plane.b << ", " << plane.c << ", " <<plane.d << "]";
    return os;
}

/**** others ****/

Plane3d ConvertToPlane(const Line3d& line)
{
    Plane3d plane;
    plane.a = line.vector.x;
    plane.b = line.vector.y;
    plane.c = line.vector.z;

    Eigen::Vector3d vector;
    vector << plane.a, plane.b, plane.c;
    Eigen::Vector3d point;
    point << line.point.x, line.point.y, line.point.z;
    plane.d = -vector.dot(point);

    return plane;    
}

Line3d ConvertToLine(const Plane3d& plane)
{
    Eigen::Vector3d vector_temp;
    vector_temp << plane.a, plane.b, plane.c;
    Eigen::Vector3d Point_temp;
    Point_temp << 0., 0., 0.;
    Point3d point{Point_temp};
    Vector3d vector{vector_temp};
    Line3d line{point, vector};
    return line;
}


} //namespace am