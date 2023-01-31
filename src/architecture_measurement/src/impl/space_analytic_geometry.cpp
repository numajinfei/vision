#include "architecture_measurement/impl/space_analytic_geometry.hpp"

#include <assert.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace am
{

const double INFINITESMAL = 1e-4;

// Eigen::Matrix3d GetMatrixFromRPY(const double& roll, const double& pitch, const double& yaw)
// {

// 	//std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
// 	double realPitch = std::asin(std::sin(pitch)/std::cos(roll));
// 	Eigen::Matrix3d matrix;
// 	matrix =  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(realPitch, Eigen::Vector3d::UnitY());
// 	return matrix;
// }


// Eigen::Matrix3d GetMatrixFromRPY(const double& roll, const double& pitch, const double& yaw)
// {
//     double inclinometer_x_value =  - pitch;
//     double inclinometer_y_value =   roll;
//     Eigen::Vector3d inclinometer_x_axis{std::cos(inclinometer_x_value), 0, std::sin(inclinometer_x_value)};
//     Eigen::Vector3d inclinometer_y_axis{-std::sin(inclinometer_x_value), inclinometer_y_value / abs(inclinometer_y_value) * sqrt((std::cos(inclinometer_x_value) / std::sin(inclinometer_y_value)) * (std::cos(inclinometer_x_value) / std::sin(inclinometer_y_value)) - 1), std::cos(inclinometer_x_value)};
//     inclinometer_y_axis = inclinometer_y_axis * inclinometer_y_value / abs(inclinometer_y_value);
//     inclinometer_y_axis = inclinometer_y_axis / inclinometer_y_axis.norm(); 
//     // 倾角仪xoy平面法向量
//     Eigen::Vector3d inclinometer_xoy_normal;
//     inclinometer_xoy_normal = inclinometer_x_axis.cross(inclinometer_y_axis);
//     std::cout << "inclinometer_x_axis: " << inclinometer_x_axis.transpose() << std::endl;
//     std::cout << "inclinometer_y_axis: " << inclinometer_y_axis.transpose() << std::endl;

//     Eigen::Matrix3d transform_1;
//     transform_1.col(0) = inclinometer_x_axis;
//     transform_1.col(1) = inclinometer_y_axis;
//     transform_1.col(2) = inclinometer_xoy_normal;

//     Eigen::Vector3d unitZ = Eigen::Vector3d::UnitZ();
//     // 两平面法向量夹角
//     double angle;
//     angle = std::acos(sqrt(std::cos(inclinometer_x_value) * std::cos(inclinometer_x_value) - std::sin(inclinometer_y_value) * std::sin(inclinometer_y_value)));
//     // 两平面交线
//     Eigen::Vector3d intersection_line_vector;
//     intersection_line_vector = inclinometer_xoy_normal.cross(unitZ);
//     intersection_line_vector /= intersection_line_vector.norm();
    
//     Eigen::AngleAxisd transform_2_angle_axis{angle, intersection_line_vector} ;
//     Eigen::Matrix3d transform_2;
//     transform_2 = transform_2_angle_axis;

//     Eigen::Matrix3d transform = transform_1;

//     // std::cout << "transform_1: \n" << transform_1 << std::endl;
//     // std::cout << "transform_2: \n" << transform_2 << std::endl;
//     std::cout << "transform: \n" << transform << std::endl;

//     return transform;
// }

Eigen::Matrix3d GetMatrixFromRPY(const double& roll, const double& pitch, const double& yaw)
{

    Eigen::Vector3d inclinometer_x_axis{std::cos(pitch), 0, -std::sin(pitch)};
    Eigen::Vector3d inclinometer_y_axis{std::sin(roll) * std::tan(pitch), std::sqrt( 1 - std::pow( std::sin(roll),2) * ( 1 + std::pow(std::tan(pitch), 2) ) ), std::sin(roll)};
    Eigen::Vector3d inclinometer_z_axis = inclinometer_x_axis.cross(inclinometer_y_axis);

    Eigen::Matrix3d transform;
    transform.col(0) = inclinometer_x_axis;
    transform.col(1) = inclinometer_y_axis;
    transform.col(2) = inclinometer_z_axis;

    return transform;
}


double Preprocess(double& value)
{
    double tmp = value;
    if(tmp < 0)
        tmp *= 0.01;
    return tmp;
}


Eigen::Vector3d TransformPoint(const Eigen::Vector3d point)
{
    Eigen::Matrix<double, 5, 3> matrix1; 
    matrix1<< -2.86084723, 0.83590466, -1.73929751,
            -2.03712225, 0.15644135, 2.04888105,
            -1.44779420, -6.82058811, -0.00549243,
            3.65117383, -0.42722210, 1.63368726,
            3.69441867, -6.20532179, 1.66805899;

    Eigen::Matrix<double, 3, 5> matrix2;
    matrix2 << -32.85906219, -30.25151062, 1.42601395, 36.32200241, 44.40985870,
            -0.01400453, -0.00261978, -22.54638863, 0.22191396, -1.05920744,
            -0.34965616, 0.51574320, -0.09356522, 0.95735604, 0.48018152;

    Eigen::VectorXd tmpVec1;
    tmpVec1 = matrix1 * point;
    for(std::size_t i = 0; i < tmpVec1.rows(); i++)
    {
        tmpVec1(i) = Preprocess(tmpVec1(i));
    }

    Eigen::Vector3d tmpVec2;
    tmpVec2 = matrix2 * tmpVec1;
    for(std::size_t i = 0; i < tmpVec2.rows(); i++)
    {
        tmpVec2(i) = Preprocess(tmpVec2(i));
    }

    return tmpVec2;
}

// Eigen::Matrix3d GetMatrixFromRPY(const double& roll, const double& pitch, const double& yaw)
// {
//     Eigen::Matrix3d mat;
//     // mat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
//     mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
//     return mat;
// }

double Radian(double& degree)
{
    double radian = degree / 180. * M_PI;
    return radian;
}

double Degree(double& radian)
{
    double degree = radian / M_PI * 180;
    return degree;
}

/**** DoublePoints ****/
DoublePoints::DoublePoints()
{

}

DoublePoints::DoublePoints(const Point3d& _startPoint, const Point3d& _endPoint) : startPoint(_startPoint), endPoint(_endPoint)
{

}

DoublePoints::~DoublePoints()
{

}

DoublePoints& DoublePoints::operator=(const DoublePoints& dp)
{
    if(this == &dp)
        return *this;

    startPoint = dp.startPoint;
    endPoint = dp.endPoint;   

    return *this; 
}

void DoublePoints::SetProperty(const Point3d& _startPoint, const Point3d& _endPoint)
{
    startPoint = _startPoint;
    endPoint = _endPoint;
}

void DoublePoints::GetVector(Vector3d& vector)
{
    Vector3d vector_temp{endPoint.point - startPoint.point};
    vector = vector_temp;
}

void DoublePoints::GetDistance(double& distance)
{
    Vector3d startToEndVector;
    GetVector(startToEndVector);
    distance = startToEndVector.vector.norm();
}

void DoublePoints::GetLine(Line3d& line)
{
    Point3d point{startPoint};
    Vector3d vector;
    GetVector(vector);
    Line3d line_temp{point,vector};
    line = line_temp;
}

/**** DoubleLines ****/
DoubleLines::DoubleLines()
{

}

DoubleLines::DoubleLines(const Line3d& _line1, const Line3d& _line2) : line1(_line1), line2(_line2)
{

}

DoubleLines::~DoubleLines()
{

}

void DoubleLines::SetProperty(const Line3d& _line1, const Line3d& _line2)
{
    line1 = _line1;
    line2 = _line2;
}

DoubleLines& DoubleLines::operator=(const DoubleLines& dl)
{
    if(this == &dl)
        return *this;

    line1 = dl.line1;
    line2 = dl.line2;

    return *this;
}

bool DoubleLines::IsCoincident()
{   
    Eigen::Vector3d vector1 = line1.vector.vector;
    Eigen::Vector3d vector2 = line2.vector.vector;

    double dev = (vector1.cross(vector2)).norm();
    if(dev < INFINITESMAL)
    {
        double distance;
        DoublePoints dp(line1.point,line2.point);
        dp.GetDistance(distance);
        if(distance < INFINITESMAL)
            return true;
        else
            return false;
    }        
    else
        return false;
}

bool DoubleLines::IsParallel()
{    
    Eigen::Vector3d vec1 = line1.vector.vector;
    Eigen::Vector3d vec2 = line2.vector.vector;
    double dev = (vec1.cross(vec2)).norm();
    //std::cout << "dev: " << dev << std::endl;
    if(dev < INFINITESMAL && !IsCoincident())
        return true;
    else
        return false;
}

bool DoubleLines::IsVertical()
{
    auto vector1 = line1.vector.vector;
    auto vector2 = line2.vector.vector;
    auto innerProduct = std::fabs(vector1.dot(vector2));
    if(innerProduct < INFINITESMAL)
        return true;
    else    
        return false;
}

bool DoubleLines::IsCoplanar()
{
    if(IsParallel() || IsIntersectant())
        return true;
    else
        return false;
}

bool DoubleLines::IsIntersectant()
{
    if(IsParallel())
        return false;

    const auto& vector1 = line1.vector.vector;
    const auto& vector2 = line2.vector.vector;
    const auto& vector3 = line1.point.point - line2.point.point;
    Eigen::Matrix3d paramMatrix;
    paramMatrix << vector1, vector2, vector3;
    if(std::fabs(paramMatrix.determinant()) < INFINITESMAL)
        return true;
    else
        return false;
}

void DoubleLines::GetDistance(double& distance)
{
    if(IsParallel())
    {
        auto pointToPointVector = line1.point.point - line2.point.point;
        auto crossVector = pointToPointVector.cross(line1.vector.vector);
        distance = crossVector.norm();
    }
    else if(!IsCoplanar())
    {
        auto crossVector = line1.vector.vector.cross(line2.vector.vector);
        auto pointToPointVecxtor = line1.point.point - line2.point.point;
        distance = std::fabs(crossVector.dot(pointToPointVecxtor))/crossVector.norm();
    }
    else
        distance = 0.;
}

void DoubleLines::GetIntersectionPoint(Point3d& point)
{
    assert(IsIntersectant());

    const auto& vector1 = line1.vector.vector;
    const auto& vector2 = line2.vector.vector;
    const auto& pointToPointVector = line2.point.point - line1.point.point;
    Eigen::Matrix<double,3,2> A;
    A << vector1,vector2;
    Eigen::Matrix<double,3,1> b;
    b << pointToPointVector;
    //std::cout << "Here is the matrix A:\n" << A << std::endl;
    //std::cout << "Here is the vector b:\n" << b << std::endl;
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    //std::cout << "The solution is:\n" << x << std::endl;

    const auto& point1 = line1.point.point;
    const auto& point2 = line2.point.point;
    auto intersectionPoint = point1 + x(0) * vector1;
    auto intersectionPoint_test = point2 + x(1) * vector2;
    //std::cout << "intersectionPoint: " << intersectionPoint.transpose() << std::endl;
    //std::cout << "intersectionPoint_test: " << intersectionPoint_test.transpose() << std::endl;
	point = Point3d(intersectionPoint);
}

void DoubleLines::GetCommonNormalLine(Line3d& line)
{
    Eigen::Vector3d op1 = line1.point.point;
    Eigen::Vector3d op2 = line2.point.point;
    Eigen::Vector3d b = op2 - op1;

    auto vector1 = line1.vector.vector;
    auto vector2 = line2.vector.vector;
    Eigen::Vector3d crossVector = vector1.cross(vector2);

    Eigen::Matrix3d A;
    A << vector1, -vector2, crossVector;
    //std::cout << "Here is the matrix A:\n" << A << std::endl;
    //std::cout << "Here is the vector b:\n" << b << std::endl;
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    //std::cout << "The solution is:\n" << x << std::endl;

    Eigen::Vector3d normalLinePoint = op1 + x(0)*vector1;
    Point3d point{normalLinePoint};
    Vector3d vector{crossVector};
    Line3d line_temp{point,vector};
    line = std::move(line_temp);
}

/**** DoublePlanes ****/

DoublePlanes::DoublePlanes()
{

}

DoublePlanes::DoublePlanes(const Plane3d& _plane1, const Plane3d& _plane2) : plane1(_plane1), plane2(_plane2)
{

}

DoublePlanes::~DoublePlanes()
{

}

DoublePlanes& DoublePlanes::operator=(const DoublePlanes& dp)
{
    if(this == &dp)
        return * this;

    plane1 = dp.plane1;
    plane2 = dp.plane2;

    return *this;
}

void DoublePlanes::SetProperty(const Plane3d& _plane1, const Plane3d& _plane2)
{
    plane1 = _plane1;
    plane2 = _plane2;
}

bool DoublePlanes::IsParallel()
{
    const Eigen::Vector3d normalVector1{plane1.a,plane1.b,plane1.c};
    const Eigen::Vector3d normalVector2{plane2.a,plane2.b,plane2.c};

    auto crossProductNorm = normalVector1.cross(normalVector2);
    if(crossProductNorm.norm() < INFINITESMAL)
        return true;
    else
        return false;
}

bool DoublePlanes::IsVertical()
{
    const Eigen::Vector3d normalVector1{plane1.a,plane1.b,plane1.c};
    const Eigen::Vector3d normalVector2{plane2.a,plane2.b,plane2.c};

    double dotProductValue = std::fabs(normalVector1.dot(normalVector2));
    if(dotProductValue < INFINITESMAL)
        return true;
    else
        return false;
}

void DoublePlanes::GetDistance(double& distance)
{
    assert(IsParallel());

    const Eigen::Vector3d normalVector{plane1.a,plane1.b,plane1.c};
    double mol = fabs(plane1.d - plane2.d);
    double den = normalVector.norm();
    distance = mol / den;
}

void DoublePlanes::GetIntersectionLine(Line3d& line)
{
    assert(!IsParallel());

    const Eigen::Vector3d normalVector1{plane1.a,plane1.b,plane1.c};
    const Eigen::Vector3d normalVector2{plane2.a,plane2.b,plane2.c};

    Eigen::Matrix2d A;
    A << normalVector1.squaredNorm(), normalVector1.dot(normalVector2), normalVector1.dot(normalVector2), normalVector2.squaredNorm();
    Eigen::Vector2d b;
    b << -plane1.d, -plane2.d;
    std::cout << "Here is the matrix A:\n" << A << std::endl;
    std::cout << "Here is the vector b:\n" << b << std::endl;
    Eigen::Vector2d x = A.inverse()*b;
    std::cout << "The solution is:\n" << x << std::endl;

    Point3d point{x(0) * normalVector1 + x(1) * normalVector2};
    Vector3d vector{normalVector1.cross(normalVector2)};
    Line3d line_temp{point, vector};
    line = line_temp;
}

/**** PointAndLine ****/
PointAndLine::PointAndLine()
{

}

PointAndLine::PointAndLine(const Point3d& _point, const Line3d& _line) : point(_point), line(_line)
{

}

PointAndLine::~PointAndLine()
{

}

PointAndLine& PointAndLine::operator=(const PointAndLine& pal)
{
    if(this == &pal)
        return *this;

    point = pal.point;
    line = pal.line;

    return *this;
}

void PointAndLine::SetProperty(const Point3d& _point, const Line3d& _line)
{
    point = _point;
    line = _line;
}

bool PointAndLine::IsIncluded()
{
    Eigen::Vector3d pointToPointVector = point.point - line.point.point;
    Eigen::Vector3d crossVector = pointToPointVector.cross(line.vector.vector);
    if(crossVector.norm() < INFINITESMAL)
        return true;
    else
        return false;

}

void PointAndLine::GetProjection(Point3d& _point)
{
    Eigen::Vector3d pointToPointVector = point.point - line.point.point;
    Eigen::Vector3d vector = line.vector.vector;

    double t = pointToPointVector.dot(vector) / vector.squaredNorm();

    double xd = vector(0) * t + line.point.x;
    double yd = vector(1) * t + line.point.y;
    double zd = vector(2) * t + line.point.z;

    Eigen::Vector3d pointVector;
    pointVector << xd, yd, zd;
    Point3d PointProjection{pointVector};
    _point = PointProjection;
}

void PointAndLine::GetDistance(double& distance)
{
    Point3d pointProjection;
    GetProjection(pointProjection);
    DoublePoints dp{pointProjection,point};
    dp.GetDistance(distance);
}

/**** pointAndPlane ****/
PointAndPlane::PointAndPlane()
{

}

PointAndPlane::PointAndPlane(const Point3d& _point, const Plane3d& _plane) : point(_point), plane(_plane)
{

}

PointAndPlane::~PointAndPlane()
{

}

bool PointAndPlane::IsIncluded()
{
    Eigen::Vector4d pointHomogeneous;
    pointHomogeneous << point.x, point.y, point.z, 1.;
    double dotProductValue = 0.;
    dotProductValue = std::fabs(pointHomogeneous.dot(plane.plane));
    if(dotProductValue < INFINITESMAL)
        return true;
    else
        return false;
}

PointAndPlane& PointAndPlane::operator=(const PointAndPlane& pap)
{
    if(this == &pap)
        return *this;

    point = pap.point;
    plane = pap.plane;

    return *this;
}

void PointAndPlane::SetProperty(const Point3d& _point, const Plane3d& _plane)
{
    point = _point;
    plane = _plane;
}

void PointAndPlane::GetProjection(Point3d& _point)
{
    Eigen::Vector3d planeNormVector;
    planeNormVector << plane.a, plane.b, plane.c;
    double mol = - (planeNormVector.dot(point.point) + plane.d);
    double den = planeNormVector.squaredNorm();
    double t = mol/den;

    Eigen::Vector3d pointVector = point.point + t * planeNormVector;
    Point3d point_temp{pointVector};
    _point = point_temp;
}

void PointAndPlane::GetDistance(double& distance)
{
    Eigen::Vector4d pointHomogeneous;
    pointHomogeneous << point.x, point.y, point.z, 1;
    double mol = std::fabs(pointHomogeneous.dot(plane.plane));

    Eigen::Vector3d normalVector;
    normalVector << plane.a, plane.b, plane.c;
    double den = normalVector.norm();

    distance = mol/den;
}

/**** LineAndPlane ****/
LineAndPlane::LineAndPlane()
{

}

LineAndPlane::LineAndPlane(const Line3d& _line, const Plane3d& _plane) : line(_line), plane(_plane)
{

}

LineAndPlane::~LineAndPlane()
{

}

LineAndPlane& LineAndPlane::operator=(const LineAndPlane& lap)
{
    if(this == & lap)
        return *this;

    line = lap.line;
    plane = lap.plane;

    return *this;
}

void LineAndPlane::SetProperty(const Line3d& _line, const Plane3d& _plane)
{
    line = _line;
    plane = _plane;
}

bool LineAndPlane::IsParallel()
{
    const Eigen::Vector3d& lineVector = line.vector.vector;
    Eigen::Vector3d planeNormalVector;
    planeNormalVector << plane.a, plane.b, plane.c;

    double dotProjectValue = std::fabs(lineVector.dot(planeNormalVector));
    if(dotProjectValue < INFINITESMAL)
        return true;
    else
        return false;
}

bool LineAndPlane::IsVertical()
{
    const Eigen::Vector3d& lineVector = line.vector.vector;
    Eigen::Vector3d planeNormalVector;
    planeNormalVector << plane.a, plane.b, plane.c;

    double crossProjectValue = lineVector.cross(planeNormalVector).norm();
    if(crossProjectValue < INFINITESMAL)
        return true;
    else
        return false;
}

bool LineAndPlane::IsIncluded()
{
    Point3d point1{line.point.point};
    Eigen::Vector3d pointVector;
    pointVector << line.point.x + line.vector.x, line.point.y + line.vector.y, line.point.z + line.vector.z;
    Point3d point2{pointVector};

    PointAndPlane pap1{point1, plane};
    PointAndPlane pap2{point2, plane};

    if( pap1.IsIncluded() && pap2.IsIncluded() )
        return true;
    else
        return false;
}

void LineAndPlane::GetAngle(double& angle)
{
    Eigen::Vector3d planeNormalVector;
    planeNormalVector << plane.a, plane.b, plane.c;

    double cosT = fabs(planeNormalVector.dot(line.vector.vector)/planeNormalVector.norm()/line.vector.vector.norm());
    angle = M_PI/(double)2 - std::acos(cosT);
}

void LineAndPlane::GetProjection(Line3d& _line)
{
    Eigen::Vector3d pointVector1 = line.point.point;
    Eigen::Vector3d pointVector2 = pointVector1 + line.vector.vector;
    Point3d point1{pointVector1};
    Point3d point2{pointVector2};
    PointAndPlane pap1(point1,plane);
    PointAndPlane pap2(point2,plane);
    Point3d pointProject1;
    Point3d pointProject2;
    pap1.GetProjection(pointProject1);
    pap2.GetProjection(pointProject2);
    //std::cout << "point1: " << point1 << std::endl;
    //std::cout << "point2: " << point2 << std::endl;
    //std::cout << "pointProject1: " << pointProject1 << std::endl;
    //std::cout << "pointProject2: " << pointProject2 << std::endl;
    Eigen::Vector3d pointToPointVector = pointProject2.point - pointProject1.point;
    Vector3d vector_temp{pointToPointVector};
    Line3d line_temp(pointProject1,vector_temp);
    _line = line_temp;
}

void LineAndPlane::GetIntersectionPoint(Point3d& point)
{
    assert(!IsParallel());

    Eigen::Vector3d linePoint = line.point.point;
    Eigen::Vector3d lineVector = line.vector.vector;

    Eigen::Vector3d planeNormVector;
    planeNormVector << plane.a, plane.b, plane.c;

    Eigen::Vector3d planePoint;
    if( fabs(plane.a) > INFINITESMAL )
    {
        double x = -plane.d / plane.a;
        planePoint << x,0,0;
    }
    else if(fabs(plane.b) > INFINITESMAL)
    {
        double y = -plane.d / plane.b;
        planePoint << 0,y,0;
    }
    else
    {
        double z = -plane.d / plane.c;
        planePoint << 0,0,z;
    }

    Eigen::Vector3d pointToPointVector = planePoint - linePoint;
    double mol = planeNormVector.dot(pointToPointVector);
    double den = planeNormVector.dot(lineVector);

    double t = mol/den;

    Eigen::Vector3d pointVector = linePoint + t * lineVector;
    Point3d point_temp{pointVector};
    point = point_temp;
}

}//namespace am