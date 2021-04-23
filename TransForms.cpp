#include "math.h"
#include "TransForms.h"

/*角度转为弧度*/
double TransForms::Degrees(double angle)
{
    return angle / 180 * M_PI;
}
/*弧度转为角度*/
double TransForms::Angle(double degrees)
{
    return degrees / M_PI * 180;
}

Matrix3d TransForms::Euler2Mat(double rx, double ry, double rz)
{
    AngleAxisd rollAngle(AngleAxisd(rz, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(ry, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(rx, Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
}

Matrix3d TransForms::EulerAngle2Mat(double rx, double ry, double rz)
{
    rx = Degrees(rx);
    ry = Degrees(ry);
    rz = Degrees(rz);
    AngleAxisd rollAngle(AngleAxisd(rz, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(ry, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(rx, Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
}

Quaterniond TransForms::Euler2Quat(double rx, double ry, double rz)
{
    return Eigen::AngleAxisd(rx, ::Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(ry, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rz, ::Eigen::Vector3d::UnitZ());
}

Quaterniond TransForms::EulerAngle2Quat(double rx, double ry, double rz)
{
    rx = Degrees(rx);
    ry = Degrees(ry);
    rz = Degrees(rz);
    return Eigen::AngleAxisd(rx, ::Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(ry, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rz, ::Eigen::Vector3d::UnitZ());
}

Vector3d TransForms::Mat2Euler(Matrix3d mat)
{
    return mat.eulerAngles(2,1,0);
}


Vector3d TransForms::Mat2EulerAngle(Matrix3d mat)
{
    Vector3d rot = mat.eulerAngles(2,1,0);
    rot = rot/M_PI*180;
    return rot;
}


