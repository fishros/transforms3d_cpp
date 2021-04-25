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
    return mat.eulerAngles(2, 1, 0);
}

Vector3d TransForms::Mat2EulerAngle(Matrix3d mat)
{
    Vector3d rot = mat.eulerAngles(2, 1, 0);
    rot = rot / M_PI * 180;
    return rot;
}

Matrix3d TransForms::EulerAngle2Mat(Vector3d eular)
{
    return EulerAngle2Mat(eular.x(), eular.y(), eular.z());
}

/*通过位置和欧拉角合成一个齐次矩阵*/
Matrix4d TransForms::Compose(Vector3d positon, Vector3d rotEular)
{
    Matrix3d rot = TransForms::EulerAngle2Mat(rotEular);
    // std::cout<<Mat2EulerAngle(rot);
    Matrix4d t;
    t.setIdentity();
    t.block<3, 3>(0, 0) = rot;
    t.block<3, 1>(0, 3) = positon;
    return t;
}

/*通过位置和欧拉角合成一个齐次矩阵*/
Matrix4d TransForms::ComposeEuler(const double x, const double y, const double z, const double rx, const double ry, const double rz)
{
    Eigen::Vector3d rot(rx, ry, rz);
    Eigen::Vector3d pos(x, y, z);
    return TransForms::Compose(pos, rot);
}

/*通过位置和四元数合成一个齐次矩阵*/
Matrix4d TransForms::Compose(Vector3d positon, Quaterniond quat)
{
    // Matrix3d rot = TransForms::Quat2Mat(quat);
    Matrix4d t;
    t.setIdentity();
    // t.block<3,3>(0,0) = rot;
    t.block<3, 1>(0, 3) = positon;
    return t;
}

VectorXd TransForms::H2EulerAngle(Matrix4d t)
{
    VectorXd pose = VectorXd(6);
    Matrix3d mt  = t.block<3,3>(0,0);
    Vector3d p3 = t.block<3,1>(0,3).col(0);
    pose(0,0) = p3.x();
    pose(1,0) = p3.y();
    pose(2,0) = p3.z();
    Vector3d eular = Mat2EulerAngle(mt);
    pose(3,0) = eular.x();
    pose(4,0) = eular.y();
    pose(5,0) = eular.z();
    return pose;
}
