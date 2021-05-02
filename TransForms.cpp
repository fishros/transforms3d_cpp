#include <TransForms3d/TransForms.h>

/*---------------------------------------角度弧度转换----------------------------------------*/
/**
     * @description: 角度转为弧度
     * @param {double} angle 角度值
     * @return 返回对应弧度值,一般在-3.14~3.14之间
     */
double TransForms::Degrees(double angle)
{
    return angle / 180 * M_PI;
}

/**
     * @description: 弧度转为角度
     * @param {double} degrees 弧度值
     * @return 返回对应的角度值，一般在-180~180之间
     */
double TransForms::Angle(double degrees)
{
    return degrees / M_PI * 180;
}

/*---------------------------------------欧拉角部分---------------------------*/
/**
     * @description: 角度制欧拉角转旋转矩阵,此函数默认的旋转顺序是x-y-z.
     * @param {double} rx 绕x轴的旋转.
     * @param {double} ry 绕y轴的旋转.
     * @param {double} rz 绕z轴的旋转.
     * @return {Matrix3d}  返回3✖3的旋转矩阵.
     */
Matrix3d TransForms::EulerAngle2Mat(double rx, double ry, double rz)
{
    rx = Degrees(rx);
    ry = Degrees(ry);
    rz = Degrees(rz);
    AngleAxisd rollAngle(AngleAxisd(rx, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(ry, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(rz, Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
}
/**
     * @description: 欧拉角转旋转矩阵
     * @param {Vector3d} eular 欧拉角rx,ry,rz 
     * @return {Matrix3d} 返回3✖3的旋转矩阵.
     */
Matrix3d TransForms::Euler2Mat(double rx, double ry, double rz)
{
    AngleAxisd rollAngle(AngleAxisd(rx, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(ry, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(rz, Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
}

/**
     * @description:欧拉角转四元数
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Quaterniond} 返回对应的四元数
     */
Quaterniond TransForms::Euler2Quat(double rx, double ry, double rz)
{
    return Eigen::AngleAxisd(rx, ::Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(ry, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rz, ::Eigen::Vector3d::UnitZ());
}

/**
     * @description: 角度制欧拉角转四元数
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Quaterniond} 返回对应的四元数
     */
Quaterniond TransForms::EulerAngle2Quat(double rx, double ry, double rz)
{
    rx = Degrees(rx);
    ry = Degrees(ry);
    rz = Degrees(rz);
    return Eigen::AngleAxisd(rx, ::Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(ry, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rz, ::Eigen::Vector3d::UnitZ());
}
/**
     * @description: 旋转矩阵转欧拉角(弧度制)
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Vector3d} 欧拉角
     */
Vector3d TransForms::Mat2Euler(Matrix3d mat)
{
    return mat.eulerAngles(2, 1, 0);
}

/**
     * @description: 欧拉角转旋转矩阵
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {*}
     */
Matrix3d TransForms::EulerAngle2Mat(Vector3d eular)
{
    return EulerAngle2Mat(eular.x(), eular.y(), eular.z());
}

/**
     * @description: 旋转矩阵转角度制欧拉角(角度制)
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Vector3d}  欧拉角 
     */
Vector3d TransForms::Mat2EulerAngle(Matrix3d mat)
{
    Vector3d rot = mat.eulerAngles(0, 1, 2);
    rot = rot / M_PI * 180;
    return rot;
}

/*---------------------------------------四元数部分----------------------------------------*/
/**
     * @description: 四元数转旋转矩阵
     * @param {Quaterniond} 四元数
     * @return {Matrix3d} 对应的旋转矩阵
     */
Matrix3d TransForms::Quat2Mat(Quaterniond quat)
{
    return quat.matrix();
}

/**
     * @description: 四元数转欧拉角
     * @param {Quaterniond} 四元数
     * @return {Vector3d} 对应的欧拉角
     */
Vector3d TransForms::Quat2Eular(Quaterniond quat)
{
    return Mat2Euler(quat.matrix());
}

/**
     * @description: 四元数转弧度制欧拉角(角度制)
     * @param {Quaterniond} 四元数
     * @return {Vector3d} 对应的欧拉角
     */
Vector3d TransForms::Quat2EularAngle(Quaterniond quat)
{
    return Mat2EulerAngle(quat.matrix());
}

/**
     * @description: 旋转矩阵转四元数
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Quaterniond} 对应的四元数
     */
Quaterniond TransForms::Mat2Quat(Matrix3d mat)
{
    return Quaterniond(mat);
}

/*---------------------------------------齐次矩阵部分----------------------------------------*/
/**
     * @description: 通过位置和欧拉角合成一个齐次矩阵
     * @param {Vector3d} positon 平移位置
     * @param {Vector3d} rotEular  旋转变换(欧拉角形式)
     * @return {*}
     */
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

/**
     * @description: 通过位置和四元数合成一个齐次矩阵
     * @param {Vector3d} positon 平移位置
     * @param {Quaterniond} quat 四元数
     * @return {Matrix4d} 齐次矩阵
     */
Matrix4d TransForms::Compose(Vector3d positon, Quaterniond quat)
{
    return Compose(positon, Quat2Eular(quat));
}

/**
     * @description: 通过三个位置和三个欧拉角合成一个齐次矩阵
     * @param {double} x 沿x轴的平移
     * @param {double} y 沿y轴的平移
     * @param {double} z 沿z轴的平移
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Matrix4d} 返回4✖4的齐次变换矩阵
     */
Matrix4d TransForms::ComposeEuler(const double x, const double y, const double z, const double rx, const double ry, const double rz)
{
    Eigen::Vector3d rot(rx, ry, rz);
    Eigen::Vector3d pos(x, y, z);
    return TransForms::Compose(pos, rot);
}

/**
     * @description:  将齐次矩阵转换成平移和欧拉角形式，方便理解
     * @param {Matrix4d} 4✖4的齐次变换矩阵
     * @return {VectorXd} x,y,z,rx,ry,rz
     */
VectorXd TransForms::H2EulerAngle(Matrix4d t)
{
    VectorXd pose = VectorXd(6);
    Matrix3d mt = t.block<3, 3>(0, 0);
    Vector3d p3 = t.block<3, 1>(0, 3).col(0);
    pose(0, 0) = p3.x();
    pose(1, 0) = p3.y();
    pose(2, 0) = p3.z();
    Vector3d eular = Mat2EulerAngle(mt);
    pose(3, 0) = eular.x();
    pose(4, 0) = eular.y();
    pose(5, 0) = eular.z();
    return pose;
}