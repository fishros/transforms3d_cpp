/*
 * @Descripttion: TransForms Header
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 22:34:48
 */
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <iostream>
#include <math.h>
using namespace Eigen;

class TransForms
{
private:
public:
    /*---------------------------------------角度弧度转换----------------------------------------*/
    /**
     * @description: 角度转为弧度
     * @param {double} angle 角度值
     * @return 返回对应弧度值,一般在-3.14~3.14之间
     */
    static double Degrees(double angle);
    /**
     * @description: 弧度转为角度
     * @param {double} degrees 弧度值
     * @return 返回对应的角度值，一般在-180~180之间
     */
    static double Angle(double degrees);

    /*---------------------------------------欧拉角部分---------------------------*/
    /**
     * @description: 角度制欧拉角转旋转矩阵,此函数默认的旋转顺序是x-y-z.
     * @param {double} rx 绕x轴的旋转.
     * @param {double} ry 绕y轴的旋转.
     * @param {double} rz 绕z轴的旋转.
     * @return {Matrix3d}  返回3✖3的旋转矩阵.
     */
    static Matrix3d EulerAngle2Mat(double rx, double ry, double rz);

    /**
     * @description: 欧拉角转旋转矩阵
     * @param {Vector3d} eular 欧拉角rx,ry,rz 
     * @return {Matrix3d} 返回3✖3的旋转矩阵.
     */
    static Matrix3d EulerAngle2Mat(Vector3d eular);

    /**
     * @description:欧拉角转四元数
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Quaterniond} 返回对应的四元数
     */
    static Quaterniond Euler2Quat(double rx, double ry, double rz);

    /**
     * @description: 角度制欧拉角转四元数
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Quaterniond} 返回对应的四元数
     */
    static Quaterniond EulerAngle2Quat(double rx, double ry, double rz);

    /**
     * @description: 旋转矩阵转欧拉角(弧度制)
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Vector3d} 欧拉角
     */
    static Vector3d Mat2Euler(Matrix3d mat);

  /**
     * @description: 欧拉角转旋转矩阵
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {*}
     */
    static Matrix3d Euler2Mat(double rx, double ry, double rz);


    /**
     * @description: 旋转矩阵转角度制欧拉角(角度制)
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Vector3d}  欧拉角 
     */
    static Vector3d Mat2EulerAngle(Matrix3d mat);

    /*---------------------------------------四元数部分----------------------------------------*/
    /**
     * @description: 四元数转旋转矩阵
     * @param {Quaterniond} 四元数
     * @return {Matrix3d} 对应的旋转矩阵
     */
    static Matrix3d Quat2Mat(Quaterniond quat);

    /**
     * @description: 四元数转欧拉角
     * @param {Quaterniond} 四元数
     * @return {Vector3d} 对应的欧拉角
     */
    static Vector3d Quat2Eular(Quaterniond quat);


    /**
     * @description: 四元数转弧度制欧拉角(角度制)
     * @param {Quaterniond} 四元数
     * @return {Vector3d} 对应的欧拉角
     */
    static Vector3d Quat2EularAngle(Quaterniond quat);

    
    /**
     * @description: 旋转矩阵转四元数
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Quaterniond} 对应的四元数
     */
    static Quaterniond Mat2Quat(Matrix3d mat);

    /*---------------------------------------齐次矩阵部分----------------------------------------*/
    /**
     * @description: 通过位置和欧拉角合成一个齐次矩阵
     * @param {Vector3d} positon 平移位置
     * @param {Vector3d} rotEular  旋转变换(欧拉角形式)
     * @return {*}
     */
    static Matrix4d Compose(Vector3d positon, Vector3d rotEular);

    /**
     * @description: 通过位置和四元数合成一个齐次矩阵
     * @param {Vector3d} positon 平移位置
     * @param {Quaterniond} quat 四元数
     * @return {Matrix4d} 齐次矩阵
     */
    static Matrix4d Compose(Vector3d positon, Quaterniond quat);

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
    static Matrix4d ComposeEuler(double x, double y, double z, double rx, double ry, double rz);
    
    /**
     * @description:  将齐次矩阵转换成平移和欧拉角形式，方便理解
     * @param {Matrix4d} 4✖4的齐次变换矩阵
     * @return {VectorXd} x,y,z,rx,ry,rz
     */
    static VectorXd H2EulerAngle(Matrix4d t);

    TransForms(/* args */) = default;
    ~TransForms() = default;
};
