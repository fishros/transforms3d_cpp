/*
 * @Descripttion:
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 23:39:20
 */
#include <cmath>
#include <iostream>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;

#include "gtest/gtest.h"

TEST(TestTransForm, Euler2Mat)
{
    IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    std::string sep = "\n----------------------------------------\n";

    Eigen::Vector3d rot(76, 14, -72.511); // r p y
    Matrix3d matrix = TransForms::EulerAngle2Mat(rot);
    Vector3d rotr = TransForms::Mat2EulerAngle(matrix);
    std::cout << matrix.format(HeavyFmt) << sep;
    std::cout << matrix.eulerAngles(2, 1, 0).format(HeavyFmt) << sep;

    std::cout << rotr.x() << " " << rotr.y() << " " << rotr.z() << std::endl;
    //   return TransForms::Compose(pos, rot);
    /* base@llaser1  */
    Matrix4d base2llaser1 = TransForms::ComposeEuler(0.043, -0.163, 0.0571, 76, 14, -72.511); // xyz yam pitch roll
    std::cout << base2llaser1.format(HeavyFmt) << sep;
    // base2llaser1
}
