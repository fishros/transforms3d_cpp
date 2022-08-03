#include <iostream>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "gtest/gtest.h"
#include <cmath>
#include <transforms3d/transforms3d.h>

TEST(TestEigen, Euler2Mat)
{
    IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    Eigen::Vector3d rpy_raw, ypr;
    rpy_raw <<  76, 14, -72.511;
    rpy_raw = rpy_raw * M_PI / 180;
    std::string sep = "\n----------------------------------------\n";

    Eigen::Matrix3d R_AB,N_AB;
    //输入部分为YPR,所以转换的输出也是YPR
    R_AB = Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX());
    std::cout << R_AB.format(HeavyFmt) << sep;
    
    ypr = R_AB.eulerAngles(2, 1, 0);

    N_AB = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());

    std::cout << N_AB.format(HeavyFmt) << sep;


    std::cout << ypr * 180 / M_PI << std::endl;
}