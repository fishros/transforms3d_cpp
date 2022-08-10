/*
 * @Descripttion:
 * @Author: 小鱼
 * @Date: 2021-08-09 21:04:19
 * @LastEditTime: 2021-08-09 23:39:20
 */
#include <cmath>
#include <iostream>
#include <transforms3d/pcdtool.h>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;

#include "gtest/gtest.h"

TEST(TestPcdTool, TestRadiusFilter)
{
    PointCloud pcd;
    PointCloud cloud_filted;
    int size = 5;
    for (int i = 0; i < size; i++)
    {
        pcd.pts.push_back({0, 0, (float)(i + 1) / 10.0});
    }
    pcd.pts.push_back({0, 2.0, 1 / 1000.0});

    RadiusFilter(pcd, cloud_filted, 0.1, 4);

    printf("\n=====滤波前=====\n");
    for (int i = 0; i < pcd.pts.size(); i++)
    {
        printf("%d(%f,%f,%f),", i, pcd.pts[i].x(), pcd.pts[i].y(),
               pcd.pts[i].z());
    }
    printf("\n=====滤波后=====\n");
    for (int i = 0; i < cloud_filted.pts.size(); i++)
    {
        printf("%d(%f,%f,%f),", i, cloud_filted.pts[i].x(), cloud_filted.pts[i].y(),
               cloud_filted.pts[i].z());
    }
}