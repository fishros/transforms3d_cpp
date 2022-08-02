/*
 * @Descripttion:
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 23:39:20
 */
#include "math.h"
#include <iostream>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;

#include "gtest/gtest.h"

TEST(TestTransFormGroup, TransPcl) {
  /* base@llaser1  */
  Matrix4d base2llaser1 = TransForms::ComposeEuler(0, 0, 0, 180, 0, 0);

  TransFormsGroup tfg;
  tfg.pushTransForm("base_link", "line_laser1", base2llaser1);
  std::vector<Vector3d> points;
  for (int i = 0; i < 2; i++) {
    points.push_back({0, 0, (float)(i + 1) / 1000.0});
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
  }
  tfg.getTransWithPointCloud("line_laser1", points, "base_link");
  for (int i = 0; i < 2; i++) {
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
    EXPECT_DOUBLE_EQ(points[i].z(), -(float)(i + 1) / 1000.0);
  }
}

TEST(TestTransFormGroup, TransPcl2) {
  /* base@llaser1  */
  Matrix4d base2llaser1 = TransForms::ComposeEuler(0, 0, 1, 0, 0, 0);

  TransFormsGroup tfg;
  tfg.pushTransForm("base_link", "line_laser1", base2llaser1);
  std::vector<Vector3d> points;
  for (int i = 0; i < 20000; i++) {
    points.push_back({0, 0, (float)(i + 1) / 1000.0});
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
  }
  tfg.getTransWithPointCloud("line_laser1", points, "base_link");
  for (int i = 0; i < 20000; i++) {
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
    EXPECT_DOUBLE_EQ(points[i].z(), 1 + (float)(i + 1) / 1000.0);
  }
}
