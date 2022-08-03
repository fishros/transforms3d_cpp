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

TEST(TestTransFormGroup, TransPcl)
{
  /* base@llaser1  */
  Matrix4d base2llaser1 = TransForms::ComposeEuler(0, 0, 0, 180, 0, 0);

  TransFormsGroup tfg;
  tfg.pushTransForm("base_link", "line_laser1", base2llaser1);
  std::vector<Vector3d> points;
  for (int i = 0; i < 2; i++)
  {
    points.push_back({0, 0, (float)(i + 1) / 1000.0});
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
  }
  tfg.getTransWithPointCloud("line_laser1", points, "base_link");
  for (int i = 0; i < 2; i++)
  {
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
    EXPECT_DOUBLE_EQ(points[i].z(), -(float)(i + 1) / 1000.0);
  }
}

TEST(TestTransFormGroup, TransPcl2)
{
  /* base@llaser1  */
  Matrix4d base2llaser1 = TransForms::ComposeEuler(0, 0, 1, 0, 0, 0);

  TransFormsGroup tfg;
  tfg.pushTransForm("base_link", "line_laser1", base2llaser1);
  std::vector<Vector3d> points;
  for (int i = 0; i < 20000; i++)
  {
    points.push_back({0, 0, (float)(i + 1) / 1000.0});
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
  }
  tfg.getTransWithPointCloud("line_laser1", points, "base_link");
  for (int i = 0; i < 20000; i++)
  {
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
    EXPECT_DOUBLE_EQ(points[i].z(), 1 + (float)(i + 1) / 1000.0);
  }
}

TEST(TestTransFormGroup, TransPcl3)
{
  IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
  std::string sep = "\n----------------------------------------\n";
  /* base@llaser1  */
  Matrix4d base2llaser1 = TransForms::ComposeEuler(0.043, -0.163, 0.0571, 76, 14, -72.511); // xyz r p y
  std::cout << base2llaser1.format(HeavyFmt) << sep;
  // static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
  // rosrun tf2_ros static_transform_publisher base linerlaser2 0.043 -0.163 0.0571 76 14 -72.511
  // rosrun tf2_ros static_transform_publisher  0.043 -0.163 0.0571 -1.265555  0.24434609  1.3264502 base linerlaser2
  TransFormsGroup tfg_;
  tfg_.pushTransForm("base", "linerlaser2", base2llaser1);
  float angle = 0.299000;
  float dis = 0.033816;
  double x = dis * cos(angle);
  double y = dis * sin(angle);
  printf("%f %f \n", x, y);
  // rosrun tf2_ros static_transform_publisher 0.300746 0.096994 0 0 0 0 linerlaser2 point0
  std::vector<Vector3d> points;
  points.push_back({x, y, 0.0f});
  tfg_.pushTransForm("linerlaser2", "point0", TransForms::ComposeEuler(x, y, 0, 0, 0, 0));
  std::cout << tfg_.toString() << std::endl;
  tfg_.getTransWithPointCloud("linerlaser2", points, "base");
  printf("line_laser(%f,%f,%f)\n", points[0].x(), points[0].y(), points[0].z());
  std::cout << TransForms::H2EulerAngle(
                   tfg_.getTransForm("base", "point0"))
            << std::endl;
}
