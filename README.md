
大家好,我是小鱼,最近因为工作上的需要,把自己一两年前做的开源库又进行了维护,新增了点云坐标转换功能,小鱼测试速度可以嗖嗖的.再次分享给大家.

这个库功能和ROS的TF相似,但完全基于Eigen实现,不用像ROS那样需要很多依赖,在自己开发机器人和导航系统的时候会派上用场.

开源地址1:https://gitee.com/ohhuo/transforms3d_cpp
开源地址2:https://github.com/fishros/transforms3d_cpp

# 基于Eigen的坐标转换库-TransForms3d

- 实现一个更强大的坐标转换组，可以进行坐标关系推算，解放你的笔头和双手
- 增加点云转换函数,可以将点云从一个坐标系转到任意一个关联坐标系
- 实现欧拉角与旋转矩阵的互相转换，欧拉角到四元数的转换
- 实现四元数与旋转矩阵的互相转换，四元数到欧拉角的转换
- 实现通过齐次矩阵合成与转换


Git：[仓库地址](https://gitee.com/ohhuo/transforms3d_cpp) | Wiki：[函数列表](https://gitee.com/ohhuo/transforms3d_cpp/wikis/pages)


## 一、安装与使用

### 1.源码引入

复制`trans_forms_group.cpp`,`trans_forms.cpp`,`transforms3d.h`到你的工程即可

### 2.编译安装

```
git clone https://gitee.com/ohhuo/transforms3d_cpp.git
cd transforms3d_cpp
mkdir build && cd build
cmake ..
sudo make install
sudo ldconfig
```
### 3.使用样例

例子1：利用TransformsGroup进行手眼矩阵估算

```c++
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

TEST(TestTransFormGroup, BaseFindPath) {
  /* base@grapper  */
  Matrix4d Tbg =
      TransForms::ComposeEuler(-0.544, -0.203, -0.037, 180, 0.00000, 140);
  /*  camera@maker*/
  Matrix4d Tcw = TransForms::ComposeEuler(0.020, -0.040, 0.300, 0, 0, -45);
  /*  base@bottle  */
  Matrix4d Tbw = TransForms::ComposeEuler(-0.663, -0.193, -0.231, -180, 0, 140);

  TransFormsGroup tfg;
  tfg.pushTransForm("base", "grapper", Tbg);
  tfg.pushTransForm("camera", "bottle", Tcw);
  tfg.pushTransForm("base", "bottle", Tbw);

  cout << tfg.toString() << endl;
  cout << TransForms::H2EulerAngle(tfg.getTransForm("grapper", "camera"));
  Matrix4d Tgc = Tbg.inverse() * Tbw * Tcw.inverse();
  cout << TransForms::H2EulerAngle(Tgc);
}
```

例子2:使用TransformsGroup进行点云坐标转换

```cpp
  /* base@llaser1  */
  Matrix4d base2llaser1 = TransForms::ComposeEuler(0, 0, 0, 180, 0, 0);

  TransFormsGroup tfg;
  tfg.pushTransForm("base_link", "laser", base2llaser1);
  std::vector<Vector3d> points;
  for (int i = 0; i < 2; i++)
  {
    points.push_back({0, 0, (float)(i + 1) / 1000.0});
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
  }
  tfg.getTransWithPointCloud("laser", points, "base_link");
  for (int i = 0; i < 2; i++)
  {
    printf("%d(%f,%f,%f),", i, points[i].x(), points[i].y(), points[i].z());
    EXPECT_DOUBLE_EQ(points[i].z(), -(float)(i + 1) / 1000.0);
  }
```

例子3: 欧拉角转换

```cpp
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
    /* base@llaser1  */
    Matrix4d base2llaser1 = TransForms::ComposeEuler(0.043, -0.163, 0.0571, 76, 14, -72.511); // xyz yam pitch roll
    std::cout << base2llaser1.format(HeavyFmt) << sep;
    // base2llaser1
}
```

## 二、 函数列表

### 1.基础部分

#### 1.1 角度转换

##### 角度转弧度

```c++
    /**
     * @description: 角度转为弧度
     * @param {double} angle 角度值
     * @return 返回对应弧度值,一般在-3.14~3.14之间
     */
    static double Degrees(double angle);
```

##### 弧度转角度

```c++
    /**
     * @description: 弧度转为角度
     * @param {double} degrees 弧度值
     * @return 返回对应的角度值，一般在-180~180之间
     */
    static double Angle(double degrees);
```

#### 1.2 欧拉角部分

##### 角度制欧拉角转旋转矩阵

```c++
	/**
     * @description: 角度制欧拉角转旋转矩阵,此函数默认的旋转顺序是x-y-z.
     * @param {double} rx 绕x轴的旋转.
     * @param {double} ry 绕y轴的旋转.
     * @param {double} rz 绕z轴的旋转.
     * @return {Matrix3d}  返回3✖3的旋转矩阵.
     */
    static Matrix3d EulerAngle2Mat(double rx, double ry, double rz);
```

##### 欧拉角转旋转矩阵

```c++
    /**
     * @description: 欧拉角转旋转矩阵
     * @param {Vector3d} eular 欧拉角rx,ry,rz 
     * @return {Matrix3d} 返回3✖3的旋转矩阵.
     */
    static Matrix3d EulerAngle2Mat(Vector3d eular);
```

##### 欧拉角转四元数

```c++
    /**
     * @description:欧拉角转四元数
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Quaterniond} 返回对应的四元数
     */
    static Quaterniond Euler2Quat(double rx, double ry, double rz);
```

##### 角度制欧拉角转四元数

```c++
    /**
     * @description: 角度制欧拉角转四元数
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {Quaterniond} 返回对应的四元数
     */
    static Quaterniond EulerAngle2Quat(double rx, double ry, double rz);
```

##### 旋转矩阵转欧拉角（弧度制）

```c++
    /**
     * @description: 旋转矩阵转欧拉角(弧度制)
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Vector3d} 欧拉角
     */
    static Vector3d Mat2Euler(Matrix3d mat);
```

##### 欧拉角转旋转矩阵

```c++
  /**
     * @description: 欧拉角转旋转矩阵
     * @param {double} rx 绕x轴的旋转
     * @param {double} ry 绕y轴的旋转
     * @param {double} rz 绕z轴的旋转
     * @return {*}
     */
    static Matrix3d Euler2Mat(double rx, double ry, double rz);
```

##### 旋转矩阵转角度制欧拉角

```c++
    /**
     * @description: 旋转矩阵转角度制欧拉角(角度制)
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Vector3d}  欧拉角 
     */
    static Vector3d Mat2EulerAngle(Matrix3d mat);
```

#### 1.3 四元数部分

##### 四元数转旋转矩阵

```c++
    /**
     * @description: 四元数转旋转矩阵
     * @param {Quaterniond} 四元数
     * @return {Matrix3d} 对应的旋转矩阵
     */
    static Matrix3d Quat2Mat(Quaterniond quat);
```

##### 四元数转欧拉角

```c++
    /**
     * @description: 四元数转欧拉角
     * @param {Quaterniond} 四元数
     * @return {Vector3d} 对应的欧拉角
     */
    static Vector3d Quat2Eular(Quaterniond quat);
```

##### 四元数转弧度制欧拉角(角度制)

```c++
    /**
     * @description: 四元数转弧度制欧拉角(角度制)
     * @param {Quaterniond} 四元数
     * @return {Vector3d} 对应的欧拉角
     */
    static Vector3d Quat2EularAngle(Quaterniond quat);
```

##### 四元数转弧度制欧拉角(角度制)

```c++
    /**
     * @description: 旋转矩阵转四元数
     * @param {Matrix3d} 3✖3的旋转矩阵
     * @return {Quaterniond} 对应的四元数
     */
    static Quaterniond Mat2Quat(Matrix3d mat);
```

#### 1.4 齐次矩阵部分

##### 通过位置和欧拉角合成一个齐次矩阵

```c++
    /**
     * @description: 通过位置和欧拉角合成一个齐次矩阵
     * @param {Vector3d} positon 平移位置
     * @param {Vector3d} rotEular  旋转变换(欧拉角形式)
     * @return {*}
     */
    static Matrix4d Compose(Vector3d positon, Vector3d rotEular);
```

##### 通过位置和四元数合成一个齐次矩阵

```c++
    /**
     * @description: 通过位置和四元数合成一个齐次矩阵
     * @param {Vector3d} positon 平移位置
     * @param {Quaterniond} quat 四元数
     * @return {Matrix4d} 齐次矩阵
     */
    static Matrix4d Compose(Vector3d positon, Quaterniond quat);
```

##### 通过三个位置和三个欧拉角合成一个齐次矩阵

```c++
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
```

##### 将齐次矩阵转换成平移和欧拉角形式，方便理解

```c++
    /**
     * @description:  将齐次矩阵转换成平移和欧拉角形式，方便理解
     * @param {Matrix4d} 4✖4的齐次变换矩阵
     * @return {VectorXd} x,y,z,rx,ry,rz
     */
    static VectorXd H2EulerAngle(Matrix4d t);
```

### 2.坐标变换组

#### 2.1 添加坐标转换组
##### 添加一个变换关系，通过齐次矩阵
```c++
/**
* @description:  添加一个变换关系，通过齐次矩阵
* @param {const std::string&} parent  父节点名字
* @param {const std::string&} child  子节点名字
* @param {Matrix4d} 父节点到子节点之间的变换关系
* @return {Matrix4d} 返回4✖4的齐次变换矩阵
*/   
Matrix4d pushTransForm(const std::string& parent, const std::string& child,Matrix4d matrix);
```

##### 添加一个变换关系，通过位置和欧拉角
```c++
/**
* @description:  添加一个变换关系，通过位置和欧拉角
* @param {const std::string&} parent  父节点名字
* @param {const std::string&} child  子节点名字
* @param {double&} x 沿x轴的平移
* @param {double&} y 沿y轴的平移
* @param {double&} z 沿z轴的平移
* @param {double&} rx 绕x轴的旋转
* @param {double&} ry 绕y轴的旋转
* @param {double&} rz 绕z轴的旋转
* @return {Matrix4d} 返回4✖4的齐次变换矩阵
*/   
Matrix4d pushTransForm(const std::string& parent, const std::string& child,double& x, double& y, double& z, double &rx, double& ry, double & rz);
```

##### 添加一个变换关系,通过位置和四元数
```c++
    /**
* @description:  添加一个变换关系,通过位置和四元数
* @param {const std::string&} parent  父节点名字
* @param {const std::string&} child  子节点名字
    * @param {double&} x 沿x轴的平移
    * @param {double&} y 沿y轴的平移
    * @param {double&} z 沿z轴的平移
    * @param {double&} rx 四元数对应的x
    * @param {double&} ry 四元数对应的y
    * @param {double&} 四元数对应的z
    * @param {double&} 四元数对应的w
    * @return {Matrix4d} 返回4✖4的齐次变换矩阵
*/   
Matrix4d pushTransForm(const std::string& parent, const std::string& child,double& x, double& y, double& z, double &rx, double& ry, double & rz,double & rw);
```

#### 2.2 打印坐标转换组
##### 打印坐标转换组
```c++
/**
* @description: 将存储的节点数据按照字符串形式输出
* @param {*} 无
* @return {std::string}  字符串
*/
std::string toString();
```

#### 2.3 查找坐标关系
##### 获取坐标之间的变换转换关系
```c++
/**
* @description:  获取两个相关节点的变换关系
* @param {const std::string&} start  开始节点名字
* @param {const std::string&} end  结束节点名字
    * @return {Matrix4d} 返回4✖4的齐次变换矩阵
*/   
Matrix4d getTransForm(const std::string& start, const std::string& end);
```
##### 获取两个节点之间的最短路径
```c++
/**
* @description: 查找两个节点之间的路径
* @param {const std::string&} start  开始节点名字
* @param {const std::string&} end  结束节点名字
* @return {std::vector<std::string> }  路径数组 
*/   
std::vector<std::string> findPath(const std::string& start, const std::string& end);
```


## 三、鸣谢与反馈

### 1. 贡献

- 作者:小鱼 | Autor:fishros
- 邮箱 | Email fishros@foxmail.com

## 2. 反馈

- 在鱼香社区提问(fishros.org.cn)
- 提出Issuce

