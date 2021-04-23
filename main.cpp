#include "math.h"
#include <iostream>
#include <TransForms.h>

using namespace std;

int main()
{
    Eigen::Matrix3d mat = TransForms::Euler2Mat(0, 0, M_PI);
    Eigen::Matrix3d mat1 = TransForms::EulerAngle2Mat(0, 0, 180);
    Eigen::Quaterniond q1 = TransForms::Euler2Quat(0, 0, M_PI_2);
    q1 = TransForms::EulerAngle2Quat(0, 0, 90);
    Eigen::Vector3d rot = TransForms::Mat2Euler(mat);
    rot = TransForms::Mat2EulerAngle(mat);
    cout << rot << endl;
    return 0;
}