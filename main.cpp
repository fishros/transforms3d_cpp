#include "math.h"
#include <iostream>
#include <TransForms.h>

using namespace std;
using namespace Eigen;

int main()
{
    Matrix4d Tbg = TransForms::ComposeEuler(3,-294,375,180,0,135);
    Matrix4d Tcw  = TransForms::ComposeEuler(-2,29,300,0,0,0);
    Matrix4d Tbw  = TransForms::ComposeEuler(3,-343,163.6,180,0,-135);
    Matrix4d Tgc = Tbg.inverse()*Tbw*Tcw.inverse();
    cout<<Tbg.inverse()*Tbw*Tcw.inverse()<<endl;


    Tbg = TransForms::ComposeEuler(3.000006,-279.999992,374.999991,179.999997,0.000001,-134.999998);
    Tcw = TransForms::ComposeEuler(-2.48,15.66,300,0,0,0);
    Tbw =  TransForms::ComposeEuler(3,-343,163.6,180,0,-135);
    cout<<Tbg.inverse()*Tbw*Tcw.inverse()<<endl;
    // Tgc = Tbg.inverse()*Tbw*Tcw.inverse();

    cout<<Tbg*Tgc*Tcw<<endl;
}
// int main()
// {
//     Matrix4d Tbg = TransForms::ComposeEuler(0,-265.5,75,0,0,0);
// //     Matrix4d Tbc  = TransForms::ComposeEuler(-3.2,-345.66,164.231,0,0,0);
// //     cout<<"--------------------------末端到法兰坐标系--------------------------------"<<endl;
// //     cout<<TransForms::ComposeEuler(0,0,206,0,0,0)<<endl;

// //     cout<<"----------------------------Tbc-----------------------------------"<<endl;
// //     cout<<Tbc<<endl;
// //     cout<<"----------------------------Tbg-----------------------------------"<<endl;
// //     cout<<Tbg<<endl;
// //    Matrix4d Tgc = Tbg.inverse()*Tbc;
// //     cout<<"----------------------------Tgc-----------------------------------"<<endl;
// //     Tgc = TransForms::ComposeEuler(80,0,89,0,0,90);
// //    cout<<Tgc<<endl;
    
//     Tbg = TransForms::ComposeEuler(2.999999,-294.658329,374.999978,179.999996,0.000001,-135.000000);
//     Matrix4d Tcw = TransForms::ComposeEuler(-2 ,29,300,0,0,0);


//     Matrix3d rot = TransForms::EulerAngle2Mat(0,0,45);
//     Vector3d trans(0.0,-80.0,-89.0);
//     Matrix4d Tgc = TransForms::ComposeEuler(0.0,-80.0,-89.0,0,0,45);
//     cout<<Tgc<<endl;

//     cout<<"----------------------------Tbw-----------------------------------"<<endl;
//     cout<<Tbg*Tgc<<endl; 


//     return 0;
// }