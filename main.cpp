#include "math.h"
#include <iostream>
#include <TransForms.h>

using namespace std;
using namespace Eigen;


int main()
{
    Matrix4d Tbg = TransForms::ComposeEuler(3,-294,375,180,0,-135);
    Matrix4d Tcw  = TransForms::ComposeEuler(-2,29,300,0,0,0);
    Matrix4d Tbw  = TransForms::ComposeEuler(3,-343,163.6,180,0,-135);
    Matrix4d Tgc = Tbg.inverse()*Tbw*Tcw.inverse();
    cout<<"------------------Tgc1---------------"<<endl;
    cout<<Tbg.inverse()*Tbw*Tcw.inverse()<<endl;


    Tbg = TransForms::ComposeEuler(3.000006,-279.999992,374.999991,179.999997,0.000001,-134.999998);
    Tcw = TransForms::ComposeEuler(-2.48,15.66,300,0,0,0);
    Tbw =  TransForms::ComposeEuler(3,-343,163.6,180,0,-135);
    cout<<"------------------Tgc2---------------"<<endl;
    cout<<Tbg.inverse()*Tbw*Tcw.inverse()<<endl; 

    cout<<Tbg*Tgc*Tcw<<endl;
    // cout<<Tbw.block(0,3,1,1)<<endl;
    cout<<TransForms::H2EulerAngle(Tcw.inverse())<<endl;
    // Matrix3d mt  = Tbw.block<3,3>(0,0);
    // Vector3d p3 = Tbw.block<3,1>(0,3).col(0);
    // cout<<p3<<endl;
    // cout<<p3.x()<<endl;
    // cout<<Tbw.block<3,1>(0,3).resize(1,3).row(0)<<endl;
}