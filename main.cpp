#include "math.h"
#include <iostream>
#include <TransForms.h>

using namespace std;
using namespace Eigen;


int main()
{
    Matrix4d Tbg = TransForms::ComposeEuler(3.000006, -279.999992, 375, 180, 0.000001, -135);
    Matrix4d Tcw  = TransForms::ComposeEuler(-2.48,15.66,300,0,0,45);
    Matrix4d Tbw  = TransForms::ComposeEuler(-0.663,-0.193,-0.231,-180,0,140);
    /* Tbw=Tbg*Tgc*Tcw*/
    Matrix4d Tgc = Tbg.inverse()*Tbw*Tcw.inverse(); 
    // cout<<"------------------Tgc1---------------"<<endl;
    // cout<<Tbg.inverse()*Tbw*Tcw.inverse()<<endl;
    cout<<TransForms::H2EulerAngle(Tgc);
}