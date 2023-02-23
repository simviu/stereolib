

#include "stereolib/stereolibTest.h"

using namespace rov;
using namespace ut;
using namespace test;
using namespace cv;
using namespace vsn;
using namespace stereo;

namespace{
    bool test_oak_cfg()
    {
        //--- T of Left cam to right cam
        mat3 R21; 
        R21 <<  9.99782860e-01, -7.99524784e-03, -1.92424282e-02,
                7.98463263e-03,  9.99967933e-01, -6.28410256e-04,
                1.92468371e-02,  4.74630098e-04,  9.99814630e-01;

        vec3 t21; t21 << -7.54527378e+00, 2.22705342e-02, -7.59122521e-02;
        quat q21(R21);
        cout << "t21=" << t21.transpose() << endl;
        cout << "q21=" << q21 << endl;
        cv::Mat r21;
    //    cv::Rodrigues(R21, r21);
        //---- T of left cam to RGB cam
        mat3 Rc1;
        Rc1 <<  9.99720037e-01, -1.55289015e-02, -1.78512111e-02,
                1.54857030e-02,  9.99876857e-01, -2.55559012e-03,
                1.78886969e-02,  2.27843621e-03,  9.99837399e-01;
        vec3 tc1;
        tc1 << -3.79904151e+00, 6.28807321e-02, -2.24310070e-01;
        cv::Mat rc1;
        quat qc1(Rc1);
    //    cv::Rodrigues(Rc1, rc1);
        cout << "tc1=" << tc1.transpose() << endl;
        cout << "qc1=" << qc1 << endl;

         //---- check quat init
         Pose T;
         cout << "tmp T=" << T.str();

        return true;
    }
}

//----
bool TestVO::test_misc()
{
    log_i("test_misc:");
    return test_oak_cfg();
    return true;
}
