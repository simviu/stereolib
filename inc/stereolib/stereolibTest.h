/*
   Author: Sherman Chen
   Create Time: 2023-02-06
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "stereolib/stereolib.h"

using namespace stereo;
namespace test
{
    //----
    class TestVO : public ut::Test{
    public:
        virtual bool run() override;
    protected:
    };    
    //-----
    class TestStereo : public Test
    {
    public:
        virtual bool run() override;
    protected:
        bool testKittyGray()const;
        bool test_imgLR()const;
        bool test_misc();

    };

}
