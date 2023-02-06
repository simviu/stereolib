/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "vsn/vsnLib.h"

namespace stereo
{
    using namespace ut;
    //-----
    class StereoCmd : public Cmd
    {
    public:
        StereoCmd(){ init_cmds(); }
    protected:
        void init_cmds();
        bool init(CStrs& args);
    };
}
