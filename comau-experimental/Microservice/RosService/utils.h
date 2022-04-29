#pragma once
#include "qstring.h"
struct gripper
{
    QString cad_path, name, parent, child, tool_path;
    double x, y, z, r, p, yw;
    bool load_gripper;
};

struct tool
{
    QString parent, child;
    double x, y, z, r, p, yw;
};

struct waypoint
{
    double x, y, z, r, p, yw,qx,qy,qz,qw;
};

struct pointCloud
{
    QString path;
    double x,y,z,r,p,yw,res;
};
