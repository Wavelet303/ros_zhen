/**
 * \file        FEATUREDEF.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       manipulation features
 */

#ifndef FEATUREDEF
#define FEATUREDEF

struct bottle_features{
    float loc[3];
    float ori[3];
    int color[3];
    float size[2]; //radius, height
};

struct gripper_features{
    float loc[3];
};

struct manipulation_features{
    bottle_features bottle;
    gripper_features gripper_1;
    gripper_features gripper_2;
};

#endif  // FEATUREDEF