/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "a1_msgs/LowCmd.h"
#include "a1_msgs/LowState.h"
#include "a1_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace a1_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern a1_msgs::LowCmd lowCmd;
extern a1_msgs::LowState lowState;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);
}

#endif
