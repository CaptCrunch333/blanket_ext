#pragma once
#include "ros/ros.h"
#include "ROSUnit.hpp"
#include "Vector3D.hpp"
#include <positioning_system/twoPointLine.h>
#include "FireLineDescription.hpp"
#include <geometry_msgs/Point.h>

class ROSUnit_FireLocationClient : ROSUnit{

private:
    ros::ServiceClient _srv_client;
    static FireLineDescription fireLineDesc_msg;
    void receive_msg_data(DataMessage* t_msg);
    Vector3D<float> _point1;
    Vector3D<float> _point2;
    Vector3D<float> _fire_location;
public:
    
    ROSUnit_FireLocationClient(ros::NodeHandle&);
    ~ROSUnit_FireLocationClient();
};