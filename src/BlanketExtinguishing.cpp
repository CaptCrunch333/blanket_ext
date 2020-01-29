#include "BlanketExtinguishing.hpp"

BlanketExtinguishing::BlanketExtinguishing()
{
    _tolerance.x = 0.1;
    _tolerance.y = 0.1;
    _tolerance.z = 0.1;
    _yaw_tolerance = 0.1;
    _ros_status = STATUS::FIRST_ROD_RELEASE;
    _m_first_rod_angle = 50 * M_PI/180.f;
    _m_second_rod_angle = 70 * M_PI/180.f;
}

void BlanketExtinguishing::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::POINT)
    {
        Vector3DMessage* pose_msg = (Vector3DMessage*) t_msg;
        m_curr_pose.x = pose_msg->getData().x;
        m_curr_pose.y = pose_msg->getData().y;
        m_curr_pose.z = pose_msg->getData().z;
    }
    if(t_msg->getType() == msg_type::FLOAT)
    {
        m_curr_heading = ((FloatMsg*)t_msg)->data;
    }        
}

void BlanketExtinguishing::receive_msg_data(DataMessage* t_msg, int t_channel)
{
    if(t_channel == (int) FIRST_ROD)
    {
        PoseMsg* t_pose = (PoseMsg*) t_msg;
        _first_position.x = t_pose->pose.x;
        _first_position.y = t_pose->pose.y;
        _first_position.z = t_pose->pose.z;
        _first_heading = t_pose->pose.yaw;
       //TODO trigger first ROD 
    } else if (t_channel == (int) SECOND_ROD)
    {
        PoseMsg* t_pose = (PoseMsg*) t_msg;
        _second_position.x = t_pose->pose.x;
        _second_position.y = t_pose->pose.y;
        _second_position.z = t_pose->pose.z;
        _second_heading = t_pose->pose.yaw;
        //TODO trigger second ROD 
    }
     else if (t_channel == (int) UAV_POSITION)
    {
        Vector3DMessage* t_pose = (Vector3DMessage*) t_msg;
        _UAV_position.x = t_pose->getData().x;
        _UAV_position.y = t_pose->getData().y;
        _UAV_position.z = t_pose->getData().z;
        if (_ros_status == STATUS::FIRST_ROD_RELEASE)
        {   
            if(fabs(_UAV_position.x -_first_position.x) < _tolerance.x  && fabs(_UAV_position.y -_first_position.y) < _tolerance.y && fabs(_UAV_position.z -_first_position.z) < _tolerance.z && fabs(_UAV_Yaw -_first_heading) < _yaw_tolerance ) 
            {
                ControllerOutputMsg first_rod_msg;
                first_rod_msg.setControlSignal(_m_first_rod_angle, control_system::null_type);
                emit_message((DataMessage*) & first_rod_msg);
                Logger::getAssignedLogger()->log("first released", LoggerLevel::Info);
                _ros_status = STATUS::SECOND_ROD_RELEASE;
            }   
        } 
        else if (_ros_status == STATUS::SECOND_ROD_RELEASE)
        {
            if(fabs(_UAV_position.x -_second_position.x) < _tolerance.x  && fabs(_UAV_position.y -_second_position.y) < _tolerance.y && fabs(_UAV_position.z -_second_position.z) < _tolerance.z && fabs(_UAV_Yaw -_second_heading) < _yaw_tolerance ) 
            {
                ControllerOutputMsg second_rod_msg;
                second_rod_msg.setControlSignal(_m_second_rod_angle, control_system::null_type);
                emit_message((DataMessage*) & second_rod_msg);
                Logger::getAssignedLogger()->log("second released", LoggerLevel::Info);
                _ros_status = STATUS::BLANKET_RELEASED;
            }
        }
    }
    else if (t_channel == (int) UAV_ORIENTATION)
    {
        Vector3DMessage* t_pose = (Vector3DMessage*) t_msg;
        _UAV_Yaw = t_pose->getData().z;
        //TODO trigger second ROD 
    }
} 

