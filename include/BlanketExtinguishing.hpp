#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "PoseMsg.hpp"
#include <math.h>
#include "Vector3D.hpp"
#include "PointsMsg.hpp"
#include "FloatMsg.hpp"
#include"ControllerOutputMsg.hpp"
#include"Vector3DMessage.hpp"
#include "logger.hpp"

enum class STATUS {FIRST_ROD_RELEASE = 1, SECOND_ROD_RELEASE = 2,BLANKET_RELEASED = 3};
class BlanketExtinguishing : public msg_emitter, public msg_receiver
{
    public:
    enum Recieving_Channel {FIRST_ROD, SECOND_ROD, UAV_POSITION, UAV_ORIENTATION};
    BlanketExtinguishing();
    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage*, int);

    private:
    Vector3D<double> _first_position;
    Vector3D<double> _second_position;
    Vector3D<double> _UAV_position; 
    double _UAV_Yaw; 
    float _first_heading;
    float _second_heading;
    STATUS _ros_status;
    Vector3D<double> _tolerance;
    double _yaw_tolerance;
    float _m_first_rod_angle;
    float _m_second_rod_angle;
    Vector3D<float> m_curr_pose;
    float m_curr_heading;
};