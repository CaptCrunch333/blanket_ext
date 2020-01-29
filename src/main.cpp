#include "ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "BlanketExtinguishing.hpp"
#include <vector>
#include "linux_serial_comm_device.hpp"
#include "linux_serial_comm_device.hpp"
#include "BaseCommunication.hpp"
#include "looper.hpp"
#include "CommChecker.hpp"
#include<iostream>

#define test_messages

int main(int argc, char **argv){

    ros::init(argc, argv, "ugv_nav");
    ros::NodeHandle nh;
    //ros::Rate loop_rate(100);
    // ************************************ LOGGER ************************************
    Logger::assignLogger(new StdLogger());
    Logger::getAssignedLogger()->log("start of logger", LoggerLevel::Info);
    // ********************************************************************************
    // ***************************** COMMUNICATION DEVICE *****************************
    LinuxSerialCommDevice* mainCommDevice = new LinuxSerialCommDevice;
    BaseCommunication* mainCommStack = new BaseCommunication((CommDevice*) mainCommDevice);
    std::string port_add = "/dev/Razor";
    mainCommDevice->attach_hardware_sender((void*) &port_add);
    CommChecker* mainCommChecker = new CommChecker(mainCommDevice, (void*) &port_add, block_frequency::hz10);
    // ********************************************************************************
    // ********************************** ROS UNITS  **********************************
    ROSUnit_Factory* mainROSFactory = new ROSUnit_Factory(nh);
    ROSUnit* UAVPositionSubscriber = mainROSFactory->CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, ROSUnit_Point, "/uav_control/uav_position");
    ROSUnit* UAVOrientationSubscriber = mainROSFactory->CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, ROSUnit_Point, "uav_control/uav_orientation");
    ROSUnit* FirstRodSrv = mainROSFactory->CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_Pose, "/blanket_ext/set_release_first_rod");
    ROSUnit* SecondRodSrv = mainROSFactory->CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_Pose, "/blanket_ext/set_release_second_rod");
    // ********************************************************************************
    // ************************* BLANKET EXTINGUISHING LOGIC **************************
    BlanketExtinguishing* MainBlanketExtignishing = new BlanketExtinguishing();
    // ********************************************************************************
    // ****************************** SYSTEM CONNECTIONS ******************************
    FirstRodSrv->add_callback_msg_receiver((msg_receiver*) MainBlanketExtignishing);
    FirstRodSrv->setEmittingChannel(BlanketExtinguishing::FIRST_ROD); 

    SecondRodSrv->add_callback_msg_receiver((msg_receiver*) MainBlanketExtignishing);
    SecondRodSrv->setEmittingChannel(BlanketExtinguishing::SECOND_ROD);

    UAVPositionSubscriber->add_callback_msg_receiver((msg_receiver*) MainBlanketExtignishing);
    UAVPositionSubscriber->setEmittingChannel(BlanketExtinguishing::UAV_POSITION);

    UAVOrientationSubscriber->add_callback_msg_receiver((msg_receiver*) MainBlanketExtignishing);
    UAVOrientationSubscriber->setEmittingChannel(BlanketExtinguishing::UAV_ORIENTATION);

    MainBlanketExtignishing->add_callback_msg_receiver((msg_receiver*) mainCommStack);

    mainCommChecker->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    mainCommStack->add_callback_msg_receiver((msg_receiver*) mainCommChecker);
    // ********************************************************************************
    // ********************************* LOOPER SETUP *********************************
    pthread_t loop10hz_func_id;
    Looper* main_looper = new Looper();
    main_looper->addTimedBlock((TimedBlock*) mainCommChecker);
    pthread_create(&loop10hz_func_id, NULL, &Looper::Loop10Hz, NULL);
    // ********************************************************************************
    #ifdef test_messages
    sleep(1);
    Logger::getAssignedLogger()->log("Start Of Testing Mode!!!", LoggerLevel::Warning);
    Vector3DMessage uav_pos;
    Vector3D<double> uav_pos_xyz;
    uav_pos_xyz.x=3;
    uav_pos_xyz.y=3;
    uav_pos_xyz.z=3;
    uav_pos.setVector3DMessage(uav_pos_xyz);
    MainBlanketExtignishing->receive_msg_data((DataMessage*)&uav_pos,BlanketExtinguishing::UAV_POSITION);
    //ros::spinOnce();
    usleep(100000);
    PoseMsg first_pt,second_pt;
    first_pt.pose.x=2;
    first_pt.pose.y=2;
    first_pt.pose.z=2;
    first_pt.pose.yaw=0;
    second_pt.pose.x=4;
    second_pt.pose.y=2;
    second_pt.pose.z=2;
    second_pt.pose.yaw=0;
    MainBlanketExtignishing->receive_msg_data((DataMessage*)&first_pt,BlanketExtinguishing::FIRST_ROD);
    MainBlanketExtignishing->receive_msg_data((DataMessage*)&second_pt,BlanketExtinguishing::SECOND_ROD);
    //ros::spinOnce();
    usleep(100000);
    uav_pos_xyz.x=2;
    uav_pos_xyz.y=2;
    uav_pos_xyz.z=2;
    uav_pos.setVector3DMessage(uav_pos_xyz);
    Logger::getAssignedLogger()->log("Dropping First Rod Test!!!", LoggerLevel::Warning);
    MainBlanketExtignishing->receive_msg_data((DataMessage*)&uav_pos,BlanketExtinguishing::UAV_POSITION);
    //ros::spinOnce();
    usleep(5000000);
    uav_pos_xyz.x=4;
    uav_pos_xyz.y=2;
    uav_pos_xyz.z=2;
    uav_pos.setVector3DMessage(uav_pos_xyz);
    Logger::getAssignedLogger()->log("Dropping Second Rod Test!!!", LoggerLevel::Warning);
    MainBlanketExtignishing->receive_msg_data((DataMessage*)&uav_pos,BlanketExtinguishing::UAV_POSITION);
    //ros::spinOnce();
    #endif
    while (ros::ok()){
    
        ros::spinOnce();
        //loop_rate.sleep();
    }
    return 0;
}