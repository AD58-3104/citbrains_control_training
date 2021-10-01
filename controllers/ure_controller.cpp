#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "sUDPSocket.hpp"
#include "ure.pb.h"

class UreController
{
public:
    const std::vector<std::string> joint_name_list{
        "finger_1_joint_1",
        "finger_2_joint_1",
        "finger_middle_joint_1",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint"};
    const std::vector<std::string> sensor_name_list{
        "elbow_joint_sensor"
        ,"shoulder_lift_joint_sensor"
        ,"shoulder_pan_joint_sensor"
        ,"wrist_1_joint_sensor"
        ,"wrist_2_joint_sensor"
        ,"wrist_3_joint_sensor"};
    static const int32_t TIME_STEP = 16;
    inline static constexpr int32_t SEND_PORT = 7650;
    inline static constexpr int32_t RECEIVE_PORT = 7651;
    UreController():client_("127.0.0.1",SEND_PORT,Citbrains::Udpsocket::SocketMode::unicast_mode)
    ,server_(RECEIVE_PORT,std::bind(&UreController::receiveDataHandler,this,std::placeholders::_1),Citbrains::Udpsocket::SocketMode::unicast_mode){
        robot_ = new webots::Robot();
        for(const auto& itr:joint_name_list){
            motors_.emplace_back(robot_->getMotor(itr));
        }
        for(const auto& itr:sensor_name_list){
            sensors_.push_back(robot_->getPositionSensor(itr));
        }
    }
    ~UreController(){
        client_.terminate();
        server_.terminate();
        delete robot_;
    }
    void mainLoop()
    {
        for (int_fast64_t loop_count = 0;; ++loop_count)
        {
        }
    }

private:
    webots::Robot* robot_;
    std::vector<std::shared_ptr<webots::Motor>> motors_;
    std::vector<webots::PositionSensor*> sensors_;

    Citbrains::Udpsocket::UDPClient client_;
    Citbrains::Udpsocket::UDPServer server_;
    void receiveDataHandler(std::string &&data){
        std::string s(std::move(data));
        UreMessage::JointDegree degrees;
        degrees.ParseFromString(s);
        return;
    }
};

int main(int argc, char const *argv[])
{
    UreController ure_controller;
    ure_controller.mainLoop();
    return 0;
}
