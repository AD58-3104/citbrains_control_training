#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "sUDPSocket.hpp"
#include "ure.pb.h"

class UreController
{
public:
    enum FingerNum //TODO この辺ヘッダにして共通の場所に置いておくべき
    {
        palm_finger_1_joint = 0,
        finger_1_joint_1,
        finger_1_joint_2,
        finger_1_joint_3,
        palm_finger_2_joint,
        finger_2_joint_1,
        finger_2_joint_2,
        finger_2_joint_3,
        finger_middle_joint_1,
        finger_middle_joint_2,
        finger_middle_joint_3
    };
    enum ArmNum
    {
        shoulder_pan_joint = 0,
        shoulder_lift_joint,
        elbow_joint,
        wrist_1_joint,
        wrist_2_joint,
        wrist_3_joint
    };
    enum SensorNum
    {
        elbow_joint_sensor = 0,
        shoulder_lift_joint_sensor,
        shoulder_pan_joint_sensor,
        wrist_1_joint_sensor,
        wrist_2_joint_sensor,
        wrist_3_joint_sensor
    };
    const std::vector<std::string> joint_name_list{
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    const std::vector<std::string> sensor_name_list{
        "shoulder_pan_joint_sensor", "shoulder_lift_joint_sensor", "elbow_joint_sensor", "wrist_1_joint_sensor", "wrist_2_joint_sensor", "wrist_3_joint_sensor"};
    const std::vector<std::string> finger_name_list{
        "finger_1_joint_1",
        "finger_2_joint_1",
        "finger_middle_joint_1"
        };
    static const int32_t TIME_STEP = 16;
    inline static constexpr int32_t RECEIVE_PORT = 7650;
    inline static constexpr int32_t SEND_PORT = 7651;
    UreController() : client_("127.0.0.1", SEND_PORT, Citbrains::Udpsocket::SocketMode::unicast_mode), server_(RECEIVE_PORT, std::bind(&UreController::receiveDataHandler, this, std::placeholders::_1), Citbrains::Udpsocket::SocketMode::unicast_mode)
    {
        robot_ = new webots::Robot();
        for (const auto &itr : joint_name_list)
        {
            joint_motors_.emplace_back(robot_->getMotor(itr));
        }
        for (const auto &itr : finger_name_list)
        {
            finger_motors_.emplace_back(robot_->getMotor(itr));
        }
        for (const auto &itr : sensor_name_list)
        {
            sensors_.push_back(robot_->getPositionSensor(itr));
        }
    }
    ~UreController()
    {
        client_.terminate();
        server_.terminate();
        delete robot_;
    }
    void mainLoop()
    {
        for (int_fast64_t loop_count = 0;; ++loop_count)
        {
            std::lock_guard lock(step_mutex_);
            stepOne();
        }
    }

private:
    webots::Robot *robot_;
    std::vector<std::shared_ptr<webots::Motor>> joint_motors_;
    std::vector<std::shared_ptr<webots::Motor>> finger_motors_;
    std::vector<webots::PositionSensor *> sensors_;
    std::mutex step_mutex_;
    Citbrains::Udpsocket::UDPClient client_;
    Citbrains::Udpsocket::UDPServer server_;
    void receiveDataHandler(std::string &&data)
    {
        std::string s(std::move(data));
        UreMessage::JointDegree degrees;
        degrees.ParseFromString(s);
        {
            std::lock_guard lock(step_mutex_);
            int32_t cnt = 0;
            for (const auto &deg : degrees.arm_degree_list())
            {
                joint_motors_[cnt]->setPosition(deg);
                ++cnt;
            }
            for (const auto &deg : degrees.finger_degree_list())
            {
                finger_motors_[cnt]->setPosition(deg);
                ++cnt;
            }
        }
        return;
    }
    void stepOne()
    {
        //TODO sensorの値のやつ書く
        robot_->step(TIME_STEP);
    }
};

int main(int argc, char const *argv[])
{
    UreController ure_controller;
    ure_controller.mainLoop();
    return 0;
}
