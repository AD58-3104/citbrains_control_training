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
#include "ure_controller_constants.h"
using namespace UreController_constants;

class UreController
{
public:

    UreController() : client_("127.0.0.1", SENSOR_SEND_PORT, Citbrains::Udpsocket::SocketMode::unicast_mode)
    {
        std::cout << "-----------constructing now ..............." << std::endl;
        robot_ = new webots::Robot();
        timestep_ = (int)robot_->getBasicTimeStep();
        std::cout << "wip2\n";
        for (const auto &itr : joint_name_list)
        {
            joint_motors_.emplace_back(robot_->getMotor(itr));
        }
        for (const auto &itr : finger_name_list)
        {
            finger_motors_.emplace_back(robot_->getMotor(itr));
        }
        // for (const auto &itr : sensor_name_list)
        // {
        //     sensors_.push_back(robot_->getPositionSensor(itr));
        // }
        auto handler = [this](std::string &&data)
        {
            // printf("-------------------received !!--------------------");
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
                cnt = 0;
                for (const auto &deg : degrees.finger_degree_list())
                {
                    finger_motors_[cnt]->setPosition(deg);
                    ++cnt;
                }
                // printf("degree received !!!!");
                robot_->step(timestep_);
            }
            return;
        };
        printf("end constructing \n");
        server_ = std::make_unique<Citbrains::Udpsocket::UDPServer>(DEGREE_RECEIVE_PORT, handler, Citbrains::Udpsocket::SocketMode::unicast_mode);
        printf("end2\n");
    }
    ~UreController()
    {
        client_.terminate();
        server_->terminate();
        delete robot_;
    }
    void mainLoop()
    {
        printf("into loop......\n");
        for (int_fast64_t loop_count = 0; true; ++loop_count)
        {
            // if (loop_count % 10 == 0)
            //     printf("loop");
            // std::lock_guard lock(step_mutex_);
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
    int32_t timestep_;
    std::unique_ptr<Citbrains::Udpsocket::UDPServer> server_;
    void receiveDataHandler(std::string &&data)
    {
        printf("-------------------received !!--------------------");
        std::string s(std::move(data));
        UreMessage::JointDegree degrees;
        degrees.ParseFromString(s);
        {
            // std::lock_guard lock(step_mutex_);
            int32_t cnt = 0;
            for (const auto &deg : degrees.arm_degree_list())
            {
                joint_motors_[cnt]->setPosition(deg);
                ++cnt;
            }
            cnt = 0;
            for (const auto &deg : degrees.finger_degree_list())
            {
                finger_motors_[cnt]->setPosition(deg);
                ++cnt;
            }
            // printf("degree received !!!!");
            stepOne();
        }
        return;
    }
    void stepOne()
    {
        //TODO sensorの値のやつ書く
        // printf("step\n");
        // std::lock_guard lock(step_mutex_);
        // robot_->step(timestep_);
    }
};

int main(int argc, char const *argv[])
{
    UreController ure_controller;
    ure_controller.mainLoop();
    return 0;
}
