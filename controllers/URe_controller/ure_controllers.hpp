#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include "ure_controller_constants.h"
using namespace UreController_constants;

class UreController
{
public:
    UreController() : robot_(std::make_shared<webots::Robot>()), timestep_((int32_t)robot_->getBasicTimeStep()), loop_count_(0), is_sync_(robot_->getSynchronization())
    {
        std::cout << "-----------constructing now ..............." << std::endl;
        std::cout << "timestep is " << timestep_ << std::endl;
        for (const auto &itr : joint_name_list)
        {
            auto ptr = robot_->getMotor(itr);
            if (ptr == nullptr)
            {
                std::cerr << "ERROR!!! :: can't get joint Motor!!!!!!" << std::endl;
                std::terminate();
            }
            joint_motors_.emplace_back(ptr);
        }
        for (const auto &itr : finger_name_list)
        {
            auto ptr = robot_->getMotor(itr);
            if (ptr == nullptr)
            {
                std::cerr << "ERROR!!! :: can't get finger Motor!!!!!!" << std::endl;
                std::terminate();
            }
            finger_motors_.emplace_back(ptr);
        }
        for (const auto &itr : sensor_name_list)
        {
            auto ptr = robot_->getPositionSensor(itr);
            if (ptr == nullptr)
            {
                std::cerr << "ERROR!!! :: can't get Position Sensor!!!!!!" << std::endl;
                std::terminate();
            }
            ptr->enable(timestep_);
            sensors_.push_back(ptr);
        }
        std::cout << "end constructing !!" << std::endl;
    }
    ~UreController()
    {
    }
    void stepOne()
    {
        ++loop_count_;
        robot_->step(timestep_);
    }
    int_fast64_t getloop_count() const noexcept
    {
        return loop_count_;
    }
    void sendDegreeToJoint(const std::vector<double> &degrees)
    {
        if (degrees.size() != static_cast<size_t>(ArmNum::length))
        {
            std::cerr << "please set degrees for all joints" << std::endl;
            return;
        }
        size_t i = 0;
        for (const auto motor : joint_motors_)
        {
            motor->setPosition(degrees[i]);
            ++i;
        }
    }

    void sendDegreeToFinger(const std::vector<double> &degrees)
    {
        if (degrees.size() != static_cast<size_t>(FingerNum::length))
        {
            std::cerr << "please set degrees for all joints" << std::endl;
            return;
        }
        size_t i = 0;
        for (const auto motor : finger_motors_)
        {
            motor->setPosition(degrees[i]);
            ++i;
        }
    }
    std::vector<double> getMotorDegree(){
        std::vector<double> result;
        for(const auto pos_sensor:sensors_){
            result.push_back(pos_sensor->getValue());
        }
        return result;
    }

private:
    std::shared_ptr<webots::Robot> robot_;
    const int32_t timestep_;
    int_fast64_t loop_count_;
    const bool is_sync_;
    std::vector<std::shared_ptr<webots::Motor>> joint_motors_;
    std::vector<std::shared_ptr<webots::Motor>> finger_motors_;
    std::vector<webots::PositionSensor *> sensors_;
};
