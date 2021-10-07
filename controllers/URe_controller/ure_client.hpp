#ifndef URE_CLIENT_H
#define URE_CLIENT_H
#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <cassert>
#include "sUDPSocket.hpp"
#include "ure.pb.h"
#include "ure_controller_constants.h"

using namespace UreController_constants;
class UreClient
{
public:
    void send(std::vector<float> arm, std::vector<float> finger);
    std::vector<float> getSensorData();
    UreClient() : udpclient_("127.0.0.1", DEGREE_RECEIVE_PORT, Citbrains::Udpsocket::SocketMode::unicast_mode),
                  udpserver_(SENSOR_SEND_PORT,std::bind(&UreClient::receiveHandler, this, std::placeholders::_1)  
                  , Citbrains::Udpsocket::SocketMode::unicast_mode)
    {
        sensor_data_list_.resize(static_cast<size_t>(SensorNum::length));
    }
    ~UreClient()
    {
        udpclient_.terminate();
        udpserver_.terminate();
    }

private:
    Citbrains::Udpsocket::UDPClient udpclient_;
    Citbrains::Udpsocket::UDPServer udpserver_;
    std::vector<float> sensor_data_list_; //副作用あり
    std::mutex sensor_mutex_;
    void receiveHandler(std::string &&data);
    void send(const std::vector<float> &arm, const std::vector<float> &finger);
};

void UreClient::send(const std::vector<float> &arm, const std::vector<float> &finger)
{
    assert(static_cast<int32_t>(ArmNum::length) == arm.size());       //必ずジョイント分送る
    assert(static_cast<int32_t>(FingerNum::length) == finger.size()); //必ずジョイント分送る
    UreMessage::JointDegree degrees;
    for (const auto &arm_deg : arm)
    {
        degrees.add_arm_degree_list(arm_deg);
    }
    for (const auto &finger_deg : finger)
    {
        degrees.add_finger_degree_list(finger_deg);
    }
    std::string s = degrees.SerializeAsString();
    udpclient_.send(std::move(s));
    return;
}

std::vector<float> UreClient::getSensorData(){
    std::lock_guard lk(sensor_mutex_);
    return sensor_data_list_;
}

void UreClient::receiveHandler(std::string &&data)
{
    UreMessage::SensorData sensor;
    sensor.ParseFromString(std::move(data));
    {
        std::lock_guard lk(sensor_mutex_);
        for (int32_t i = 0; i < static_cast<int32_t>(SensorNum::length); ++i)
        {
            sensor_data_list_[i] = sensor.sensor_data_list(i);
        }
    }
    return;
}
#endif