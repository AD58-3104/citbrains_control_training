#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <cassert>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "sUDPSocket.hpp"
#include "ure.pb.h"
#include "ure_controller_constants.h"

class UreClient
{
public:
    void send(std::vector<float> arm, std::vector<float> finger);
    UreClient() : udpclient_("127.0.0.1", DEGREE_RECEIVE_PORT, Citbrains::Udpsocket::SocketMode::unicast_mode),
                  udpserver_(SENSOR_SEND_PORT, std::bind(&UreClient::receiveHandler, this, std::placeholders::_1), Citbrains::Udpsocket::SocketMode::unicast_mode)
    {
        sensor_data_list.resize(SensorNum::length);
    }
    ~UreClient(SENSOR_SEND_PORT, std::bind(), )
    {
        udpclient_.terminate();
        udpserver_.terminate();
    }

private:
    Citbrains::Udpsocket::UDPClient udpclient_;
    Citbrains::Udpsocket::UDPClient udpserver_;
    std::vector<float> sensor_data_list_; //副作用あり
    std::mutex sensor_mutex_;
    void receiveHandler(std::string &&data);
};

void UreClient::send(const std::vector<float> &arm, const std::vector<float> &finger)
{
    assert(ArmNum::length == arm.length());       //必ずジョイント分送る
    assert(FingerNum::length == finger.length()); //必ずジョイント分送る
    UreMessage::JointDegree degrees;
    for (const auto &arm_deg : arm)
    {
        degrees.add_degree_list(arm_deg);
    }
    for (const auto &finger_deg : finger)
    {
        degrees.finger_degree_list(finger_deg);
    }
    std::string s = degrees.SerializeAsString();
    udpclient_.send(std::move(s));
    return;
}

void receiveHandler(std::string &&data)
{
    UreMessage::SensorData senser;
    sensor.ParseFromString(std::move(data));
    {
        std::lock_guard lk(sensor_mutex_);
        for(size_t i = 0;i <  SensorNum::length;++i){
            sensor_data_list_[i] = sensor.sensor_data_list(i);
        }
    }
    return;
}