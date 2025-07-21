#include "include/system.hpp"
#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Sensor.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/clock.pb.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>
#include "crc32.cc"
#include "data.hpp"
#include "data.cpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#define header "[Unitree] "

using namespace gz_unitree;
using namespace unitree::common;
using namespace unitree::robot;

float GetMotorKp(MotorType type)
{
    switch (type)
    {
    case GearboxS:
        return 80;
    case GearboxM:
        return 100;
    case GearboxL:
        return 200;
    default:
        return 0;
    }
}

float GetMotorKd(MotorType type)
{
    switch (type)
    {
    case GearboxS:
        return 2;
    case GearboxM:
        return 3;
    case GearboxL:
        return 5;
    default:
        return 0;
    }
}

UnitreePlugin::UnitreePlugin()
{
    this->state_sent = false;
    this->joints_logged = false;
    // this->imu_subscriber = gz::transport::Node();
}

UnitreePlugin::~UnitreePlugin()
{
}

void UnitreePlugin::CmdHandler(const void *msg)
{
    GZ_PROFILE("Cmd")
    GZ_PROFILE_THREAD_NAME("unitree__cmd_handler");
    unitree_hg::msg::dds_::LowCmd_ _cmd = *(const unitree_hg::msg::dds_::LowCmd_ *)msg;

    MotorCommand motor_command_tmp;
    for (size_t i = 0; i < H1_NUM_MOTOR; ++i)
    {
        motor_command_tmp.tau_ff[i] = _cmd.motor_cmd()[i].tau();
        motor_command_tmp.q_target[i] = _cmd.motor_cmd()[i].q();
        motor_command_tmp.dq_target[i] = _cmd.motor_cmd()[i].dq();
        motor_command_tmp.kp[i] = _cmd.motor_cmd()[i].kp();
        motor_command_tmp.kd[i] = _cmd.motor_cmd()[i].kd();
    }

    this->motor_command_buffer.SetData(motor_command_tmp);
}

void UnitreePlugin::IMUHandler(const gz::msgs::IMU &_msg)
{
    GZ_PROFILE_THREAD_NAME("unitree__imu_handler");
    GZ_PROFILE("IMU")
    this->imu_state_buffer.Clear();

    double w = _msg.orientation().w();
    double x = _msg.orientation().x();
    double y = _msg.orientation().y();
    double z = _msg.orientation().z();

    ImuState imu_state_tmp;

    imu_state_tmp.quaternion[0] = w;
    imu_state_tmp.quaternion[1] = x;
    imu_state_tmp.quaternion[2] = y;
    imu_state_tmp.quaternion[3] = z;

    imu_state_tmp.accelerometer[0] = _msg.linear_acceleration().x();
    imu_state_tmp.accelerometer[1] = _msg.linear_acceleration().y();
    imu_state_tmp.accelerometer[2] = _msg.linear_acceleration().z();

    imu_state_tmp.rpy[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    imu_state_tmp.rpy[1] = asin(2 * (w * y - z * x));
    imu_state_tmp.rpy[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    // why is angular velocity in x/y/z ???
    // auto gyro = BodyAngularVelocityToEulerRates(
    //     _msg.angular_velocity().x(),
    //     _msg.angular_velocity().y(),
    //     _msg.angular_velocity().z(),
    //     imu_state_tmp.rpy[0],
    //     imu_state_tmp.rpy[1]);

    imu_state_tmp.gyroscope[0] = _msg.angular_velocity().x();
    imu_state_tmp.gyroscope[1] = _msg.angular_velocity().y();
    imu_state_tmp.gyroscope[2] = _msg.angular_velocity().z();

    this->imu_state_buffer.SetData(imu_state_tmp);
}

void UnitreePlugin::TickHandler(const gz::msgs::Clock &_msg)
{
    GZ_PROFILE("Tick")
    GZ_PROFILE_THREAD_NAME("unitree__tick_handler");
    this->sim_tick = _msg.sim().sec() * 1000 + _msg.sim().nsec() / 1000000;
}

void UnitreePlugin::LowStateWriter()
{
    // Make sure we run the thread
    const unitree_hg::msg::dds_::LowState_ *message = this->low_state_buffer.GetData().get();
    if (message && this->last_state_crc != message->crc())
    {
        GZ_PROFILE("Send low state")
        GZ_PROFILE_THREAD_NAME("unitree__low_state_writer");
        this->last_state_crc = message->crc();
        this->state_publisher->Write(*message);
    }
}

void UnitreePlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                               const gz::sim::EntityComponentManager &ecm)
{
}

void UnitreePlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &ecm)
{
    GZ_PROFILE("PreUpdate")
    unitree_hg::msg::dds_::LowState_ lowstate{};
    gz::sim::Model model = gz::sim::Model(this->model_id);

    GZ_PROFILE_BEGIN("Get motor state from joint");
    uint motor_state_index = 0;
    for (std::string joint_name : H1_2JointNames)
    {
        auto joint = this->joints[joint_name];
        auto position = joint.Position(ecm).value();
        auto velocity = joint.Velocity(ecm);

        float angle = 0.0f;
        if (position.size() > 0)
        {
            angle = position.at(0);
        }
        else
        {
            continue;
        }

        lowstate.motor_state().at(motor_state_index).q() = angle;

        if (velocity.has_value() && velocity.value().size() == 1)
        {
            lowstate.motor_state().at(motor_state_index).dq() = velocity.value().at(0);
        }
        else
        {
            lowstate.motor_state().at(motor_state_index).dq() = 0.0f;
        }

        motor_state_index++;
    }
    GZ_PROFILE_END();

    auto imu = this->imu_state_buffer.GetData();

    if (imu)
    {
        GZ_PROFILE("Set IMU state")
        lowstate.imu_state().quaternion().at(0) = imu->quaternion[0];
        lowstate.imu_state().quaternion().at(1) = imu->quaternion[1];
        lowstate.imu_state().quaternion().at(2) = imu->quaternion[2];
        lowstate.imu_state().quaternion().at(3) = imu->quaternion[3];
        lowstate.imu_state().gyroscope().at(0) = imu->gyroscope[0];
        lowstate.imu_state().gyroscope().at(1) = imu->gyroscope[1];
        lowstate.imu_state().gyroscope().at(2) = imu->gyroscope[2];
        lowstate.imu_state().accelerometer().at(0) = imu->accelerometer[0];
        lowstate.imu_state().accelerometer().at(1) = imu->accelerometer[1];
        lowstate.imu_state().accelerometer().at(2) = imu->accelerometer[2];
        lowstate.imu_state().rpy().at(0) = imu->rpy[0];
        lowstate.imu_state().rpy().at(1) = imu->rpy[1];
        lowstate.imu_state().rpy().at(2) = imu->rpy[2];
        lowstate.imu_state().temperature() = imu->temperature;
    }

    {
        GZ_PROFILE("Set tick and CRC")
        lowstate.tick() = this->sim_tick;
        lowstate.crc() = crc32_core((uint32_t *)&lowstate, (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1);
    }

    {
        GZ_PROFILE("Set low state on buffer")
        this->low_state_buffer.SetData(lowstate);
    }

    auto cmdbuf = this->motor_command_buffer.GetData();

    GZ_PROFILE_BEGIN("Set motor command for joint");
    motor_state_index = 0;
    for (std::string joint_name : H1_2JointNames)
    {
        auto joint = this->joints[joint_name];

        if (!cmdbuf || cmdbuf->q_target.size() <= motor_state_index)
        {
            motor_state_index++;
            continue;
        }

        auto force = cmdbuf->tau_ff.at(motor_state_index) +
                     cmdbuf->kp.at(motor_state_index) * (cmdbuf->q_target.at(motor_state_index) - lowstate.motor_state().at(motor_state_index).q()) +
                     cmdbuf->kd.at(motor_state_index) * (cmdbuf->dq_target.at(motor_state_index) - lowstate.motor_state().at(motor_state_index).dq());

        std::vector<double> torque = {force};
        joint.SetForce(ecm, torque);

        motor_state_index++;
    }
    GZ_PROFILE_END();
}

void UnitreePlugin::Configure(const gz::sim::Entity &id,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              gz::sim::EntityComponentManager &ecm,
                              gz::sim::EventManager &_eventMgr)
{
    GZ_PROFILE_THREAD_NAME("unitree__main");
    this->model_id = id;
    ChannelFactory::Instance()->Init(1, "lo");
    // this->ecm = *&ecm;

    gzmsg << header << "Created instance on DDS domain 1 with network interface 'lo'" << std::endl;

    this->state_publisher = ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_>(new ChannelPublisher<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
    this->state_publisher->InitChannel();

    this->publisher_thread =
        CreateRecurrentThreadEx("low_state_writer", UT_CPU_ID_NONE, 500,
                                &UnitreePlugin::LowStateWriter, this);

    gzmsg << header << "Created publisher on channel 'rt/lowstate'" << std::endl;

    this->cmd_subscriber = ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_>(new ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>("rt/lowcmd"));
    this->cmd_subscriber->InitChannel(
        std::bind(&UnitreePlugin::CmdHandler, this, std::placeholders::_1), 1);

    gzmsg << header << "Created subscriber on channel 'rt/lowcmd'" << std::endl;

    gz::sim::Model model = gz::sim::Model(this->model_id);

    model.SelfCollide(ecm);

    for (std::string joint_name : H1_2JointNames)
    {
        gz::sim::Joint joint = gz::sim::Joint(model.JointByName(ecm, joint_name));
        this->joints[joint_name] = joint;
        joint.EnableVelocityCheck(ecm);
        joint.EnablePositionCheck(ecm);
    }

    gzmsg << header << "Enabled velocity and position checking for all of model's joints" << std::endl;

    std::function<void(const gz::msgs::IMU &)> bound_imu_cb = std::bind(&UnitreePlugin::IMUHandler, this, std::placeholders::_1);
    if (!this->imu_subscriber.Subscribe("/imu", bound_imu_cb))
    {
        gzerr << header << "Failed to subscribe to IMU topic" << std::endl;
        return;
    }
    else
    {
        gzmsg << header << "Subscribed to IMU topic" << std::endl;
    }

    std::function<void(const gz::msgs::Clock &)> bound_tick_cb = std::bind(&UnitreePlugin::TickHandler, this, std::placeholders::_1);
    if (!this->clock_subscriber.Subscribe("/clock", bound_tick_cb))
    {
        gzerr << header << "Failed to subscribe to clock topic" << std::endl;
        return;
    }

    this->joints_logged = true;
}

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    UnitreePlugin,
    gz::sim::System,
    UnitreePlugin::ISystemPostUpdate,
    UnitreePlugin::ISystemPreUpdate,
    UnitreePlugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
    UnitreePlugin,
    "gz::sim::systems::UnitreePlugin")
