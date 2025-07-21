#include <gz/sim/System.hh>
#include "../data.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <gz/sim/Joint.hh>
#include <unitree/idl/hg/LowState_.hpp>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/clock.pb.h>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <gz/transport.hh>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

using namespace unitree::common;
using namespace unitree::robot;

namespace gz_unitree
{
    class UnitreePlugin :
        // This class is a system.
        public gz::sim::System,
        public gz::sim::ISystemPostUpdate,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemConfigure
    {
    private:
        void CmdHandler(const void *msg);
        void LowStateWriter();
        void IMUHandler(const gz::msgs::IMU &_msg);
        void TickHandler(const gz::msgs::Clock &_msg);

    private:
        // Buffers
        DataBuffer<MotorState> motor_state_buffer;
        DataBuffer<MotorCommand> motor_command_buffer;
        DataBuffer<ImuState> imu_state_buffer;


        // Map joints names to gz::sim::Joint entities
        std::unordered_map<std::string, gz::sim::Joint> joints;

        // Tick
        unsigned long sim_tick;

        // ECM
        // gz::sim::EntityComponentManager& ecm;
        gz::sim::Entity model_id;

        // Publisher
        ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> state_publisher;
        ThreadPtr publisher_thread;
        DataBuffer<unitree_hg::msg::dds_::LowState_> low_state_buffer;
        uint32_t last_state_crc;

        // Subscriber (to prevent going out of scope)
        ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> cmd_subscriber;
        gz::transport::Node imu_subscriber;
        gz::transport::Node clock_subscriber;

        bool state_sent;
        bool joints_logged;

    public:
        UnitreePlugin();

    public:
        ~UnitreePlugin() override;

    public:
        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    public:
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm) override;

    public:
        void Configure(const gz::sim::Entity &id,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &ecm,
                       gz::sim::EventManager &_eventMgr) override;
    };
}
