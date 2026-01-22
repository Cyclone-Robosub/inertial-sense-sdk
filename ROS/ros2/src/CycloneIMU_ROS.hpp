
#include <iostream>

#include "../shared/include/inertial_sense_ros.h"
#include "custom_interfaces/msg/imu.hpp"
#include "rclcpp/rclcpp/publisher.hpp"
#include "rclcpp/rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp/time.hpp"
#include "rclcpp/rclcpp/timer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
std::string params = "topic: \"inertialsense\"\n"
                       "port: [/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_205D36773430-if00]\n"
                       "baudrate: 921600\n"
                       "\n"
                       "ins:\n"
                       "  navigation_dt_ms: 16                          # EKF update period.  uINS-3: 4  default, 1 max.  Use `msg/ins.../period` to reduce INS output data rate."
                       "\n"
                       "sensors:\n"
                       "  messages:  \n"
                       "    pimu:             # Publish preintegrated IMU delta theta and delta velocity\n"
                       "      topic: \"pimu\"\n"
                       "      enable: true\n"
                       "      period: 1\n"
                       "    imu:              # Publish IMU angular rates and linear acceleration \n"
                       "      topic: \"imu\"\n"
                       "      enable: \"true\"\n"
                       "      period: 1\n";
YAML::Node nodesomething;
void SetupInertialSenseROS() {
    try {
        nodesomething = YAML::Load(params);
    } catch (const YAML::BadFile& bf) {
        //    std::cout << "Loading file \"" << paramYamlPath << "\" failed.  Using default parameters.\n\n";
        nodesomething = YAML::Node(YAML::NodeType::Undefined);
    }
    InertialSenseROS IMUROS = InertialSenseROS(nodesomething);
    IMUROS.initialize();
    while(true){ 
    rclcpp::spin_some(IMUROS.nh_);
    IMUROS.update();
    }
}
class CycloneIMU_ROS : public rclcpp::Node {
   public:
    CycloneIMU_ROS() : Node("IMU_ROS_helper_node") {
        callbackSensors = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto commandOptions = rclcpp::SubscriptionOptions();
        commandOptions.callback_group = callbackSensors;

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&CycloneIMU_ROS::imu_callback, this, std::placeholders::_1), commandOptions );

        mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>("mag", 10, std::bind(&CycloneIMU_ROS::mag_callback, this, std::placeholders::_1), commandOptions );

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom_ned", 10, std::bind(&CycloneIMU_ROS::odom_callback, this, std::placeholders::_1), commandOptions );
        pressure_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>("pressure", 10, std::bind(&CycloneIMU_ROS::pressure_callback, this, std::placeholders::_1), commandOptions );
        std::thread inertialSenseROS(SetupInertialSenseROS);
        inertialSenseROS.detach();
    }

   private:
    void imu_callback(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg);
    void mag_callback(const std::shared_ptr<const sensor_msgs::msg::MagneticField>& msg);
    void odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg);
    void pressure_callback(const std::shared_ptr<sensor_msgs::msg::FluidPressure> msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<custom_interfaces::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub;
    rclcpp::CallbackGroup::SharedPtr callbackSensors;

    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
    float somerandomValue = 0.0;
    // Magnetic field data members
    double mag_field_x = 0.0;
    double mag_field_y = 0.0;
    double mag_field_z = 0.0;
    double pressure = 0.0;
};
