#include <iostream>
#include "../shared/include/inertial_sense_ros.h"
#include "rclcpp/rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp/timer.hpp"
#include "rclcpp/rclcpp/time.hpp"
#include "rclcpp/rclcpp/publisher.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "custom_interfaces/msg/imu.hpp"

void SetupInertialSenseROS(){
        InertialSenseROS IMUROS = InertialSenseROS();
        IMUROS.initialize();
        rclcpp::spin(IMUROS.nh_);
    }
class CycloneIMU_ROS : public rclcpp::Node {
    public:
    CycloneIMU_ROS() : Node("IMU_ROS_helper_node") {
      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, 
        std::bind(&CycloneIMU_ROS::imu_callback, this, std::placeholders::_1));
    
    mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "mag", 10, 
        std::bind(&CycloneIMU_ROS::mag_callback, this, std::placeholders::_1));
        
      std::thread inertialSenseROS(SetupInertialSenseROS);
      inertialSenseROS.detach();
      
    }
    private:
      void imu_callback(const std::shared_ptr<const sensor_msgs::msg::Imu> &msg);
      void mag_callback(const std::shared_ptr<const sensor_msgs::msg::MagneticField> &msg);
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
      rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
      rclcpp::Publisher<custom_interfaces::msg::Imu>::SharedPtr imu_publisher;

   double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
    
      // Magnetic field data members
    double mag_field_x = 0.0;
    double mag_field_y = 0.0;
    double mag_field_z = 0.0;
};