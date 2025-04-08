#include <iostream>
#include "../shared/include/inertial_sense_ros.h"
#include "rclcpp/rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp/timer.hpp"
#include "rclcpp/rclcpp/time.hpp"
#include "rclcpp/rclcpp/publisher.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"


class CycloneIMU_ROS : public rclcpp::Node {
    public:
    CycloneIMU_ROS() : Node("IMU_ROS_helper_node") {
      std::cout << "here" << std::endl;
      int random = 5;
      char ** random1;
      rclcpp::init(random, random1);
        std::cout << "Found InitIMU.yaml" << std::endl;
    std::string IMUyamlconfig = "topic: \"inertialsense\"\n"
                       "port: [/dev/ttyACM1]\n"
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
      YAML::Node config = YAML::Load(IMUyamlconfig);
      if(config.IsDefined()){
      InertialSenseROS IMUROS(config);
      IMUROS.initialize();
      }else{
        std::cerr << "problem with yaml for ROS." << std::endl;
      }
    }

    
    bool started = false;
     float angular_velocity_x;
     float angular_velocity_y;
     float angular_velocity_z;
     float linear_acceleration_x;
     float linear_acceleration_y;
     float linear_acceleration_z;
    void IMURead(const sensor_msgs::msg::Imu &imu);
    private:
    
};