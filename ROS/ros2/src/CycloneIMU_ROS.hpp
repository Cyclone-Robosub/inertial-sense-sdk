#include <iostream>

#include "../shared/include/inertial_sense_ros.h"
#include "custom_interfaces/msg/imu.hpp"
#include "rclcpp/rclcpp/publisher.hpp"
#include "rclcpp/rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp/time.hpp"
#include "rclcpp/rclcpp/timer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
std::string params =
    "port: [/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_205D36773430-if00]\nbaudrate: 921600\nenable_log: false\npublishTf: true                                 # Publish "
    "Transform Frame (TR)\nframe_id: \"\"                                    # FIXME: What is this?  is it just the FrameID to use in the ROS messages?\nmag_declination: 0.0\nref_lla: [0.0, 0.0, "
    "0.0]\n# factory_reset: true                             # Reset flash config to factory defaults on startup.\n\n# Hardware platform specifying the IMX carrier board type (i.e. EVB-2) and "
    "configuration bits (see ePlatformConfig).  The platform type is used to simplify the GPS and I/O configuration process.\n# platformConfig: 1                               # "
    "PLATFORM_CFG_TYPE_NONE_ONBOARD_M8\n# platform: 10                                    # PLATFORM_CFG_TYPE_RUG3_G2\n# platformConfig: 11                              # "
    "PLATFORM_CFG_TYPE_EVB2_G2\n# platform: 15                                    # PLATFORM_CFG_TYPE_IG1_G2\n\n# Overwrite IMX ioConfig (see eIoConfig)\n# ioConfig: 0x00112044\n# ioConfig: "
    "0x26CA060       # EVB2: GPS1 Ser1 F9P, GPS2 Ser2 F9P, PPS G8\n# ioConfig: 0x025CA060      # EVB2: GPS1 Ser1 F9P, GPS2 Ser0 F9P, PPS G8\n# ioConfig: 0x0244a060      # EVB2: GPS1 Ser1 F9P, GPS2 "
    "disabled F9P, PPS G8\n\n# Service Points - should we allow these?  Or should they always be hardcoded?\n# service_endpoints:\n  # REF_LLA: \"set_refLLA\"                         # /value "
    "/current\n  # MAG_CAL: \"mag_cal\"                            # /single /multi\n  # FIRMWARE: \"firmware\"                          # /get /update\n\n# Publish the firmware upgrade "
    "progress/status\nfirmware_status:\n  message:\n    topic: \"firmware_status\"\n    enable: true\n\ntransform_frame:\n  # static_transform:\n  message:\n    topic: \"\"\n    enable: "
    "true\n\nins:\n  rotation: [0, 0, 0]                           # Rotation in radians about the X,Y,Z axes from Sensor Frame to Intermediate Output Frame.  Order applied: Z,Y,X.\n  offset: [0, 0 "
    ",0]                             # X,Y,Z offset in meters from Intermediate Output Frame to INS Output Frame.\n  # navigation_dt_ms: 16                          # EKF update period.  IMX-5:  16 "
    "default, 8 max.  Use `msg/ins.../period` to reduce INS output data rate.\n  navigation_dt_ms: 16                           # EKF update period.  uINS-3: 4  default, 1 max.  Use "
    "`msg/ins.../period` to reduce INS output data rate.\n  dynamic_model: 8                              # FIXME: these should be named types/srtring (PORTABLE, STATIONARY, AGV, UAV, etc)\n  "
    "enable_covariance: false                      # Include covariance data in odom_ins_... messages\n  messages:\n    odom_ins_ned:\n      topic: \"odom_ins_ned\"\n      enable: true\n             "
    "                  #";
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
    rclcpp::spin(IMUROS.nh_);
}
class CycloneIMU_ROS : public rclcpp::Node {
   public:
    CycloneIMU_ROS() : Node("IMU_ROS_helper_node") {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&CycloneIMU_ROS::imu_callback, this, std::placeholders::_1));

        mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>("mag", 10, std::bind(&CycloneIMU_ROS::mag_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom_ned", 10, std::bind(&CycloneIMU_ROS::odom_callback, this, std::placeholders::_1));

        std::thread inertialSenseROS(SetupInertialSenseROS);
        inertialSenseROS.detach();

    }

   private:
    void imu_callback(const std::shared_ptr<const sensor_msgs::msg::Imu> &msg);
    void mag_callback(const std::shared_ptr<const sensor_msgs::msg::MagneticField> &msg);
    void odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
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
    float somerandomValue = 0.0;
    // Magnetic field data members
    double mag_field_x = 0.0;
    double mag_field_y = 0.0;
    double mag_field_z = 0.0;
};
