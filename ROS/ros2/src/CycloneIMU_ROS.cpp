#include "CycloneIMU_ROS.hpp"

#include <iostream>
std::chrono::time_point startTime = std::chrono::steady_clock::now();
int main(int argc, char* argv[]) {
    std::cout << "Start of Cyclone Robosub IMU API" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CycloneIMU_ROS>();
    startTime = std::chrono::steady_clock::now();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
int imucount = 0;
bool isBenchmarkCompletedIMU = false;
void CycloneIMU_ROS::imu_callback(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
    imucount++;
    angular_velocity_x = msg->angular_velocity.x;
    angular_velocity_y = msg->angular_velocity.y;
    angular_velocity_z = msg->angular_velocity.z;
    linear_acceleration_x = msg->linear_acceleration.x;
    linear_acceleration_y = msg->linear_acceleration.y;
    linear_acceleration_z = msg->linear_acceleration.z;

    std::cout << "IMU Data:" << std::endl;
    std::cout << "  Orientation: x=" << msg->orientation.x << " y=" << msg->orientation.y << " z=" << msg->orientation.z << " w=" << msg->orientation.w << std::endl;
    std::cout << "  Angular Velocity: x=" << msg->angular_velocity.x << " y=" << msg->angular_velocity.y << " z=" << msg->angular_velocity.z << std::endl;
    std::cout << "  Linear Acceleration: x=" << msg->linear_acceleration.x << " y=" << msg->linear_acceleration.y << " z=" << msg->linear_acceleration.z << std::endl;
    std::cout << "---" << std::endl;
    if (imucount > 100 && isBenchmarkCompletedIMU == false) {
        /* something which might take time */

        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(endTime - startTime);
        std::cout << "It took " << duration.count() << " seconds.\n";
        isBenchmarkCompletedIMU = true;
    }
}
void CycloneIMU_ROS::mag_callback(const std::shared_ptr<const sensor_msgs::msg::MagneticField>& msg) {
    mag_field_x = msg->magnetic_field.x;
    mag_field_y = msg->magnetic_field.y;
    mag_field_z = msg->magnetic_field.z;
}
int odomCount = 0;
void CycloneIMU_ROS::odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
    somerandomValue = msg->twist.twist.linear.x;
    if (odomCount > 100) {
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(endTime - startTime);
        std::cout << "It took ODOM " << duration.count() << " seconds.\n";
        if (isBenchmarkCompletedIMU) {
            std::cout << "Fail to catach up to IMU" << std::endl;
        }
    }
}
void CycloneIMU_ROS::pressure_callback(const std::shared_ptr<sensor_msgs::msg::FluidPressure> msg){

}