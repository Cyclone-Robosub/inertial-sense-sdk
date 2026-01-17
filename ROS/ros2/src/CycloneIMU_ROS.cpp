#include "CycloneIMU_ROS.hpp"
#include <iostream>
   std::unique_ptr<CycloneIMU_ROS> IMU_ROS_obj = std::make_unique<CycloneIMU_ROS>();
void CycloneIMU_ROS::IMURead(const sensor_msgs::msg::Imu &imu){
    angular_velocity_x = imu.angular_velocity.x;
    angular_velocity_y = imu.angular_velocity.y;
    angular_velocity_z = imu.angular_velocity.z;
    linear_acceleration_x = imu.linear_acceleration.x;
    linear_acceleration_y = imu.linear_acceleration.y;
    linear_acceleration_z = imu.linear_acceleration.z;
    std::cout << angular_velocity_x << std::endl;
}
int main(){
    std::cout << "Here" << std::endl;
    CycloneIMU_ROS startup();
    return 0;
}