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
void CycloneIMU_ROS::imu_callback(std::shared_ptr<sensor_msgs::msg::Imu> msg) {
    imucount++;
    std::unique_lock<std::mutex> lock(imu_mutex);
    imu_ptr = msg;
    lock.unlock();
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

void CycloneIMU_ROS::Controls_Publisher(){
    while(true){
        auto start_time_publishing = std::chrono::steady_clock::now();
        auto predicted_time = start_time_publishing + std::chrono::milliseconds(10);
        std::unique_lock<std::mutex> lock(imu_mutex);
        if(imu_ptr){
            custom_msg.imu_fusion = *imu_ptr;
        }
        lock.unlock();
        imu_publisher->publish(custom_msg);
        std::this_thread::sleep_until(predicted_time);

    }
}