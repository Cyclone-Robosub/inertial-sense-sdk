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

void CycloneIMU_ROS::mag_callback(std::shared_ptr<sensor_msgs::msg::MagneticField> msg) {
    std::unique_lock<std::mutex> lock(mag_mutex);
    mag_ptr = std::make_shared<sensor_msgs::msg::MagneticField>(*msg);
    mag_field_x = msg->magnetic_field.x;
    mag_field_y = msg->magnetic_field.y;
    mag_field_z = msg->magnetic_field.z;
    lock.unlock();
}

void CycloneIMU_ROS::odom_callback(std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    std::unique_lock<std::mutex> lock(odom_mutex);
    odom_ptr = std::make_shared<nav_msgs::msg::Odometry>(*msg);
    somerandomValue = msg->twist.twist.linear.x;
    lock.unlock();

    odomCount++;
    if (odomCount > 100) {
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(endTime - startTime);
        std::cout << "It took ODOM " << duration.count() << " seconds.\n";
        if (isBenchmarkCompletedIMU) {
            std::cout << "Fail to catch up to IMU" << std::endl;
        }
    }
}

void CycloneIMU_ROS::pressure_callback(std::shared_ptr<sensor_msgs::msg::FluidPressure> msg) {
    std::unique_lock<std::mutex> lock(pressure_mutex);
    pressure_ptr = msg;
    pressure = msg->fluid_pressure;
    lock.unlock();
}

void CycloneIMU_ROS::Controls_Publisher(){
    while(true){
        auto start_time_publishing = std::chrono::steady_clock::now();
        auto predicted_time = start_time_publishing + std::chrono::milliseconds(10);
        std::unique_lock<std::mutex> lock_imu(imu_mutex);
        if(imu_ptr){
            custom_msg.imu_fusion = *imu_ptr;
        }
        lock_imu.unlock();
        if(mag_ptr)
        {
            std::unique_lock<std::mutex> lockmag(mag_mutex);
            if (mag_ptr) {
                custom_msg.mag_array = *mag_ptr;
                RCLCPP_INFO(this->get_logger(),
                    "Added Magnetometer data: Magnetic Field (x: %.4f, y: %.4f, z: %.4f)",
                    mag_ptr->magnetic_field.x, mag_ptr->magnetic_field.y, mag_ptr->magnetic_field.z);
            } else {
               std::cout <<"Mag values not set." << std::endl;
            }
            lockmag.unlock();
        }

       if(pressure_ptr) {
            std::unique_lock<std::mutex> lockpressure(pressure_mutex);
            if (pressure_ptr) {
                custom_msg.pressure = *pressure_ptr;
                RCLCPP_INFO(this->get_logger(),
                    "Added Pressure data: %.4f Pa",
                    pressure_ptr->fluid_pressure);
            } else {
                custom_msg.pressure.fluid_pressure = pressure;
                RCLCPP_INFO(this->get_logger(),
                    "Using default Pressure value: %.4f Pa",
                    pressure);
            }
            lockpressure.unlock();
        }

    
       if(odom_ptr) {
            std::unique_lock<std::mutex> lock_odom(odom_mutex);
            if (odom_ptr) {
                custom_msg.ahrs_database = *odom_ptr;
                RCLCPP_INFO(this->get_logger(),
                    "Added Odometry data: Position (x: %.4f, y: %.4f, z: %.4f), "
                    "Velocity (x: %.4f, y: %.4f, z: %.4f)",
                    odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y, odom_ptr->pose.pose.position.z,
                    odom_ptr->twist.twist.linear.x, odom_ptr->twist.twist.linear.y, odom_ptr->twist.twist.linear.z);
            }
            lock_odom.unlock();
        }

        imu_publisher->publish(custom_msg);
        RCLCPP_INFO(this->get_logger(), "Published combined sensor message");
        std::this_thread::sleep_until(predicted_time);
    }
}