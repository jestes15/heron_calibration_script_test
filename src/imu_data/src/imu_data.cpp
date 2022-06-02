#include <ros/ros.h>
#include <imu_data/imu_data.h>

#include <boost/array.hpp>

#include <chrono>
#include <random>

int main(int argc, char **argv)
{
    ROS_INFO("Service Starting....");
    ros::init(argc, argv, "imudata");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<imu_data::imu_data>("/imu/data", 1000);
    ros::Rate loop_rate(91);

    boost::array<double, 9> arr = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

    std::random_device gen;
    std::uniform_int_distribution<int> dist(-999, 999);

    int count = 0;
    while (ros::ok())
    {
        imu_data::imu_data data;
        data.header.seq = static_cast<uint32_t>(count);
        data.header.stamp.sec = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        data.header.stamp.nsec = data.header.stamp.toNSec();
        data.header.frame_id = "imu_link";

        data.orientation.x = static_cast<double>(dist(gen)) / 10000.0;
        data.orientation.y = static_cast<double>(dist(gen)) / 10000.0;
        data.orientation.z = static_cast<double>(dist(gen)) / 10000.0;
        data.orientation.w = static_cast<double>(dist(gen)) / 10000.0;

        data.orientation_covariance.fill(0.0);

        data.angular_velocity.x = static_cast<double>(dist(gen)) / 10000.0;
        data.angular_velocity.y = static_cast<double>(dist(gen)) / 10000.0;
        data.angular_velocity.z = static_cast<double>(dist(gen)) / 10000.0;

        data.angular_velocity_covariance = arr;

        data.linear_acceleration.x = static_cast<double>(dist(gen)) / 10000.0;
        data.linear_acceleration.y = static_cast<double>(dist(gen)) / 10000.0;
        data.linear_acceleration.z = -9.75 + static_cast<double>(dist(gen)) / 100000.0;

        data.angular_velocity_covariance = arr;

        chatter_pub.publish(data);

        loop_rate.sleep();
        ++count;
    }
    return 0;
}