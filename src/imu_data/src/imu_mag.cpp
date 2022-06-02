#include <ros/ros.h>
#include <imu_data/imu_mag.h>

#include <chrono>
#include <random>

int main(int argc, char **argv)
{
    ROS_INFO("Service Starting....");
    ros::init(argc, argv, "imumag");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<imu_data::imu_mag>("/imu/mag", 1000);
    ros::Rate loop_rate(7000);

    std::random_device gen;
    std::uniform_int_distribution<int> dist(-999, 999);

    int count = 0;
    while (ros::ok())
    {
        imu_data::imu_mag data;
        data.header.seq = static_cast<uint32_t>(count);
        data.header.stamp.sec = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        data.header.stamp.nsec = data.header.stamp.toNSec();
        data.header.frame_id = "imu_link";

        data.magnetic_field.x = static_cast<double>(dist(gen)) / 10000.0;
        data.magnetic_field.y = static_cast<double>(dist(gen)) / 10000.0;
        data.magnetic_field.z = static_cast<double>(dist(gen)) / 10000.0;

        data.magnetic_field_covariance.fill(0.0);

        chatter_pub.publish(data);

        loop_rate.sleep();
        ++count;
    }
    return 0;
}