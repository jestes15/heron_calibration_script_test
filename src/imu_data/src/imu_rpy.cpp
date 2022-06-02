#include <ros/ros.h>
#include <imu_data/imu_rpy.h>

#include <chrono>
#include <random>

int main(int argc, char **argv)
{
    ROS_INFO("Service Starting....");
    ros::init(argc, argv, "imurpy");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<imu_data::imu_rpy>("/imu/rpy", 1000);
    ros::Rate loop_rate(86);

    std::random_device gen;
    std::uniform_int_distribution<int> dist(-999, 999);

    int count = 0;
    while (ros::ok())
    {
        imu_data::imu_rpy data;
        data.header.seq = static_cast<uint32_t>(count);
        data.header.stamp.sec = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        data.header.stamp.nsec = data.header.stamp.toNSec();
        data.header.frame_id = "imu_link";

        data.vector.x = 3.14 + static_cast<double>(dist(gen)) / 100000.0;
        data.vector.y = static_cast<double>(dist(gen)) / 10000.0;
        data.vector.z = -2.3496 + static_cast<double>(dist(gen)) / 100000.0;

        chatter_pub.publish(data);

        loop_rate.sleep();
        ++count;
    }
    return 0;
}