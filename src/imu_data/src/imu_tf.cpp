#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <imu_data/imu_tf.h>

#include <chrono>
#include <random>

int main(int argc, char **argv)
{

    ROS_INFO("argc: [%d]", argc);
    for (int i = 0; i < argc; ++i)
        ROS_INFO("argv[%d] = %s", i, argv[i]);

    ROS_INFO("Service Starting....");
    ros::init(argc, argv, "imutf");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<imu_data::imu_tf>("/tf", 1000);
    ros::Rate loop_rate(90);

    std::random_device gen;
    std::uniform_int_distribution<int> dist(-999, 999);

    int count = 0;
    while (ros::ok())
    {
        imu_data::imu_tf data;

        geometry_msgs::TransformStamped data_v2;

        data_v2.header.seq = static_cast<uint32_t>(count);
        data_v2.header.stamp.sec = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        data_v2.header.stamp.nsec = data_v2.header.stamp.toNSec();
        data_v2.header.frame_id = "odom";

        data_v2.child_frame_id = "imu_link";

        data_v2.transform.translation.x = static_cast<double>(dist(gen)) / 10000.0;
        data_v2.transform.translation.y = static_cast<double>(dist(gen)) / 10000.0;
        data_v2.transform.translation.z = static_cast<double>(dist(gen)) / 10000.0;

        data_v2.transform.rotation.x = static_cast<double>(dist(gen)) / 10000.0;
        data_v2.transform.rotation.y = static_cast<double>(dist(gen)) / 10000.0;
        data_v2.transform.rotation.z = static_cast<double>(dist(gen)) / 10000.0;
        data_v2.transform.rotation.w = static_cast<double>(dist(gen)) / 10000.0;

        data.transforms.push_back(data_v2);

        chatter_pub.publish(data);

        loop_rate.sleep();
        ++count;
    }
    return 0;
}